"""
pypowertrain — browser app (Shiny for Python, runs fully client-side under Pyodide).

Interactive port of the desktop Dash GUI (pypowertrain/app.py). The whole compute core
(numpy + scipy) runs in the browser via Pyodide — no server, no HTTP callbacks.

Parity status vs. the Dash app:
  done  : motor / battery / controller / thermal / load controls, library presets,
          property readouts, the 4 heatmap types, all 6 overlay contours,
          off-by-default axis rescaling, negative-axis toggles, resolution,
          zero-axis reference lines, motor geometry cross-section.
  todo  : editable thermal table, target markers.
"""
import numpy as np
import plotly.graph_objects as go

from shiny import App, ui, reactive, render, req
from shinywidgets import output_widget, render_widget

from pypowertrain.system import System, system_limits, system_detect_limits, DummyLoad
from pypowertrain.components.actuator import Actuator
from pypowertrain.components.battery import define_battery
from pypowertrain.library import odrive, grin, moteus


# ---------------------------------------------------------------- presets & base
# callables returning fresh instances (mirrors the dropdowns in the Dash app)
MOTOR_PRESETS = {
    "odrive.D5065_270KV": odrive.D5065_270KV,
    "odrive.M8325s_100KV": odrive.M8325s_100KV,
    "odrive.botwheel": odrive.botwheel,
    "grin.all_axle": grin.all_axle,
    "moteus.mj5208": moteus.mj5208,
}
CONTROLLER_PRESETS = {
    "odrive.s1": odrive.s1,
    "odrive.pro": odrive.pro,
    "odrive.pro_nominal": odrive.pro_nominal,
    "odrive.pro_overclock": odrive.pro_overclock,
    "grin.phaserunner": grin.phaserunner,
    "moteus.n1": moteus.n1,
}
DEFAULT_MOTOR = "grin.all_axle"
DEFAULT_CONTROLLER = "odrive.pro"

_m0 = MOTOR_PRESETS[DEFAULT_MOTOR]()
_c0 = CONTROLLER_PRESETS[DEFAULT_CONTROLLER]()
_base = System(actuator=Actuator(motor=_m0, controller=_c0), battery=define_battery(v=48, wh=1e3))
_INIT_RPM, _INIT_TORQUE = (float(v) for v in system_detect_limits(_base))

PLOT_TYPES = ["Efficiency", "Bus power", "Dissipation", "Acceleration"]
OVERLAYS = ["Thermal", "Field weakening", "Efficiency", "Acceleration", "Open loop", "Saturation"]
DEFAULT_OVERLAYS = ["Thermal", "Field weakening", "Efficiency", "Saturation"]

# default thermal-limit curves (mirrors app.py thermal_specs; editable table deferred)
THERMAL_SPECS = [
    {"color": "yellow", "dt": 5000, "dT": 60, "key": "coils"},
    {"color": "orange", "dt": 60, "dT": 60, "key": "coils"},
    {"color": "red", "dt": 2, "dT": 40, "key": "coils"},
]


# ---------------------------------------------------------------- compute helpers
def build_system(inp):
    """Assemble a System from the current inputs (mirrors app.py compute_handler_system)."""
    motor = MOTOR_PRESETS[inp.motor_preset()]().replace(
        __geometry__turns=int(inp.turns()),
        __geometry__radius_scale=inp.radius(),
        __geometry__slot_depth_scale=inp.slot_depth(),
        __geometry__length_scale=inp.axial(),
        __geometry__slot_width_scale=inp.slot_width(),
        __geometry__reluctance_scale=inp.reluctance(),
        __geometry__frequency_scale=inp.frequency(),
        __geometry__termination=inp.termination(),
        coil_temperature=inp.coil_temp(),
        magnet_temperature=inp.magnet_temp(),
    )
    controller = CONTROLLER_PRESETS[inp.controller_preset()]().replace(
        field_weakening=bool(inp.field_weakening()),
        bus_voltage_limit=inp.bus_voltage(),
        phase_current_limit=inp.phase_current(),
        power_limit=inp.power_limit(),
        freq_limit=inp.freq_limit(),
        modulation_factor=inp.modulation(),
    )
    thermal = motor.thermal.replace(
        conductivity__statorade=inp.statorade(),
        conductivity__vented=inp.vented(),
        conductivity__potted=inp.potted(),
        conductivity__emissivity=inp.emissivity(),
        conductivity__rim_exposure=inp.rim_exp(),
        conductivity__side_exposure=inp.side_exp(),
    )
    battery = _base.battery.replace(
        charge_state=inp.charge(), P=int(inp.batt_p()), S=int(inp.batt_s()),
    )
    load = DummyLoad(inertia=inp.inertia(), drag=inp.drag())
    return _base.replace(
        actuator__n_series=int(inp.n_series()),
        __controller=controller,
        __motor=motor,
        __thermal=thermal,
        load=load,
        battery=battery,
    )


def _json_safe(arr):
    """2D ndarray -> nested lists with NaN mapped to None (JSON/widget transport can't hold NaN)."""
    return [[None if not np.isfinite(v) else float(v) for v in row] for row in np.asarray(arr)]


def get_contour(X, Y, data):
    """Zero-level contours of `data` (shape [Y, X]) mapped to (rpm, torque) coords."""
    import skimage.measure
    out = []
    for c in skimage.measure.find_contours(np.asarray(data, dtype=float), 0):
        x = np.interp(c[:, 1], np.arange(len(X)), X)
        y = np.interp(c[:, 0], np.arange(len(Y)), Y)
        out.append((x, y))
    return out


def geometry_traces(motor):
    """Motor cross-section as {color: (xs, ys)} polylines. Segments of one color are
    merged into a single trace (None-separated) to keep the trace count tiny."""
    by_color = {}
    for grp in motor.geometry.plot_geometry():
        segs = grp["segments"]
        colors = grp.get("colors", None)
        for i, seg in enumerate(segs):
            color = colors[i] if colors else "gray"
            xs, ys = by_color.setdefault(color, ([], []))
            seg = np.asarray(seg, dtype=float)
            xs.extend(seg[:, 0].tolist()); xs.append(None)
            ys.extend(seg[:, 1].tolist()); ys.append(None)
    return by_color


def compute_plot(system, plot_key, max_rpm, max_torque, resolution, neg_axes, overlays):
    """Compute the heatmap grid + overlay contour lines. Returns plain data (no Figure),
    so the caller can push it into a persistent FigureWidget without remounting the plot."""
    n = int(resolution)
    rpm_range = np.linspace(-max_rpm * ("rpm" in neg_axes), max_rpm, n, endpoint=True)
    torque_range = np.linspace(-max_torque * ("torque" in neg_axes), max_torque, n, endpoint=True)

    g = system_limits(system, torque_range, rpm_range)  # arrays shape [torque, rpm]
    copper, iron = g["copper_loss"], g["iron_loss"]
    dissipation = copper + iron
    bus_power = g["bus_power"]
    efficiency = 1 - dissipation / np.maximum(np.abs(g["mechanical_power"]), np.abs(bus_power))
    acceleration = system.acceleration(rpm_range, g["mechanical_torque"])

    def amax(a):
        m = np.abs(np.nan_to_num(a)).max()
        return m if m > 0 else 1.0

    specs = {
        "Efficiency":   (efficiency,   0, 1, "Rainbow"),
        "Bus power":    (bus_power,    -amax(bus_power), amax(bus_power), "RdBu_r"),
        "Dissipation":  (dissipation,  0, amax(dissipation), "Electric"),
        "Acceleration": (acceleration, -amax(acceleration), amax(acceleration), "RdBu"),
    }
    data, zmin, zmax, cscale = specs[plot_key]

    # each contour -> a colored line + a dashed black backing line (matches the Dash app)
    contours = []

    def emit(conts, color, name):
        for x, y in conts:
            xl, yl = [float(v) for v in x], [float(v) for v in y]
            contours.append((xl, yl, color, name, False))
            contours.append((xl, yl, "black", name, True))

    gc = lambda v: get_contour(rpm_range, torque_range, v)
    if "Thermal" in overlays:
        for s in THERMAL_SPECS:
            emit(gc(system.temperatures(rpm_range, copper, iron, dt=s["dt"], key=s["key"]) - s["dT"]),
                 s["color"], "Thermal")
    if "Field weakening" in overlays:
        emit(gc(g["v_ratio_2"]), "white", "Field weakening")
    if "Efficiency" in overlays:
        emit(gc(efficiency - 0.9), "green", "90% efficiency")
    if "Acceleration" in overlays:
        fmt, lines = system.acceleration_lines()
        for ll in lines:
            emit(gc(acceleration - ll), "black", fmt.format(float(ll)))
    if "Open loop" in overlays:
        emit(gc(g["Vq_bal"]), "purple", "Open loop")
    if "Saturation" in overlays:
        emit(gc(np.abs(g["Iq"]) - system.actuator.motor.electrical.saturation), "gray", "Saturation")

    return dict(
        z=_json_safe(data), x=[float(v) for v in rpm_range], y=[float(v) for v in torque_range],
        zmin=float(zmin), zmax=float(zmax), colorscale=cscale, title=plot_key, contours=contours,
    )


# ---------------------------------------------------------------- UI
def _slider(id, label, lo, hi, val, step):
    return ui.input_slider(id, label, lo, hi, val, step=step)


app_ui = ui.page_sidebar(
    ui.sidebar(
        ui.navset_tab(
            ui.nav_panel(
                "Motor",
                ui.input_select("motor_preset", "Preset", list(MOTOR_PRESETS), selected=DEFAULT_MOTOR),
                ui.output_text_verbatim("motor_props"),
                _slider("turns", "Turns", 1, 12, _m0.geometry.turns, 1),
                _slider("radius", "Radius scale", 0.1, 2.0, 1.0, 0.1),
                _slider("slot_depth", "Slot-depth scale", 0.3, 1.5, 1.0, 0.1),
                _slider("axial", "Axial-length scale", 0.5, 1.5, 1.0, 0.1),
                _slider("slot_width", "Slot-width scale", 0.5, 1.0, 1.0, 0.1),
                _slider("reluctance", "Reluctance scale", 0.5, 1.5, 1.0, 0.1),
                _slider("frequency", "Frequency scale", 0.5, 1.5, 1.0, 0.1),
                ui.input_select("termination", "Termination", ["star", "delta"],
                                selected=_m0.electrical.geometry.termination),
                _slider("coil_temp", "Coil temperature (°C)", -20, 150, _m0.coil_temperature, 10),
                _slider("magnet_temp", "Magnet temperature (°C)", -20, 150, _m0.magnet_temperature, 10),
            ),
            ui.nav_panel(
                "Battery",
                ui.output_text_verbatim("battery_props"),
                _slider("charge", "Charge state", 0.0, 1.0, round(_base.battery.charge_state, 2), 0.05),
                _slider("batt_p", "Cells parallel (P)", 1, 20, int(_base.battery.P), 1),
                _slider("batt_s", "Cells series (S)", 1, 20, int(_base.battery.S), 1),
            ),
            ui.nav_panel(
                "Controller",
                ui.input_select("controller_preset", "Preset", list(CONTROLLER_PRESETS), selected=DEFAULT_CONTROLLER),
                ui.input_checkbox("field_weakening", "Field weakening", bool(_c0.field_weakening)),
                _slider("n_series", "Number of controllers", 1, 4, _base.actuator.n_series, 1),
                ui.input_numeric("bus_voltage", "Bus voltage limit", _c0.bus_voltage_limit, min=0, max=200, step=1),
                ui.input_numeric("phase_current", "Phase current limit", _c0.phase_current_limit, min=0, max=400, step=10),
                ui.input_numeric("power_limit", "Power limit", _c0.power_limit, min=0, max=20000, step=500),
                ui.input_numeric("freq_limit", "Frequency limit", _c0.freq_limit, min=100, max=10000, step=100),
                ui.input_numeric("modulation", "Modulation", round(_c0.modulation_factor, 3), min=0.5, max=1, step=0.01),
            ),
            ui.nav_panel(
                "Thermal",
                _slider("statorade", "Statorade", 0.0, 1.0, _m0.thermal.conductivity.statorade, 1),
                _slider("vented", "Vented", 0.0, 1.0, _m0.thermal.conductivity.vented, 1),
                _slider("potted", "Potted", 0.0, 1.0, _m0.thermal.conductivity.potted, 1),
                _slider("emissivity", "Emissivity", 0.0, 1.0, _m0.thermal.conductivity.emissivity, 1),
                _slider("rim_exp", "Rim exposure", 0.0, 1.0, _m0.thermal.conductivity.rim_exposure, 1),
                _slider("side_exp", "Flow exposure", 0.0, 1.0, _m0.thermal.conductivity.side_exposure, 0.1),
            ),
            ui.nav_panel(
                "Load",
                _slider("inertia", "Inertia (kg·m²)", 0.0, 10.0, 1.0, 0.5),
                _slider("drag", "Drag (Nm/(rad/s))", 0.0, 0.01, 0.002, 0.001),
            ),
            ui.nav_panel(
                "Visual",
                ui.input_select("plot_key", "Plot", PLOT_TYPES, selected="Bus power"),
                ui.input_checkbox_group("overlays", "Overlays", OVERLAYS, selected=DEFAULT_OVERLAYS),
                ui.input_checkbox_group("neg_axes", "Negative axis", ["rpm", "torque"], selected=["torque"]),
                ui.input_checkbox("rescale", "Auto-rescale axes to system", False),
                ui.input_numeric("max_rpm", "Max rpm", round(_INIT_RPM, 1), min=1),
                ui.input_numeric("max_torque", "Max torque (Nm)", round(_INIT_TORQUE, 2), min=0.1),
                _slider("resolution", "Grid resolution", 30, 140, 70, 10),
            ),
        ),
        width=380,
    ),
    ui.tags.style(
        # fill the inner Plotly divs to their widget's height (set via output_widget height=),
        # but NOT the widget container itself, so its explicit vh height is respected inside tabs.
        ".shiny-ipywidget-output .plotly-graph-div,"
        ".shiny-ipywidget-output .js-plotly-plot { height: 100% !important; }"
        ".tab-content, .tab-pane { height: 90vh; }"
    ),
    ui.navset_tab(
        ui.nav_panel("Performance", output_widget("plot", height="88vh", fill=True)),
        ui.nav_panel("Geometry", output_widget("geometry", height="88vh", fill=True)),
    ),
    title="pypowertrain (browser)",
    fillable=True,
)


# ---------------------------------------------------------------- server
def server(input, output, session):

    @reactive.calc
    def system():
        # A blank numeric field (e.g. backspaced empty) reads as None. Halt cleanly
        # instead of crashing; the last good plot is held until a valid value returns.
        req(*(getattr(input, name)() is not None
              for name in ("bus_voltage", "phase_current", "power_limit", "freq_limit", "modulation")))
        return build_system(input)

    # When a preset is picked, sync its intrinsic values into the controls
    # (mirrors the Dash "reset scalings to unity + adopt preset params" behavior).
    @reactive.effect
    @reactive.event(input.motor_preset)
    def _sync_motor():
        m = MOTOR_PRESETS[input.motor_preset()]()
        ui.update_slider("turns", value=m.geometry.turns)
        for sid in ("radius", "slot_depth", "axial", "slot_width", "reluctance", "frequency"):
            ui.update_slider(sid, value=1.0)
        ui.update_select("termination", selected=m.electrical.geometry.termination)
        ui.update_slider("coil_temp", value=m.coil_temperature)
        ui.update_slider("magnet_temp", value=m.magnet_temperature)

    @reactive.effect
    @reactive.event(input.controller_preset)
    def _sync_controller():
        c = CONTROLLER_PRESETS[input.controller_preset()]()
        ui.update_checkbox("field_weakening", value=bool(c.field_weakening))
        ui.update_numeric("bus_voltage", value=c.bus_voltage_limit)
        ui.update_numeric("phase_current", value=c.phase_current_limit)
        ui.update_numeric("power_limit", value=c.power_limit)
        ui.update_numeric("freq_limit", value=c.freq_limit)
        ui.update_numeric("modulation", value=round(c.modulation_factor, 3))

    # Auto-rescale is opt-in: only when checked do we recompute bounds from the
    # current system and push them into the (otherwise stable) manual inputs.
    @reactive.effect
    def _rescale():
        if not input.rescale():
            return
        max_rpm, max_torque = system_detect_limits(system())
        ui.update_numeric("max_rpm", value=round(float(max_rpm), 1))
        ui.update_numeric("max_torque", value=round(float(max_torque), 2))

    @render.text
    def motor_props():
        m = system().actuator.motor
        return (f"Mass:  {m.mass.total:0.3f} kg\n"
                f"Kt:    {m.Kt_ll:0.3f} Nm/A\n"
                f"R:     {m.R_ll:0.4f} ohm\n"
                f"L:     {m.L_ll*1000:0.3f} mH")

    @render.text
    def battery_props():
        b = system().battery
        return (f"Weight:   {b.weight:0.2f} kg\n"
                f"Voltage:  {b.voltage:0.1f} V\n"
                f"Capacity: {b.capacity:0.0f} Wh")

    # One persistent FigureWidget, mutated in place. Because the widget's DOM node is
    # never torn down and remounted, the plot keeps its full height across updates
    # (no flicker back to the Plotly default size on every slider move).
    fig = go.FigureWidget()
    fig.add_heatmap(z=[[0.0]], colorbar=dict(title=""))
    fig.update_layout(
        xaxis_title="rpm", yaxis_title="torque (Nm)",
        margin=dict(l=60, r=20, t=40, b=50),
        autosize=True, template="plotly_white",
    )
    # zero-axis reference lines (rpm=0 and torque=0). As layout shapes they persist
    # through the per-update data rebuild and auto-track the axes when they rescale.
    fig.add_hline(y=0, line=dict(color="rgba(40,40,40,0.7)", width=1.5))
    fig.add_vline(x=0, line=dict(color="rgba(40,40,40,0.7)", width=1.5))

    @render_widget
    def plot():
        return fig

    @reactive.effect
    def _update_plot():
        # guard against blank axis-bound fields (None) before touching the widget
        req(input.max_rpm() is not None, input.max_torque() is not None)
        req(input.max_rpm() > 0, input.max_torque() > 0)
        r = compute_plot(
            system(), input.plot_key(),
            input.max_rpm(), input.max_torque(),
            input.resolution(), input.neg_axes(), input.overlays(),
        )
        hm = fig.data[0]
        with fig.batch_update():
            hm.z, hm.x, hm.y = r["z"], r["x"], r["y"]
            hm.zmin, hm.zmax, hm.colorscale = r["zmin"], r["zmax"], r["colorscale"]
            hm.colorbar.title.text = r["title"]
            fig.layout.title.text = r["title"]
        # rebuild overlay traces: keep only the heatmap, then add the current contours
        if len(fig.data) > 1:
            fig.data = fig.data[:1]
        for x, y, color, name, dash in r["contours"]:
            fig.add_scatter(
                x=x, y=y, mode="lines", name=name, showlegend=False,
                line=dict(color=color, dash="dash" if dash else "solid"),
                hoverinfo="skip" if dash else "name",
            )

    # Second persistent widget: motor cross-section geometry. Depends only on the motor,
    # but recomputing from system() on any change is cheap (pure numpy).
    geo_fig = go.FigureWidget()
    geo_fig.update_layout(
        title="Motor cross-section", margin=dict(l=20, r=20, t=40, b=20),
        autosize=True, template="plotly_white", showlegend=False,
    )
    geo_fig.update_xaxes(visible=False)
    geo_fig.update_yaxes(visible=False, scaleanchor="x", scaleratio=1)  # equal aspect

    @render_widget
    def geometry():
        return geo_fig

    @reactive.effect
    def _update_geometry():
        traces = geometry_traces(system().actuator.motor)
        geo_fig.data = ()  # clear and redraw
        for color, (xs, ys) in traces.items():
            geo_fig.add_scatter(x=xs, y=ys, mode="lines", line=dict(color=color, width=1.5),
                                showlegend=False, hoverinfo="skip")


app = App(app_ui, server)