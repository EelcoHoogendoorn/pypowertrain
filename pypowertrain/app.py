"""Build a dash app to interact with a System object"""
import numpy as np

from pypowertrain.utils import *
from pypowertrain.system import *


import plotly.graph_objects as go
from dash import Dash, html, dcc, callback, Output, Input, State, ctx, dash_table
import dash
import dash_bootstrap_components as dbc
import plotly.express as px

import jsonpickle
import jsonpickle.ext.numpy as jsonpickle_numpy

jsonpickle_numpy.register_handlers(ndarray_mode='ignore')


def get_contour(X, Y, data):
	import skimage.measure
	contours = skimage.measure.find_contours(data, 0)

	c = []
	for contour in contours:
		x = np.interp(contour[:, 1], np.arange(len(X)), X)
		y = np.interp(contour[:, 0], np.arange(len(Y)), Y)
		c.append((x, y))
	return c


def calc_stuff(system, x_range, y_range, thermal_specs):
	"""do heavy calcs on system"""
	# eval the system performance graphs
	rpm_range = system.x_axis_inverse(x_range)
	torque_range = system.y_axis_inverse(y_range)
	graphs = system_limits(system, torque_range, rpm_range)

	copper_loss = graphs['copper_loss']
	iron_loss = graphs['iron_loss']
	bus_power = graphs['bus_power']
	mechanical_power = graphs['mechanical_power']
	Iq = graphs['Iq']
	vbal = graphs['Vq_bal']
	torque = graphs['mechanical_torque']

	dissipation = copper_loss + iron_loss
	efficiency = 1 - dissipation / np.maximum(np.abs(mechanical_power), np.abs(bus_power))
	acceleration = system.acceleration(x_range, torque)

	get_cont = lambda v: get_contour(x_range, y_range, v)

	# construct thermal curves
	thermal_contours = {
		s['color']: get_cont(system.temperatures(
			rpm_range, copper_loss, iron_loss, dt=s['dt'], key=s['key']) - s['dT'])
		for s in thermal_specs
	}

	name, lines = system.acceleration_lines()
	acceleration_contours = {name.format(float(ll)): get_cont(acceleration - ll) for ll in lines}

	# q = np.where(Id == 0, 1, np.nan_to_num(Id, nan=-1))
	# import skimage.filters
	# fw_lim = skimage.filters.gaussian(q, [3, 1]) + Id * 0
	# fw_lim_contour = get_cont(fw_lim + 3)
	fw_lim_contour = get_cont(graphs['v_ratio_2'])

	efficiency_contour = get_cont(efficiency - 0.9)

	vbal_contour = get_cont(vbal)

	p = np.abs(Iq) - system.actuator.motor.electrical.saturation
	saturation_contour = get_cont(p)

	contours = thermal_contours, acceleration_contours, fw_lim_contour, efficiency_contour, vbal_contour, saturation_contour
	graphs = copper_loss, iron_loss, bus_power, mechanical_power, torque

	return graphs, contours


def plot_contour(fig, contours, color="rgba(0,0,0,1)", name=''):
	for contour in contours:
		x, y = contour
		fig.add_trace(
			go.Scatter(
				x=x, y=y, mode='lines',
				line_color=color,
				showlegend=False, name=name,
			))
		fig.add_trace(
			go.Scatter(
				x=x, y=y, mode='lines', line_dash='dash',
				line_color='black',
				showlegend=False, name=name,
			))


def system_dash(
	system: System,
	n_x=50,
	n_y=51,
	targets=None,
):
	"""dash plotly app

	add range sliders, to set bounds for optimization
	then add a button to do some grad descent within the bounds given by the slider

	https://dash.plotly.com/sharing-data-between-callbacks

	FIXME: we cannot remove callbacks dynamically. this means we cannot have a thermal config tab,
	 and allow switching motors with different thermal models at the same time.
	 or would pattern matching callbacks make it possible? dunno
	 perhaps just add tabs for all thermal model types in motor list; and hide/show as appropriate?

	FIXME: can we do A/B comparison app? i thnk it requires seperate app of sorts
	  should prob modularize things; two tabs with two independent systems
	  or partially independent systems; might want to keep some parts in sync, like load params?

	"""

	plots = [
		'Dissipation',
		'Bus power',
		'Efficiency',
		'Acceleration'
	]

	turns_tooltip = "This changes the number of turns, at equal total copper volume in the coils. As such it does not impact the motor mass. There is no right or wrong here; but it is important to tune it properly to the intended application and controller constraints. This scaling law is exact; although it ignores enamel thickness, eddy-losses, and winding-practicalities."
	radius_tooltip = 'Radial scaling has a quadratic effect on both motor mass and torque. Dialing in this value to one appropriate for your design constraints is often one of the more important ones. Note that radial scaling is exact in terms of the electrical parameters. The mass predictions of the frameless parts of the motor are exact, but the main uncertainty here is in the mass of the structural parts; depending on the scale different construction technicues might be appropriate'
	slot_depth_tooltip = 'The slot depth has a direct influence on the amount of copper available, and tends to scale the thermal curves along the y-axis. It also has a profound effect on motor weight, since most of the mass in in the teeth and their copper. Very few caveats apply to this scaling law and extrapolations should be valid over a wide range of scalings'
	axial_tooltip = 'Scaling of the axial length of the motor. Too low values will results in excessive negative end-effects, such as coil overhang. Longer values tend to have more restrictive thermal performance, per kg of motor'
	reluctance_tooltip = 'Scaling of the amount of mu-0 in the motor magnetic circuit, at equal flux-density, and equal iron saturation. This is a fairly safe rescaling, although it ignores flux leakage and fringing effects. It will impact inductance, saturation and demagnetization limits.'
	slot_width_tooltip = 'Scaling of the slot width. This trades copper current area for iron flux area. Narrower slots will raise resistance, and lower iron losses. Increasing slot width is generally not recommended, since motors tend to be designed near their iron saturation limits. A slight decrease might make sense though, if the resistance-headroom exists. Wider teeth will benefit iron saturation limits too.'
	frequency_tooltip = "Scaling of the number of poles and slots, at equal gap radius. Note that these fractional scalings are generally not realizable; but adjusting this value should give quite an accurate idea, if your application would benefit from a motor with a different pole count."
	# flux_tooltip = 'Flux scaling, at equal field density. This increases magnet volume and tooth width, at equal iron field level'

	thermal_specs = [
		{'color': 'yellow', 'dt': 5000, 'dT': 60, 'key': 'coils'},
		{'color': 'orange', 'dt': 60, 'dT': 60, 'key': 'coils'},
		{'color': 'red', 'dt': 2, 'dT': 40, 'key': 'coils'},
	]

	from pypowertrain.library import odrive, grin, moteus
	controllers = {
		'odrive.pro': odrive.pro(),
		'odrive.pro_nominal': odrive.pro_nominal(),
		'odrive.pro_overclock': odrive.pro_overclock(),
		'grin.phaserunner': grin.phaserunner(),
		'moteus.n1': moteus.n1(),
	}
	motors = {
		'odrive.M8325s_100KV': odrive.M8325s_100KV(),
		'odrive.botwheel': odrive.botwheel(),
		'grin.all_axle': grin.all_axle(),
		'moteus.mj5208': moteus.mj5208(),
	}
	overlays = [
		'Thermal',
		'Field weakening',
		'Efficiency',
		'Acceleration',
		'Open loop',
		'Saturation'
	]
	termination_options = [
		'star',
		'delta'
	]
	plot_types = ['Geometry', 'Graph']

	app = dash.Dash(external_stylesheets=[dbc.themes.BOOTSTRAP])

	thermal_table = dash_table.DataTable(
		id='table-thermal',
		columns=([
			{'id': p, 'name': p, 'type': 'text' if isinstance(v, str) else 'numeric'}
			for p, v in thermal_specs[0].items()
		]),
		data=thermal_specs,
		editable=True
	)
	x_key, _ = system.x_axis(0)
	y_key, _ = system.y_axis(0)
	limits_table = dash_table.DataTable(
		id='table-limits',
		columns=([
			{'id': 'x', 'name': x_key, 'type': 'numeric'},
			{'id': 'y', 'name': y_key, 'type': 'numeric'},
		]),
		data=[{'x': 100, 'y': 100}],
		editable=False
	)
	# stats_table = dash_table.DataTable()

	load_tab = system.load.dash_tab()

	motor_tab = dbc.Tab(label='Motor', children=[
		dcc.Dropdown(list(motors.keys()), '', id='dropdown-motor'),
		html.Div(id='motor-props-label', style={'whiteSpace': 'pre-wrap'}),
		html.Label('Turns'),
		dcc.Slider(1, 12, 1, value=system.actuator.motor.geometry.turns, id='turns-slider'),
		dbc.Tooltip(turns_tooltip, target='turns-slider'),
		html.Label('Radius-scale'),
		dcc.Slider(0.1, 2, 0.1, value=1, id='radius-slider'),
		dbc.Tooltip(radius_tooltip, target='radius-slider'),
		html.Label('Slot-depth-scale'),
		dcc.Slider(0.3, 1.5, 0.1, value=1, id='slot-depth-slider'),
		dbc.Tooltip(slot_depth_tooltip, target='slot-depth-slider'),
		html.Label('Axial-length-scale'),
		dcc.Slider(0.5, 1.5, 0.1, value=1, id='axial-slider'),
		dbc.Tooltip(axial_tooltip, target='axial-slider'),
		html.Label('Slot width scale'),
		dcc.Slider(0.5, 1.0, 0.1, value=1, id='slot-width-slider'),
		dbc.Tooltip(slot_width_tooltip, target='slot-width-slider'),
		html.Label('Reluctance scale'),
		dcc.Slider(0.5, 1.5, 0.1, value=1, id='reluctance-slider'),
		dbc.Tooltip(reluctance_tooltip, target='reluctance-slider'),
		html.Label('Frequency scale'),
		dcc.Slider(0.5, 1.5, 0.1, value=1, id='frequency-slider'),
		dbc.Tooltip(frequency_tooltip, target='frequency-slider'),
		html.Label('Termination'),
		dcc.Dropdown(termination_options, system.actuator.motor.electrical.geometry.termination, id='termination-selection'),
		html.Label('Coil temperature'),
		dcc.Slider(-20, 150, 10, value=system.actuator.motor.coil_temperature, id='coil-temperature-slider'),
		html.Label('Magnet temperature'),
		dcc.Slider(-20, 150, 10, value=system.actuator.motor.magnet_temperature, id='magnet-temperature-slider'),

	])
	battery_tab = dbc.Tab(label='Battery', children=[
		html.Label('Battery charge'),
		dcc.Slider(0.0, 1.0, 0.1, value=system.battery.charge_state, id='charge-slider'),
		html.Label('Battery P'),
		dcc.Slider(1, 20, 1, value=system.battery.P, id='P-slider'),
		html.Label('Battery S'),
		dcc.Slider(1, 20, 1, value=system.battery.S, id='S-slider'),
		# html.Div(id='battery-weight-label'),
		html.Div(id='battery-props-label', style={'whiteSpace': 'pre-wrap'}),
	])
	# FIXME: this is specific to the shelled thermal model
	#  prob best to make app code more modular; so there can be broad motor-controller comparison functionality,
	#  as well as a different app for tuning a specific motor?
	thermal_tab = dbc.Tab(label='Thermal', children=[
		html.Label('Statorade'),
		dcc.Slider(0.0, 1.0, 1, value=system.actuator.motor.thermal.conductivity.statorade, id='statorade-slider'),
		html.Label('Vented'),
		dcc.Slider(0.0, 1.0, 1, value=system.actuator.motor.thermal.conductivity.vented, id='vented-slider'),
		html.Label('Potted'),
		dcc.Slider(0.0, 1.0, 1, value=system.actuator.motor.thermal.conductivity.potted, id='potted-slider'),
		html.Label('Emissivity'),
		dcc.Slider(0.0, 1.0, 1, value=system.actuator.motor.thermal.conductivity.emissivity, id='emissivity-slider'),
		html.Label('Rim exposure'),
		dcc.Slider(0.0, 1.0, 1, value=system.actuator.motor.thermal.conductivity.rim_exposure, id='rim-exp-slider'),
		html.Label('Flow exposure'),
		dcc.Slider(0.0, 1.0, .1, value=system.actuator.motor.thermal.conductivity.side_exposure, id='side-exp-slider'),
	])
	fw_options = ['Field weakening']
	controller_tab = dbc.Tab(label='Controller', children=[
		html.Label('Controller'),
		dcc.Dropdown(list(controllers.keys()), '', id='dropdown-controller'),
		dcc.Checklist(options=fw_options, value=fw_options, id='field-weakening'),
		html.Label('Number of controllers'),
		dcc.Slider(1, 4, 1, value=system.actuator.n_series, id='n_series-slider'),
		dbc.Label('Bus voltage limit'),
		dbc.Input(value=system.actuator.controller.bus_voltage_limit, type='number', id='c-voltage-input', min=0, max=200, step=1),
		dbc.Label('Phase current limit'),
		dbc.Input(value=system.actuator.controller.phase_current_limit, type='number', id='c-amp-input', min=0, max=400, step=10),
		dbc.Label('Power limit'),
		dbc.Input(value=system.actuator.controller.power_limit, type='number', id='c-power-input', min=0, max=20000, step=500),
		dbc.Label('Frequency limit'),
		dbc.Input(value=system.actuator.controller.freq_limit, type='number', id='c-freq-input', min=100, max=10000, step=100),
		dbc.Label('Modulation'),
		dbc.Input(value=system.actuator.controller.modulation_factor, type='number', id='c-modulation-input', min=0.5, max=1, step=0.01),
	])
	visual_tab = dbc.Tab(label='Visual', children=[
		html.Label('Plot types'),
		dcc.Checklist(options=plot_types, value=plot_types, id='plot-type-list'),
		html.Label('Plot image'),
		dcc.Dropdown(plots, 'Acceleration', id='dropdown-selection'),
		html.Label('Plot overlays'),
		dcc.Checklist(options=overlays, value=overlays, id='check-overlay', labelStyle={'display': 'block'},),
		html.Label('Automatic plot bound rescaling'),
		dcc.Checklist(options=['Rescale'], value=['Rescale'], id='rescale'),
		limits_table,
		html.Label('Simulation resolution'),
		dcc.Slider(1, 4, 1, value=1, id='resolution-slider'),
		html.Label('Thermal curve specification'),
		thermal_table,
		html.Div(id='debug'),
		html.Label('Negative axis'),
		dcc.Checklist(options=['x-axis', 'y-axis'], value=['y-axis'], id='axes-check'),
	])

	graph_tab = dbc.Tab(label='Graph', children=[
		dcc.Graph(
			id='graph-content',
			style={"width": "100%", "height": "90vh"},
			responsive=True,
		),
	])
	geometry_tab = dbc.Tab(label='Geometry', children=[
		dcc.Graph(
			id='geometry-content',
			style={"width": "100%", "height": "100vh"},
			responsive=True,
		),
	])

	# main layout structure
	app.layout = html.Div([
		dbc.Container(
			dbc.Row([
				dbc.Col(
					dbc.Tabs([
						graph_tab,
						geometry_tab,
					]),
					width=8,
				),
				dbc.Col(
					dbc.Tabs([
						motor_tab,
						thermal_tab,
						battery_tab,
						controller_tab,
						load_tab,
						visual_tab,
					]),
					width=4,
				),
			]),
		),

		dcc.Store(id='load'),
		dcc.Store(id='thermal'),
		dcc.Store(id='controller_og'),
		dcc.Store(id='controller'),
		dcc.Store(id='motor_og'),
		dcc.Store(id='motor'),
		dcc.Store(id='battery_og'),
		dcc.Store(id='battery'),
		dcc.Store(id='system'),
		dcc.Store(id='ranges'),
		dcc.Store(id='graphdata'),
	])

	# add the load callback
	system.load.dash_callback()

	@callback(
		Output('thermal', 'data'),

		Input('statorade-slider', 'value'),
		Input('vented-slider', 'value'),
		Input('potted-slider', 'value'),
		Input('emissivity-slider', 'value'),
		Input('rim-exp-slider', 'value'),
		Input('side-exp-slider', 'value'),

	)
	def compute_handler_thermal(
			statorade, vented, potted, emissivity, rim_exp, side_exp,
	):
		thermal = system.actuator.motor.thermal
		thermal = thermal.replace(
			conductivity__statorade=statorade,
			conductivity__vented=vented,
			conductivity__potted=potted,
			conductivity__emissivity=emissivity,
			conductivity__rim_exposure=rim_exp,
			conductivity__side_exposure=side_exp,
		)
		return pickle_encode(thermal)

	@callback(
		Output('controller_og', 'data'),
		Output('field-weakening', 'value'),
		Output('c-voltage-input', 'value'),
		Output('c-amp-input', 'value'),
		Output('c-power-input', 'value'),
		Output('c-modulation-input', 'value'),
		Output('c-freq-input', 'value'),

		Input('dropdown-controller', 'value'),
	)
	def compute_handler_controller(
		controller,
	):
		controller = controllers.get(controller, system.actuator.controller)
		ui = (
			fw_options * controller.field_weakening,
			controller.bus_voltage_limit,
			controller.phase_current_limit,
			controller.power_limit,
			controller.modulation_factor,
			controller.freq_limit,)
		return (pickle_encode(controller),) + ui

	@callback(
		Output('controller', 'data'),

		Input('controller_og', 'data'),
		Input('field-weakening', 'value'),
		Input('c-voltage-input', 'value'),
		Input('c-amp-input', 'value'),
		Input('c-power-input', 'value'),
		Input('c-modulation-input', 'value'),
		Input('c-freq-input', 'value'),
	)
	def compute_handler_controller_adjust(
			controller, field_weakening, voltage, amps, power, modulation, freq
	):
		controller = pickle_decode(controller)
		controller = controller.replace(
			field_weakening=bool(field_weakening),
			bus_voltage_limit=voltage,
			phase_current_limit=amps,
			power_limit=power,
			modulation_factor=modulation,
			freq_limit=freq,
		)
		return pickle_encode(controller)


	@callback(
		Output('motor_og', 'data'),
		Output('turns-slider', 'value'),
		Output('radius-slider', 'value'),
		Output('slot-depth-slider', 'value'),
		Output('axial-slider', 'value'),
		Output('slot-width-slider', 'value'),
		Output('reluctance-slider', 'value'),
		Output('frequency-slider', 'value'),
		Output('termination-selection', 'value'),
		Output('coil-temperature-slider', 'value'),
		Output('magnet-temperature-slider', 'value'),

		Input('dropdown-motor', 'value'),
	)
	def compute_handler_motor(
		motor,
	):
		motor = motors.get(motor, system.actuator.motor)
		ui = (
			motor.electrical.geometry.turns,
			1.0, 1.0, 1.0, 1.0, 1.0, 1.0,	# reset scalings to unity
			motor.electrical.geometry.termination,
			motor.coil_temperature,
			motor.magnet_temperature,
		)
		return (pickle_encode(motor),) + ui

	@callback(
		Output('motor', 'data'),

		Input('motor_og', 'data'),
		Input('turns-slider', 'value'),
		Input('radius-slider', 'value'),
		Input('slot-depth-slider', 'value'),
		Input('axial-slider', 'value'),
		Input('slot-width-slider', 'value'),
		Input('reluctance-slider', 'value'),
		Input('frequency-slider', 'value'),
		Input('termination-selection', 'value'),
		Input('coil-temperature-slider', 'value'),
		Input('magnet-temperature-slider', 'value'),
	)
	def compute_handler_motor_adjust(
			motor,
			turns,
			radius,
			slot_depth,
			axial,
			slot_width,
			reluctance,
			frequency,
			termination, coil_temperature, magnet_temperature,
	):
		motor = pickle_decode(motor)
		motor = motor.replace(
			__geometry__turns=turns,
			__geometry__radius_scale=radius,
			__geometry__slot_depth_scale=slot_depth,
			__geometry__length_scale=axial,
			__geometry__slot_width_scale=slot_width,
			__geometry__reluctance_scale=reluctance,
			__geometry__frequency_scale=frequency,
			__geometry__termination=termination,
			coil_temperature=coil_temperature,
			magnet_temperature=magnet_temperature,
		)
		return pickle_encode(motor)


	@callback(
		Output('battery', 'data'),

		# Input('battery_og', 'data'),
		Input('charge-slider', 'value'),
		Input('P-slider', 'value'),
		Input('S-slider', 'value'),
	)
	def compute_handler_battery_adjust(
			charge, P, S,
	):
		# battery = pickle_decode(battery)

		battery = system.battery
		battery = battery.replace(
			charge_state=charge,
			P=P,
			S=S,
		)
		return pickle_encode(battery)

	@callback(
		Output('system', 'data'),

		Input('controller', 'data'),
		Input('n_series-slider', 'value'),
		Input('motor', 'data'),
		Input('battery', 'data'),

		Input('load', 'data'),
		Input('thermal', 'data'),

	)
	def compute_handler_system(
			controller, n_series,
			motor,
			battery,
			load,
			thermal,
	):
		motor = pickle_decode(motor)
		controller = pickle_decode(controller)
		battery = pickle_decode(battery)
		load = pickle_decode(load)
		thermal = pickle_decode(thermal)
		sysr = system.replace(
			actuator__n_series=n_series,
			__controller=controller,
			__motor=motor,
			__thermal=thermal,
			load=load,
			battery=battery
		)
		return pickle_encode(sysr)

	@callback(
		Output('motor-props-label', 'children'),
		Input('motor', 'data'),
	)
	def compute_handler_motor_props(motor):
		motor = pickle_decode(motor)
		return \
			f'Mass:\t {motor.mass.total:0.3f} \tkg\n' \
			f'Kt:\t\t {motor.Kt_ll:0.3f} \tNm/A\n' \
			f'R: \t\t {motor.R_ll:0.3f} \tohm\n' \
			f'L: \t\t {motor.L_ll*1000:0.3f} \tmH'


	@callback(
		Output('battery-props-label', 'children'),
		Input('system', 'data'),
	)
	def compute_handler_battery_props(system):
		system = pickle_decode(system)
		return \
			f'Weight:\t {system.battery.weight:0.3f} \t kg\n' \
			f'Voltage:\t {system.battery.voltage:0.1f} \t V\n' \
			f'Capacity:\t {system.battery.capacity:0.0f} \t wh\n'

	@callback(
		Output('table-limits', 'editable'),

		Input('rescale', 'value'),
	)
	def compute_handler_rescale(rescale):
		return not bool(rescale)

	@callback(
		Output('table-limits', 'data'),

		Input('system', 'data'),
		Input('rescale', 'value'),
	)
	def compute_handler_limits(system, rescale):
		"""update range estimates"""
		if rescale:
			system = pickle_decode(system)
			max_x, max_y = system_detect_limits(system, fw=1.3)
			return [{
				'x': system.x_axis_forward(max_x),
				'y': system.y_axis_forward(max_y)
			}]

		return dash.no_update

	@callback(
		Output('ranges', 'data'),

		Input('table-limits', 'data'),
		Input('resolution-slider', 'value'),
		Input('axes-check', 'value'),
	)
	def compute_handler_ranges(limits, resolution, axes):
		"""update range estimates"""
		max_x = limits[0]['x']
		max_y = limits[0]['y']

		x_range = np.linspace(-max_x * ('x-axis' in axes), +max_x, n_x * resolution, endpoint=True)
		y_range = np.linspace(-max_y * ('y-axis' in axes), +max_y, n_y * resolution, endpoint=True)
		return jsonpickle.dumps((x_range, y_range))

	@callback(
		Output('debug', 'children'),
		Input('table-thermal', 'data'),
	)
	def compute_handler(thermals):
		return str(thermals)

	@callback(
		Output('graphdata', 'data'),

		Input('system', 'data'),
		Input('ranges', 'data'),
		Input('table-thermal', 'data'),	# could be its own thing
	)
	def compute_handler(sysr, ranges, thermals):
		"""do motor calcs and write to data store"""
		sysr = pickle_decode(sysr)
		x_range, y_range = jsonpickle.loads(ranges)

		res = calc_stuff(sysr, x_range, y_range, thermals)
		return jsonpickle.dumps(res)


	@callback(
		Output('geometry-content', 'figure'),

		Input('system', 'data'),
		Input('plot-type-list', 'value'),
	)
	def update_geo_plot(system, plot_types):
		if not 'Geometry' in plot_types: return go.Figure()

		system = pickle_decode(system)

		fig = go.Figure()

		geo = system.actuator.motor.geometry.plot_geometry()
		for g in geo:
			data = g['segments']
			c = g.get('colors', None)
			for i in range(len(data)):
				fig.add_trace(
					go.Scatter(
						x=data[i][:, 0],
						y=data[i][:, 1],
						mode='lines',
						line_color=c[i] if c else 'gray',
						showlegend=False,
					)
				)
		return fig

	@callback(
		Output('graph-content', 'figure'),

		State('system', 'data'),
		Input('dropdown-selection', 'value'),
		Input('check-overlay', 'value'),
		Input('graphdata', 'data'),
		State('ranges', 'data'),
		Input('plot-type-list', 'value'),
	)
	def update_graph(system, plot_key, overlay_data, data, ranges, plot_types):
		if not 'Graph' in plot_types: return go.Figure()
		system = pickle_decode(system)

		x_range, y_range = jsonpickle.loads(ranges)
		graphs, contours = jsonpickle.loads(data)

		copper_loss, iron_loss, bus_power, mechanical_power, torque = graphs
		thermal_contours, acceleration_contours, fw_lim_contour, efficiency_contour, vbal_contour, saturation_contour = contours

		dissipation = copper_loss + iron_loss
		efficiency = 1 - dissipation / np.maximum(np.abs(mechanical_power), np.abs(bus_power))
		acceleration = system.acceleration(x_range, torque)

		x_label, _ = system.x_axis(0)
		y_label, _ = system.y_axis(0)

		import xarray as xr
		def wrap(arr, name=''):
			return xr.DataArray(
				data=arr,
				dims=[y_label, x_label],
				coords={x_label: x_range, y_label: y_range},
				name=name
			)


		if plot_key == 'Efficiency':
			# FIXME: add custom color scale with high contrast at the high end
			fig = px.imshow(wrap(efficiency, plot_key), origin='lower', zmin=0, zmax=1, color_continuous_scale='rainbow')
		if plot_key == 'Bus power':
			zlim = np.abs(np.nan_to_num(bus_power)).max()
			fig = px.imshow(wrap(bus_power, plot_key), origin='lower', zmin=-zlim, zmax=zlim, color_continuous_scale='RdBu_r')
		if plot_key == 'Dissipation':
			zlim = np.abs(np.nan_to_num(dissipation)).max()
			fig = px.imshow(wrap(dissipation, plot_key), origin='lower', zmin=0, zmax=zlim, color_continuous_scale='Electric')
		if plot_key == 'Acceleration':
			zlim = np.abs(np.nan_to_num(acceleration)).max()
			fig = px.imshow(wrap(acceleration, plot_key), origin='lower', zmin=-zlim, zmax=zlim, color_continuous_scale='RdBu')

		if 'Thermal' in overlay_data:
			for color, contour in thermal_contours.items():
				plot_contour(fig, contour, color, name='Thermal')

		if 'Field weakening' in overlay_data:
			plot_contour(fig, fw_lim_contour, color="white", name='Field weakening limit')

		if 'Efficiency' in overlay_data:
			plot_contour(fig, efficiency_contour, color='green', name='90% efficiency')

		if 'Acceleration' in overlay_data:
			# add acceleration based load lines
			for a, contour in acceleration_contours.items():
				# FIXME: debug string formatting
				plot_contour(fig, contour, color='black', name=a)

		if 'Open loop' in overlay_data:
			plot_contour(fig, vbal_contour, color='purple', name='Open loop')

		if 'Saturation' in overlay_data:
			plot_contour(fig, saturation_contour, color='gray', name='Saturation linear limit')

		# add x axis
		fig.add_trace(
			go.Scatter(x=x_range, y=x_range*0, name='x-axis', mode='lines', line_color='gray', showlegend=False,)
		)

		if targets is not None:
			t_torque, t_rpm, t_dissipation, t_weight = [np.array(t) for t in targets]
			x, X = system.x_axis(t_rpm)
			y, Y = system.y_axis(t_torque)
			trace = go.Scatter(
					x=X, y=Y,
					showlegend=False,
					mode='markers',
				)
			trace.update(marker=dict(size=12, line=dict(width=2, vcolor='DarkSlateGrey')))
			fig.add_trace(trace)

		fig.update_layout(margin=dict(b=10, t=10, l=10, r=10))

		return fig

	return app
