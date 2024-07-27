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


def calc_stuff(system, rpm_range, torque_range, thermal_specs):
	"""do heavy calcs on system"""
	# eval the system performance graphs

	graphs = system_limits(system, torque_range, rpm_range).astype(np.float32)
	copper_loss, iron_loss, bus_power, mechanical_power, Iq, Id, torque = graphs

	dissipation = copper_loss + iron_loss
	# efficiency = 1 - dissipation / np.abs(bus_power)
	efficiency = 1 - dissipation / np.maximum(np.abs(mechanical_power), np.abs(bus_power))
	acceleration = system.acceleration(rpm_range, torque)

	x, X = system.x_axis(rpm_range)
	y, Y = system.y_axis(torque_range)

	# construct thermal curves
	thermal_contours = {
		s['color']: get_contour(X, Y, system.temperatures(
			rpm_range, copper_loss, iron_loss, dt=s['dt']) - s['dT'])
		for s in thermal_specs
	}

	name, lines = system.acceleration_lines()
	acceleration_contours = {name.format(float(ll)): get_contour(X, Y, acceleration - ll) for ll in lines}

	q = np.where(Id == 0, 1, np.nan_to_num(Id, nan=-1))
	import skimage.filters
	fw_lim = skimage.filters.gaussian(q, [3, 1]) + Id * 0
	fw_lim_contour = get_contour(X, Y, fw_lim + 3)

	efficiency_contour = get_contour(X, Y, efficiency - 0.9)

	contours = thermal_contours, acceleration_contours, fw_lim_contour, efficiency_contour
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
	n_rpm=50,
	n_torque=51,
	max_rpm=None,
	max_torque=None,
	targets=None,
):
	"""dash plotly app

	add range sliders, to set bounds for optimization
	then add a button to do some grad descent within the bounds given by the slider

	https://dash.plotly.com/sharing-data-between-callbacks

	"""

	plots = ['dissipation', 'power', 'efficiency', 'Id', 'acceleration']

	turns_tooltip = "This changes the number of turns, at equal total copper volume in the coils. As such it does not impact the motor mass. There is no right or wrong here; but it is important to tune it properly to the intended application. This scaling law is exact; although it ignores enamel thickness, eddy-losses, and winding-practicalities."
	radius_tooltip = 'Radial scaling has a quadratic effect on both motor mass and torque. Dialing in this value to one appropriate for your design constraints is often one of the more important ones. Note that radial scaling is exact in terms of the electrical parameters. The mass predictions of the frameless parts of the motor are exact, but the main uncertainty here is in the mass of the structural parts; depending on the scale different construction technicues might be appropriate'
	slot_depth_tooltip = 'The slot depth has a direct influence on the amount of copper available, and tends to scale the thermal curves along the y-axis. It also has a profound effect on motor weight, since most of the mass in in the teeth and their copper. Very few caveats apply to this scaling law and extrapolations should be valid over a wide range of scalings'
	axial_tooltip = 'Scaling of the axial length of the motor. Too low values will results in excessive negative end-effects, such as coil overhang. Longer values tend to have more restrictive thermal performance, per kg of motor'
	reluctance_tooltip = 'Scaling of the amount of mu-0 in the motor magnetic circuit, at equal flux-density, and equal iron saturation. This is a fairly safe rescaling, although it ignores flux leakage and fringing effects. It will impact inductance and demagnetization limits.'
	slot_width_tooltip = 'Scaling of the slot width. This trades copper current area for iron flux area. Narrower slots will raise resistance, and lower iron losses. Increasing slot width is generally not recommended, since motors tend to be designed near their iron saturation limits. A slight decrease might make sense though, if the resistance-headroom exists. Iron saturation is not currently modelled but more iron should raise the iron saturation limit too.'
	frequency_tooltip = "Scaling of the number of poles and slots, at equal gap radius. Note that these fractional scalings are generally not realizable; but adjusting this value should give quite an accurate idea, if your application would benefit from a motor with a different pole count."
	# flux_tooltip = 'Flux scaling, at equal field density. This increases magnet volume and tooth width, at equal iron field level'

	thermal_specs = [
		{'color': 'yellow', 'dt': 5000, 'dT': 60},
		{'color': 'orange', 'dt': 60, 'dT': 60},
		{'color': 'red', 'dt': 2, 'dT': 40},
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
	])
	battery_tab = dbc.Tab(label='Battery', children=[
		html.Label('Battery charge'),
		dcc.Slider(0.0, 1.0, 0.1, value=system.battery.charge_state, id='charge-slider'),
		html.Label('Battery P'),
		dcc.Slider(1, 20, 1, value=system.battery.P, id='P-slider'),
		html.Label('Battery S'),
		dcc.Slider(1, 20, 1, value=system.battery.S, id='S-slider'),
		html.Div(id='battery-weight-label'),
	])
	controller_tab = dbc.Tab(label='Controller', children=[
		html.Label('Controller'),
		dcc.Dropdown(list(controllers.keys()), '', id='dropdown-controller'),
		dcc.Checklist(options=['Field weakening'], value=['Field weakening'], id='field-weakening'),
		html.Label('Number of controllers'),
		dcc.Slider(1, 5, 1, value=system.actuator.n_series, id='n_series-slider'),
	])
	visual_tab = dbc.Tab(label='Visual', children=[
		html.Label('Plot types'),
		dcc.Checklist(options=plot_types, value=plot_types, id='plot-type-list'),
		html.Label('Plot image'),
		dcc.Dropdown(plots, 'acceleration', id='dropdown-selection'),
		html.Label('Plot overlays'),
		dcc.Checklist(options=overlays, value=overlays, id='check-overlay', labelStyle={'display': 'block'},),
		html.Label('Automatic plot bound rescaling'),
		dcc.Checklist(options=['Rescale'], value=['Rescale'], id='rescale'),
		html.Label('Simulation resolution'),
		dcc.Slider(1, 4, 1, value=1, id='resolution-slider'),
		html.Label('Thermal curve specification'),
		thermal_table,
		html.Div(id='debug'),
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
						battery_tab,
						controller_tab,
						load_tab,
						visual_tab,
					]),
					width=4,
				),
			]),
		),

		# dcc.Store(id='coarsedata'),		# FIXME: split this more granular? sep arrays? coarse and fine?
		dcc.Store(id='load'),		# also split other subcomponents into substore.
		dcc.Store(id='system'),
		dcc.Store(id='ranges'),
		dcc.Store(id='graphdata'),
	])

	# add the load callback
	system.load.dash_callback()

	@callback(
		Output('system', 'data'),

		Input('dropdown-motor', 'value'),
		Input('turns-slider', 'value'),
		Input('radius-slider', 'value'),
		Input('slot-depth-slider', 'value'),
		Input('axial-slider', 'value'),
		Input('slot-width-slider', 'value'),
		Input('reluctance-slider', 'value'),
		Input('frequency-slider', 'value'),

		Input('charge-slider', 'value'),
		Input('P-slider', 'value'),
		Input('S-slider', 'value'),

		Input('dropdown-controller', 'value'),
		Input('field-weakening', 'value'),
		Input('n_series-slider', 'value'),

		Input('load', 'data'),
	)
	def compute_handler_system(
			motor, turns, radius, slot_depth, length, slot_width, reluctance, frequency,
			charge, P, S,
			controller, field_weakening, n_series,
			load,
	):
		"""Put all modifiers to the system object here"""
		# FIXME: place base object selection upstream?
		sysr = system
		if motor:
			sysr = sysr.replace(__motor=motors[motor])
		if controller:
			sysr = sysr.replace(__controller=controllers[controller])
		sysr = sysr.replace(
			# __motor=motors[motor],
			# __controller=controllers[controller],
			load=pickle_decode(load),
		).replace(
			__geometry__turns=turns,
			__geometry__radius_scale=radius,
			__geometry__slot_depth_scale=slot_depth,
			__geometry__length_scale=length,
			__geometry__slot_width_scale=slot_width,
			__geometry__reluctance_scale=reluctance,
			__geometry__frequency_scale=frequency,
			battery__charge_state=charge,
			battery__P=P,
			battery__S=S,
			__controller__field_weakening=bool(field_weakening),
			actuator__n_series=n_series,
		)
		return pickle_encode(sysr)


	@callback(
		Output('motor-props-label', 'children'),
		Input('system', 'data'),
	)
	def compute_handler_actuator_props(system):
		system = pickle_decode(system)
		return \
			f'Mass:\t {system.actuator.motor.mass.total:0.3f} \tkg\n' \
			f'Kt:\t\t {system.actuator.motor.electrical.Kt:0.3f} \tNm/A\n' \
			f'R: \t\t {system.actuator.motor.electrical.R:0.3f} \tohm\n' \
			f'L: \t\t {system.actuator.motor.electrical.L*1000:0.3f} \tmH'

	@callback(
		Output('battery-weight-label', 'children'),
		Input('system', 'data'),
	)
	def compute_handler_battery_weight(system):
		system = pickle_decode(system)
		return f'Weight: {system.battery.weight:0.3f} kg'

	@callback(
		Output('ranges', 'data'),

		Input('system', 'data'),
		Input('rescale', 'value'),
		Input('resolution-slider', 'value'),
	)
	def compute_handler_ranges(system, rescale, resolution):
		"""update range estimates"""
		_n_rpm = n_rpm * resolution
		_n_torque = n_torque * resolution

		if rescale:
			system = pickle_decode(system)
			max_rpm, max_torque = system_detect_limits(system, fw=1.3)
			rpm_range = np.linspace(0, max_rpm, _n_rpm, endpoint=True)
			torque_range = np.linspace(-max_torque, +max_torque, _n_torque, endpoint=True)
			return jsonpickle.dumps((rpm_range, torque_range))

		return dash.no_update


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
		Input('table-thermal', 'data'),
	)
	def compute_handler(sysr, ranges, thermals):
		"""do motor calcs and write to data store"""
		sysr = pickle_decode(sysr)
		rpm_range, torque_range = jsonpickle.loads(ranges)

		res = calc_stuff(sysr, rpm_range, torque_range, thermals)
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

		rpm_range, torque_range = jsonpickle.loads(ranges)
		graphs, contours = jsonpickle.loads(data)

		copper_loss, iron_loss, bus_power, mechanical_power, Iq, Id, torque = graphs
		thermal_contours, acceleration_contours, fw_lim_contour, efficiency_contour = contours

		dissipation = copper_loss + iron_loss
		# efficiency = 1 - dissipation / np.abs(bus_power)
		efficiency = 1 - dissipation / np.maximum(np.abs(mechanical_power), np.abs(bus_power))
		acceleration = system.acceleration(rpm_range, torque)

		x, X = system.x_axis(rpm_range)
		y, Y = system.y_axis(torque_range)

		import xarray as xr
		def wrap(arr, name=''):
			return xr.DataArray(
				data=arr,
				dims=[y, x],
				coords={x: X, y: Y},
				name=name
			)


		if plot_key == 'efficiency':
			# FIXME: add custom color scale with high contrast at the high end
			fig = px.imshow(wrap(efficiency, plot_key), origin='lower', zmin=0, zmax=1, color_continuous_scale='rainbow')
		if plot_key == 'power':
			zlim = np.abs(np.nan_to_num(bus_power)).max()
			fig = px.imshow(wrap(bus_power, plot_key), origin='lower', zmin=-zlim, zmax=zlim, color_continuous_scale='RdBu_r')
		if plot_key == 'dissipation':
			zlim = np.abs(np.nan_to_num(dissipation)).max()
			fig = px.imshow(wrap(dissipation, plot_key), origin='lower', zmin=0, zmax=zlim, color_continuous_scale='Electric')
		if plot_key == 'Id':
			zlim = np.abs(np.nan_to_num(Id)).max()
			fig = px.imshow(wrap(Id, plot_key), origin='lower', zmin=-zlim, zmax=zlim, color_continuous_scale='RdBu')
		if plot_key == 'acceleration':
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

		# add x axis
		fig.add_trace(
			go.Scatter(x=X, y=X*0, name='x-axis', mode='lines', line_color='gray', showlegend=False,)
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

		# FIXME: move to declaration?
		# fig.update_layout(margin=dict(b=0, t=0, l=0, r=0))

		return fig

	return app

