import numpy
import numpy as np

from pypowertrain.utils import *
from pypowertrain.components.actuator import Actuator
from pypowertrain.components.battery import Battery


@dataclass
class System(Base):
	"""Simple battery-controller-motor system"""
	battery: Battery
	actuator: Actuator

	@property
	def weight(self):
		return self.battery.weight + self.actuator.weight

	@property
	def inertia(self):
		# FIXME: move to actuator?
		return self.actuator.motor.mass.rotor_inertia * self.actuator.gearing.ratio ** 2

	def load(self, rpm):
		"""override"""
		# by default, lets have only a light aero drag on the output shaft
		f = rpm / 1000
		return f * np.abs(f)

	def x_axis(self, rpm=0):
		return 'rpm', rpm
	def y_axis(self, nm=0):
		return 'Nm', nm



def system_limits(
	system: System,
	trange, rpm,
	gridsize=500
):
	"""
	Find powertrain operating points,
	that attain a given torque, with the least energy consumption,
	where possible, given the various physical constraints along the powertrain

	Parameters
	----------
	system: System
	trange: output torque range
	rpm: output rpm range
	gridsize: number of Id points to search over

	Returns
	-------
	List of graphs

	References
	-------
	https://nl.mathworks.com/help/mcb/gs/pmsm-constraint-curves-and-their-application.html#PMSMConstraintCurvesAndTheirApplicationExample-6
	"""
	actuator = system.actuator
	battery = system.battery
	motor = actuator.motor
	controller = actuator.controller

	if controller.field_weakening:
		arange = np.linspace(-actuator.phase_current_limit, +actuator.phase_current_limit, gridsize+1)
	else:
		arange = np.array([0.])	# always pursue Id=0

	# map output ranges to motor ranges
	# FIXME: gearing efficiency makes torque function of rpm, through sign alone.
	#  need to move this into process loop below to make that work
	#  alternatively; dont paramerize in terms of output torque? stick with EM torque?
	rpm, trange = actuator.gearing.backward(rpm, trange)

	salience = motor.electrical.salience # -(motor.Lq - motor.Ld)
	Id, em_torque = np.meshgrid(arange, trange)
	Iq = em_torque / (3/2) / motor.geometry.pole_pairs / (motor.flux + Id * salience)
	# FIXME: encode saturation relation; torque goes down with excessive (pm_flux - id)**2+iq**2
	#  that is, we might use FW at high torque to combat saturation

	# should ripple count towards phase current limits? i guess so conservatively.
	Is = Id**2 + Iq**2 + actuator.ripple_current**2

	bus_resistance = actuator.bus.resistance + battery.resistance
	Rt = actuator.phase_resistance


	Vlim = lambda omega: (Id*Rt - omega*motor.electrical.Lq*Iq)**2 + (Iq*Rt + omega*motor.electrical.Ld*Id + omega*motor.flux)**2
	def process_omega(omega_axle_hz):
		"""construct and intersect amp and V limits for a given omega"""
		omega_elec_hz = omega_axle_hz * motor.geometry.pole_pairs
		omega_axle_rad = omega_axle_hz * 2 * np.pi
		omega_elec_rad = omega_elec_hz * 2 * np.pi

		# FIXME: work Id-FW-dependence into iron drag? Id division should be about equal to demag current limit
		#  effect seems quite minimal in practice; like 3% kph continuous rating
		drag_torque = motor.iron_drag(omega_axle_hz) #* (1+Id/300)**2
		copper_loss = Is*Rt
		iron_loss = omega_axle_rad * drag_torque + omega_elec_rad**2*Is*0
		dissipation = copper_loss + iron_loss
		mechanical_torque = em_torque - drag_torque
		mechanical_power = mechanical_torque * omega_axle_rad
		bus_power = dissipation + mechanical_power

		bus_current = bus_power / battery.voltage	# FIXME: solve bus voltage sag properly; get errors right now for high resistsances
		effective_bus_voltage = battery.voltage - bus_current * bus_resistance
		effective_voltage = actuator.effective_voltage(effective_bus_voltage)

		mask = np.abs(omega_elec_hz) < controller.freq_limit
		mask = np.logical_and(mask, Vlim(omega_elec_rad) < effective_voltage ** 2)
		mask = np.logical_and(mask, Is < actuator.phase_current_limit**2)
		# cap bus power
		mask = np.logical_and(mask, bus_power < battery.peak_discharge_power)
		mask = np.logical_and(mask, bus_power > -battery.peak_charge_power)
		# mech power here not appropriate; wrong sign of dissipation.
		# yet we are missing something here no? caps will dissipate
		# when switching large current zero bus regen braking
		mask = np.logical_and(mask, np.abs(bus_power) < actuator.power_limit)

		# of all valid Id options for a given em_torque, we pick the one most favorable to battery
		ma = np.ma.array(bus_power, mask=1-mask)
		# this is a reduction over Id
		mask = np.ma.min(ma, axis=1).mask
		idx = np.ma.argmin(ma, axis=1)
		i = np.arange(len(trange))

		# map to output torque
		_, mechanical_torque = actuator.gearing.forward(0, mechanical_torque)

		return [
			np.ma.array(o[i, idx], mask=mask).filled(np.nan)
			for o in [copper_loss, iron_loss, bus_power, mechanical_power, Iq, Id, mechanical_torque]
		]
	# good old for loop rather than vectorize over rpm; memory use might explode otherwise
	outputs = np.array([process_omega(o) for o in rpm/60])
	return np.moveaxis(outputs, [1, 2, 0], [0, 1, 2])




def round(x, digits):
	"""round to a number of significant digits"""
	shift = 10**int(np.log10(x) - digits + 1)
	return np.ceil(x / shift) * shift


def system_detect_limits(system, fw=3, frac=0.95, padding=1.1):
	"""auto-detect some reasonably tight limits on the actuator system"""
	max_torque = system.actuator.peak_torque * 1.1
	max_rpm = system.actuator.motor.electrical.Kv * system.battery.voltage * fw
	trange = np.linspace(-max_torque, +max_torque, 20, endpoint=True)
	rpm = np.linspace(0, max_rpm, 20, endpoint=False)
	_, _, _, _, _, _, torque = system_limits(system, trange, rpm)
	max_torque = np.max(np.abs(np.nan_to_num(torque)))
	max_rpm = rpm[::-1][np.argmin(np.mean(np.isnan(torque), axis=0)[::-1] > frac)]
	max_rpm = round((max_rpm) * padding, 2)
	max_torque = round((max_torque) * padding, 2)
	return max_rpm, max_torque



def calc_stuff(system, rpm_range, torque_range, thermal_specs):
	"""do heavy calcs on system"""
	# eval the system performance graphs
	things = system_limits(system, torque_range, rpm_range).astype(np.float32)
	copper_loss, iron_loss, bus_power, mechanical_power, Iq, Id, torque = things

	# construct thermal curves
	linear_velocity = rpm_range * 0
	thermals = {
		s['color']: system.actuator.temperatures(
			linear_velocity, rpm_range, copper_loss, iron_loss, dt=s['dt']) - s['dT']
		for s in thermal_specs
	}

	return things, thermals


def system_dash(
	system: System,
	n_rpm=50,
	n_torque=50,
	max_rpm=None,
	max_torque=None,
	targets=None,
):
	"""dash plotly plot of system limits

	add range sliders, to set bounds for optimization
	then add a button to do some grad descent within the bounds given by the slider

	https://dash.plotly.com/sharing-data-between-callbacks
	FIXME:	precomp thermals and their contours too.
	  then send via jsonpiakle, with numpy extension.

	"""
	import plotly.graph_objects as go
	from dash import Dash, html, dcc, callback, Output, Input, State, ctx, dash_table
	import dash
	import dash_bootstrap_components as dbc
	import plotly.express as px

	import jsonpickle
	import jsonpickle.ext.numpy as jsonpickle_numpy
	jsonpickle_numpy.register_handlers()

	import pickle	# fixme: can we drop pickle for json here? perhaps some small changes to Base dataclass?
	import codecs

	plots = ['dissipation', 'power', 'efficiency', 'Id', 'acceleration']

	turns_tooltip = "This changes the number of turns, at equal total copper volume in the coils. As such it does not impact the motor mass. There is no right or wrong here; but it is important to tune it properly to the intended application. This scaling law is exact; although it ignores enamel thickness, eddy-losses, and winding-practicalities."
	radius_tooltip = 'Radial scaling has a quadratic effect on both motor mass and torque. Dialing in this value to one appropriate for your design constraints is often one of the more important ones. Note that radial scaling is exact in terms of the electrical parameters. The mass predictions of the frameless parts of the motor are exact, but the main uncertainty here is in the mass of the structural parts; depending on the scale different construction technicues might be appropriate'
	slot_depth_tooltip = 'The slot depth has a direct influence on the amount of copper available, and tends to scale the thermal curves along the y-axis. It also has a profound effect on motor weight, since most of the mass in in the teeth and their copper. Very few caveats apply to this scaling law and extrapolations should be valid over a wide range of scalings'
	axial_tooltip = 'Scaling of the axial length of the motor. Too low values will results in excessive negative end-effects, such as coil overhang. Longer values tend to have more restrictive thermal performance, per kg of motor'
	reluctance_tooltip = 'Scaling pf the amount of mu-0 in the motor magnetic circuit, at equal flux-density, and equal iron saturation. This is a fairly safe rescaling, although it ignores flux leakage and fringing effects. It will impact inductance and demagnetization limits.'
	slot_width_tooltip = 'Scaling of the slot width. This trades copper current area for iron flux area. Narrower slots will raise resistance, and lower iron losses. Increasing slot width is generally not recommended, since motors tend to be designed near their iron saturation limits. A slight decrease might make sense though, if the resistance-headroom exists. Iron saturation is not currently modelled but more iron should raise the iron saturation limit too.'
	frequency_tooltip = "Scaling of the number of poles and slots, at equal gap radius. Note that these fractional scalings are generally not realizable; but adjusting this value should give quite an accurate idea, if your application would benefit from a motor with a different pole count."
	# flux_tooltip = 'Flux scaling, at equal field density. This increases magnet volume and tooth width, at equal iron field level'

	# FIXME: why is editor not working?
	thermal_specs = [
		{'color': 'yellow', 'dt': 5000, 'dT': 60},
		{'color': 'orange', 'dt': 60, 'dT': 60},
		{'color': 'red', 'dt': 2, 'dT': 40},
	]
	thermal_table = dash_table.DataTable(
		id='table-thermal',
		columns=([
			{'id': p, 'name': p, 'type': 'text' if isinstance(v, str) else 'numeric'}
			for p, v in thermal_specs[0].items()
		]),
		data=thermal_specs,
		editable=True
	)

	from pypowertrain.library import odrive, grin, moteus
	controllers = {
		'odrive.pro': odrive.pro(),
		'odrive.pro_nominal': odrive.pro_nominal(),
		'odrive.pro_overclock': odrive.pro_overclock(),
		'grin.phaserunner': grin.phaserunner(),
		'moteus.n1': moteus.n1()
	}
	motors = {
		'odrive.M8325s_100KV': odrive.M8325s_100KV(),
		'grin.all_axle': grin.all_axle(),
		'moteus.mj5208': moteus.mj5208()
	}

	# app = Dash()
	app = dash.Dash(external_stylesheets=[dbc.themes.BOOTSTRAP])

	app.layout = html.Div(children=[
		# html.H1(children='pyPowerTrain', style={'textAlign': 'center'}),

		dbc.Container(
			dbc.Row([
				dbc.Col(
					width=8,
					# main plotting area
					children=dcc.Graph(id='graph-content'),
				),
				dbc.Col(
					width=4,
					children=dbc.Tabs([
						dbc.Tab(label='Motor', children=[
							dcc.Dropdown(list(motors.keys()), 'grin.all_axle', id='dropdown-motor'),
							html.Div(id='motor-weight-label'),
							html.Label('Turns'),
							dcc.Slider(1, 10, 1, value=system.actuator.motor.geometry.turns, id='turns-slider'),
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

						]),
						dbc.Tab(label='Battery', children=[
							html.Label('Battery charge'),
							dcc.Slider(0.0, 1.0, 0.1, value=system.battery.charge_state, id='charge-slider'),
							html.Label('Battery P'),
							dcc.Slider(1, 20, 1, value=system.battery.P, id='P-slider'),
							html.Label('Battery S'),
							dcc.Slider(1, 20, 1, value=system.battery.S, id='S-slider'),
							html.Div(id='battery-weight-label'),
						]),
						dbc.Tab(label='Controller', children=[
							html.Label('Controller'),
							dcc.Dropdown(list(controllers.keys()), 'odrive.pro', id='dropdown-controller'),
							dcc.Checklist(options=['Field weakening'], value=['Field weakening'], id='field-weakening'),
						]),
						dbc.Tab(label='Visual', children=[
							dcc.Dropdown(plots, 'dissipation', id='dropdown-selection'),
							dcc.Checklist(options=['Rescale'], value=['Rescale'], id='rescale'),
							# html.Button(id='refine', children=html.Label('Refine')),
							html.Label('Resolution'),
							dcc.Slider(1, 4, 1, value=1, id='resolution-slider'),
							thermal_table,
							html.Div(id='debug'),
						]),
					]),

				),
			]),


		),


		# dcc.Store(id='coarsedata'),		# FIXME: split this more granular? sep arrays? coarse and fine?
		dcc.Store(id='system'),
		dcc.Store(id='ranges'),
		dcc.Store(id='graphdata'),
	])


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

	)
	def compute_handler_system(
			motor, turns, radius, slot_depth, length, slot_width, reluctance, frequency,
			charge, P, S,
			controller, field_weakening,
	):
		"""Put all modifiers to the system object here"""
		# FIXME: place base object selection upstream?
		sysr = system.replace(
			__motor=motors[motor],
			__controller=controllers[controller],
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
		)
		return codecs.encode(pickle.dumps(sysr), "base64").decode()

	@callback(
		Output('motor-weight-label', 'children'),
		Input('system', 'data'),
	)
	def compute_handler_actuator_weight(system):
		system = pickle.loads(codecs.decode(system.encode(), "base64"))
		return f'Weight: {system.actuator.weight:0.3f} kg'

	@callback(
		Output('battery-weight-label', 'children'),
		Input('system', 'data'),
	)
	def compute_handler_battery_weight(system):
		system = pickle.loads(codecs.decode(system.encode(), "base64"))
		return f'Weight: {system.battery.weight:0.3f} kg'

	@callback(
		Output('ranges', 'data'),
		Input('system', 'data'),
		Input('rescale', 'value'),
		Input('resolution-slider', 'value'),
	)
	def compute_handler_ranges(sysr, rescale, resolution):
		"""update range estimates"""
		_n_rpm = n_rpm * resolution
		_n_torque = n_torque * resolution

		if rescale:
			sysr = pickle.loads(codecs.decode(sysr.encode(), "base64"))
			max_rpm, max_torque = system_detect_limits(sysr, fw=2)
			rpm_range = np.linspace(0, max_rpm, _n_rpm + 1, endpoint=True)
			torque_range = np.linspace(-max_torque, +max_torque, _n_torque + 1, endpoint=True)
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
		sysr = pickle.loads(codecs.decode(sysr.encode(), "base64"))
		# dash.distributed.print(str(thermals))
		rpm_range, torque_range = jsonpickle.loads(ranges)

		res = calc_stuff(sysr, rpm_range, torque_range, thermals)
		# FIXME: only transport arrays that are actually used. pre-extract contours?
		# copper_loss, iron_loss, bus_power, mechanical_power, Iq, Id, torque = things
		return jsonpickle.dumps(res)


	@app.callback(
		Output('graph-content', 'figure'),
		Input('dropdown-selection', 'value'),
		Input('graphdata', 'data'),
		State('ranges', 'data'),

	)
	def update_graph(value, data, ranges):
		things, thermals = jsonpickle.loads(data)
		copper_loss, iron_loss, bus_power, mechanical_power, Iq, Id, torque = things
		rpm_range, torque_range = jsonpickle.loads(ranges)

		x, X = system.x_axis(rpm_range)
		y, Y = system.y_axis(torque_range)

		import xarray as xr
		def wrap(arr, name=''):
			return xr.DataArray(
				data=arr,
				dims=["Nm", "rpm"],
				coords={"rpm": X, 'Nm': Y},
				name=name
			)

		def contour(fig, data, color="rgba(0,0,0,1)"):
			import skimage.measure
			contours = skimage.measure.find_contours(data, 0)

			for contour in contours:
				x = numpy.interp(contour[:, 1], np.arange(len(X)), X)
				y = numpy.interp(contour[:, 0], np.arange(len(Y)), Y)
				fig.add_trace(
					go.Scatter(
						x=x, y=y, mode='lines',
						line_color=color,
						showlegend=False,
					))
				fig.add_trace(
					go.Scatter(
						x=x, y=y, mode='lines', line_dash='dash',
						line_color='black',
						showlegend=False,
					))

		efficiency = np.where(
			torque >= 0,
			np.abs(mechanical_power) / np.abs(bus_power),
			np.abs(bus_power) / np.abs(mechanical_power)
		)

		if value == 'efficiency':
			# FIXME: add custom color scale with high contrast at the high end
			fig = px.imshow(wrap((efficiency), value), origin='lower', zmin=0, zmax=1, color_continuous_scale='rainbow')
		if value == 'power':
			zlim = np.abs(np.nan_to_num(bus_power)).max()
			fig = px.imshow(wrap(bus_power, value), origin='lower', zmin=-zlim, zmax=zlim, color_continuous_scale='RdBu_r')
		if value == 'dissipation':
			dissipation = copper_loss + iron_loss
			zlim = np.abs(np.nan_to_num(dissipation)).max()
			fig = px.imshow(wrap(dissipation, value), origin='lower', zmin=0, zmax=zlim, color_continuous_scale='Electric')
		if value == 'Id':
			zlim = np.abs(np.nan_to_num(Id)).max()
			fig = px.imshow(wrap(Id, value), origin='lower', zmin=-zlim, zmax=zlim, color_continuous_scale='RdBu')
		if value == 'acceleration':
			# FIXME: make this an overloadable system property
			a = torque / system.actuator.motor.mass.rotor_inertia
			zlim = np.abs(np.nan_to_num(a)).max()
			fig = px.imshow(wrap(a, value), origin='lower', zmin=-zlim, zmax=zlim, color_continuous_scale='RdBu')

		# add thermal lines
		# FIXME: precompute lines
		for c, m in thermals.items():
			contour(fig, m, c)

		# add FW limit
		q = np.where(Id==0, 1, np.nan_to_num(Id, nan=-1))
		import skimage.filters
		lim = skimage.filters.gaussian(q, [3, 1])+Id*0 + 3
		contour(fig, lim, color="white")

		# max efficiency region FIXME whats the issue here?
		contour(fig, efficiency - np.nan_to_num(efficiency*(torque>0), nan=0).max() * 0.95, color='green')

		# add x axis
		fig.add_trace(
			go.Scatter(x=rpm_range, y=rpm_range*0, name='x-axis', mode='lines', line_color='gray', showlegend=False,)
		)

		# FIXME: pass in system here? or precomp the lines?
		# add acceleration based load lines
		# a = (torque - system.load(rpm_range)) / system.inertia
		# for ll in [-100, 0, 100]:
		# 	contour(fig, a - ll, color='black')

		return fig

	app.run(debug=False)

	# if targets is not None:
	# 	t_torque, t_rpm, t_dissipation, t_weight = np.array(targets)
	# 	fig.add_trace(
	# 		go.Scatter(
	# 			x=t_rpm, y=t_torque,
	# 			showlegend=False,
	# 		))


def system_plot(
	system: System,
	n_rpm=200,
	n_torque=500,
	max_rpm=None,
	max_torque=None,
	targets=None,
):
	"""mpl plots of system limits"""
	import matplotlib.pyplot as plt

	# setup calculation grids
	if max_torque is None:
		max_rpm, max_torque = system_detect_limits(system)

	rpm_range = np.linspace(0, max_rpm, n_rpm+1, endpoint=True)
	torque_range = np.linspace(-max_torque, +max_torque, n_torque + 1, endpoint=True)

	# eval the system performance graphs
	copper_loss, iron_loss, bus_power, mechanical_power, Iq, Id, torque = system_limits(system, torque_range, rpm_range)
	dissipation = copper_loss + iron_loss

	# construct thermal curves
	thermal_specs = [
		{'color': 'yellow', 'dt': 5000, 'dT': 60},
		{'color': 'orange', 'dt': 60, 'dT': 60},
		{'color': 'red', 'dt': 2, 'dT': 40},
	]
	# NOTE: hardcoded zero here now; but should be configurable on the system object
	linear_velocity = rpm_range * 0.0
	thermals = {
		s['color']: system.actuator.temperatures(
			linear_velocity, rpm_range, copper_loss, iron_loss, dt=s['dt']) - s['dT']
		for s in thermal_specs
	}

	x, X = system.x_axis(rpm_range)
	y, Y = system.y_axis(torque_range)

	def imshow(im, **kwargs):
		a = plt.pcolormesh(X, Y, im, mouseover=True, **kwargs)
		plt.colorbar()
		return a
	def contour(im, **kwargs):
		return plt.contour(X, Y, im, **kwargs)

	def default_annotate():
		# V = system.battery.voltage - rpm / system.actuator.motor.electrical.Kv
		# A = V* system.actuator.controller.modulation_factor / system.actuator.motor.R
		# plt.plot(kph, A*system.actuator.motor.electrical.Kt)

		# delineate FW-region
		contour(Id, levels=[-1], colors='white')
		# plot x axis
		plt.plot(X, X * 0, c='gray')
		# plot thermal limits
		for c, m in thermals.items():
			contour(m, levels=[0], colors=c)

		if targets is not None:
			t_torque, t_rpm, t_dissipation, t_weight = [np.array(t) for t in targets]
			plt.scatter(t_rpm, t_torque)

		plt.xlabel(x)
		plt.xticks(X[::len(X)//10])
		plt.ylabel(y)
		plt.yticks(Y[::len(Y)//10])

	plt.figure()
	efficiency = np.where(
		torque >= 0,
		np.abs(mechanical_power) / np.abs(bus_power),
		np.abs(bus_power) / np.abs(mechanical_power)
	)
	imshow(efficiency, cmap='nipy_spectral', clim=(0, 1))
	plt.title('Efficiency')
	default_annotate()

	plt.figure()
	current_range = np.abs(np.nan_to_num(Id)).max()
	imshow(Id, cmap='bwr', clim=(-current_range, current_range))
	plt.title('Id')
	default_annotate()

	# bus power levels
	plt.figure()
	blim = np.abs(np.nan_to_num(bus_power)).max()
	imshow(bus_power, cmap='bwr', clim=(-blim, blim))
	plt.title('Bus power')
	default_annotate()

	# dissipation
	plt.figure()
	imshow(dissipation, cmap='cool', clim=(0, np.nan_to_num(dissipation, nan=0).max()))
	plt.title('Dissipation')
	default_annotate()

	plt.show()


def system_plotly(
	system: System,
	n_rpm=200,
	n_torque=500,
	max_rpm=None,
	max_torque=None,
	targets=None,
):
	"""plotly plots of system limits"""
	import plotly.graph_objects as go
	import plotly.express as px

	# setup calculation grids
	if max_torque is None:
		max_rpm, max_torque = system_detect_limits(system, fw=2)

	rpm_range = np.linspace(0, max_rpm, n_rpm+1, endpoint=True)
	torque_range = np.linspace(-max_torque, +max_torque, n_torque + 1, endpoint=True)

	# eval the system performance graphs
	copper_loss, iron_loss, bus_power, mechanical_power, Iq, Id, torque = system_limits(system, torque_range, rpm_range)
	dissipation = copper_loss + iron_loss

	# construct thermal curves
	thermal_specs = [
		{'color': 'yellow', 'dt': 5000, 'dT': 60},
		{'color': 'orange', 'dt': 60, 'dT': 60},
		{'color': 'red', 'dt': 4, 'dT': 20},
	]
	linear_velocity = rpm_range * 0
	thermals = {
		s['color']: system.actuator.temperatures(
			linear_velocity, rpm_range, copper_loss, iron_loss, dt=s['dt']) - s['dT']
		for s in thermal_specs
	}

	x, X = system.x_axis(rpm_range)
	y, Y = system.y_axis(torque_range)
	import xarray as xr

	def wrap(arr, name=''):
		return xr.DataArray(
			data=arr,
			dims=["Nm", "rpm"],
			coords={"rpm": X, 'Nm': Y},
			name=name
		)

	def contour(data, color="rgba(0,0,0,1)"):
		import skimage.measure
		contours = skimage.measure.find_contours(data, 0)

		for contour in contours:
			x = numpy.interp(contour[:, 1], np.arange(len(X)), X)
			y = numpy.interp(contour[:, 0], np.arange(len(Y)), Y)
			fig.add_trace(
				go.Scatter(
					x=x, y=y, mode='lines',
					line_color=color,
					showlegend=False,
				))
			fig.add_trace(
				go.Scatter(
					x=x, y=y, mode='lines', line_dash='dash',
					line_color='black',
					showlegend=False,
				))

	# Id[250:260, 20:30] = -6
	zlim = np.abs(bus_power).max()
	fig = px.imshow(wrap(bus_power), origin='lower', zmin=-zlim, zmax=zlim, color_continuous_scale='RdBu_r')
	# fig = go.Figure(
	# 	go.Image(x=X, y=Y, z=Id)
	# )
	for c, m in thermals.items():
		contour(m, c)
	# add FW limit
	# lim = skimage.filters.gaussian(np.nan_to_num(Id, nan=0), 2)+Id*0 + 3
	q = np.where(Id==0, 1, np.nan_to_num(Id, nan=-1))
	import skimage.filters
	lim = skimage.filters.gaussian(q, [3, 1])+Id*0 + 3
	contour(lim, color="white")

	# torque lines
	for t in np.linspace(-200, 200, 9, endpoint=True):
		# FIXME: add system.load type offset here
		contour(torque - t - (rpm_range/200)**2, 'gray')
	fig.show()




	# if targets is not None:
	# 	t_torque, t_rpm, t_dissipation, t_weight = np.array(targets)
	# 	fig.add_trace(
	# 		go.Scatter(
	# 			x=t_rpm, y=t_torque,
	# 			showlegend=False,
	# 		))


def sample_graph(graph, sample_point):
	"""find i,j in graph closest to sampling point for each column"""
	delta = np.nan_to_num(graph, nan=-1e9) - sample_point
	i = np.argmin(delta**2, axis=0)
	j = np.arange(len(i))
	return i, j


def graph_sampler(graph, sample=0, s=0, e=None):
	"""find crossing point of graph in each column"""
	if e is None: e = graph.shape[1]+1
	delta = graph[:,s:e] - sample
	delta = np.nan_to_num(delta, nan=1e9)
	p, n = np.argsort(delta**2, axis=0)[:2]
	j = np.arange(len(p))
	dp, dn = delta[p, j], delta[n, j]
	wn = dp / (dp - dn)
	wn = np.clip(wn, 0, 1)
	wp = 1 - wn
	def inner(data):
		return data[:,s:e][p, j] * wp + data[:,s:e][n, j] * wn
	return inner


def integrate_traject(v, a, d=0, reverse=False):
	"""calculate braking/accel distance and dissipation

	velocity, acceleration and dissipation sample points

	could be either in radial or linear units
	"""
	v, a, d = np.broadcast_arrays(v, a, d)
	if reverse:
		v = v[::-1]
		a = a[::-1]
		d = d[::-1]
	dv = np.diff(v)
	x = 0
	e = 0
	t = 0
	for i in range(len(dv)):
		dt = dv[i] / a[i]
		t += dt
		x += v[i] * dt
		e += d[i] * dt
	return x, e, t


def integrate_step(rpm, accel, size):
	"""calculate time taken to perform a step function
	of `size` units, from standstill to standstill,
	given an acceleration graph
	"""
	f = graph_sampler(accel, 300)(accel)
	r = graph_sampler(accel, -300)(accel)
	radps = rpm/60*2*np.pi

	def step(i):
		x1, _, t1 = integrate_traject(radps[:i], f[:i])
		x2, _, t2 = integrate_traject(radps[:i], r[:i], reverse=True)
		return x1+x2, t1+t2
	for i in range(len(rpm)):
		angle, time = step(i)
		if angle > size:
			return time, rpm[i]
