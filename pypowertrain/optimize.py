import numpy as np

from pypowertrain.system import *


def system_score(
	system: System,
	targets,
	gridsize=50,
):
	"""score a systems performance relative to torque and dissipation target points"""
	t_torque, t_rpm, t_dissipation, t_weight = [np.array(t) for t in targets]

	# setup calculation grids
	torque_range = system.actuator.peak_torque * 1.1
	trange = np.linspace(-torque_range, +torque_range, gridsize+1, endpoint=True)
	outputs = system_limits(system, trange, t_rpm, gridsize=100)
	copper, iron, bus_power, mechanical_power, Iq, Id, torque = outputs
	dissipation = copper + iron

	# sample closest realizable torque at given kph
	i, j = sample_graph(torque, t_torque)
	s_torque = torque[i, j]
	s_dissipation = dissipation[i, j]
	# higher is worse
	# d = np.clip((t_torque - s_torque) / t_torque, 0, 10)
	d = (t_torque - s_torque) / t_torque
	w = np.where(d<0, *t_weight.T)
	score_torque = np.mean(d * w) / np.mean(w+1e-6)
	# higher is worse
	# d = np.clip((s_dissipation - t_dissipation) / t_dissipation, 0, 10)
	d =(s_dissipation - t_dissipation) / t_dissipation
	w = np.where(d<0, *t_weight.T)
	score_dissipation = np.mean(d * w) / np.mean(w+1e-6)

	return np.array([score_torque, score_dissipation])


def system_optimize(
	system: System,
	bounds: Dict,
	targets,
	conditions: List[Dict],
):
	"""Optimize a motor-controller-battery system,
	over the parameters specified in `bounds`,
	averaged over all `conditions`,
	to best satisfy all `targets` operating points"""
	keys = list(bounds.keys())
	values = np.array([bounds[k] for k in keys])
	integrality = [isinstance(bounds[a][0], int) for a in keys]

	def evolve(args):
		kwargs = dict(zip(keys, args))
		return system.replace(**kwargs)

	def objective(args):
		evolved_system = evolve(args)
		scores = [system_score(evolved_system.replace(**c), targets) for c in conditions]
		torque_score, thermal_score = np.mean(scores, axis=0)
		weight_score = evolved_system.weight
		return torque_score + thermal_score + weight_score / 20

	from scipy.optimize import differential_evolution
	res = differential_evolution(
		objective,
		bounds=values,
		maxiter=100,
		polish=True,
		integrality=integrality,
	)
	return evolve(res.x)
