from typing import List, Dict, Tuple
import dataclasses
from _operator import attrgetter
import inspect

dataclass = dataclasses.dataclass(frozen=True)

@dataclass
class Base:
	"""Base class for immutable data class hierarchy,
	that allows for replacing and rescaling attributes"""
	# FIXME: replace the rescale paradigm with pure cached property logic?

	def replace(self, /, **kwargs):
		"""Intercept arguments to be passed to rescaling"""
		if hasattr(self, 'rescale'):
			sig = inspect.signature(self.rescale).parameters.keys()
			place = {k: v for k, v in kwargs.items() if k not in sig}
			scale = {k: v for k, v in kwargs.items() if k in sig}
			return self.rescale(**scale)._replace(**place)
		else:
			return self._replace(**kwargs)

	def _replace(obj, /, **kwargs):
		"""
		Like dataclasses.replace but can replace an arbitrarily nested attributes
		"""
		for k, v in kwargs.items():
			k = k.replace("__", ".")

			while "." in k:
				prefix, _, attr = k.rpartition(".")
				deep_attr = attrgetter(prefix)(obj)
				# v = dataclasses.replace(deep_attr, **{attr: v})
				v = deep_attr.replace(**{attr: v})
				k = prefix
			obj = dataclasses.replace(obj, **{k: v})
		return obj
