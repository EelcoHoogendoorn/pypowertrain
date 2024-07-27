from typing import List, Dict, Tuple
import dataclasses
from _operator import attrgetter
import inspect
from functools import cached_property, lru_cache
import numpy as np

dataclass = dataclasses.dataclass(frozen=True)


def all_paths(obj):
	"""find all attributes in the object hierarchy that we could call 'replace' on"""
	if issubclass(type(obj), Base):
		for f in dataclasses.fields(obj):
			a = getattr(obj, f.name)
			yield f.name
			for p in all_paths(a):
				yield f.name + '.' + p
		try:
			for k in inspect.signature(obj.rescale).parameters.keys():
				# FIXME: filter duplicate fields and rescales args? should diappear with setters
				yield k
		except:
			pass


def expand_paths(k, obj):
	"""given an object and key/path with . and wildcards,
	yield the full path if is present on the object hierarchy
	"""
	for p in all_paths(obj):
		if p.endswith(k):
			# dont want to match to partial leafs
			if p.rpartition('.')[-1] == k.rpartition('.')[-1]:
				yield p


@dataclass
class Base:
	"""Base class for immutable data class hierarchy,
	that allows for replacing and rescaling attributes"""
	# FIXME: replace the rescale paradigm with pure cached property logic?

	def replace_norescale(self, **kwargs):
		return dataclasses.replace(self, **kwargs)

	def rescale(self, **kwargs):
		return self
	def rescale_replace(self, /, **kwargs):
		"""Intercept arguments to be passed to rescaling"""
		# if hasattr(self, 'rescale'):
		sig = inspect.signature(self.rescale).parameters.keys()
		scale = {k: v for k, v in kwargs.items() if k in sig}
		kwargs = {k: v for k, v in kwargs.items() if k not in sig}
		self = self.rescale(**scale)
		return self.replace_norescale(**kwargs)

	def replace(obj, /, **kwargs):
		"""
		Like dataclasses.replace but can replace an arbitrarily nested attributes
		leading underscores __ denote a wildcard pattern,
		which will be replaced everywhere it matches in the object hierarchy
		"""
		for kk, v in kwargs.items():
			kk = kk.replace("__", ".")

			for k in expand_paths(kk, obj):
				obj = obj.replace_inner(**{k:v})
		return obj

	def replace_inner(obj, /, **kwargs):
		"""
		Like dataclasses.replace but can replace an arbitrarily nested attributes
		"""
		for k, v in kwargs.items():
			# kk = kk.replace("__", ".")

			while "." in k:
				prefix, _, attr = k.rpartition(".")
				try:
					deep_attr = attrgetter(prefix)(obj)
					# v = dataclasses.replace(deep_attr, **{attr: v})
					v = deep_attr.rescale_replace(**{attr: v})
				except:

					pass
				k = prefix
			obj = obj.rescale_replace(**{k: v})

		return obj


@dataclass
class Scaled(Base):
	"""Base class for dimensionally scalable properties

	Like an attr-dict, the attributes of which are viewed through the lens of scaling laws
	"""
	# FIXME: can we shield these from fields-dict? or make private with underscore?
	scaling: dict
	context: List[str]
	attrs: dict

	@classmethod
	def init(cls, context, scaling, **kwargs):
		# write attrs to object in dimensionless terms
		return cls(context=context, scaling=scaling, attrs={}, **kwargs)

	def from_dimensional(self, attrs):
		# write dimensional attrs to object in dimensionless terms
		return self._set_attrs(attrs)
	def from_dimensionless(self, attrs):
		# write dimensionless attrs to object in dimensionless terms
		self.attrs.update(attrs)
		return self

	# @classmethod
	# def from_unscaled(cls, context, scaling, attrs, **kwargs):
	# 	# write attrs to object in dimensionless terms
	# 	return cls(context=context, scaling=scaling, attrs={}, **kwargs)._set_attrs(attrs)

	def sample_context(self, k):
		for c in self.context:
			try:
				return getattr(getattr(self, c), k)
			except:
				pass
		return getattr(self, k)

	def scale_factor(self, attr):
		scaling = self.scaling.get(attr, {})
		f = [self.sample_context(k) ** v for k, v in scaling.items()]
		return np.prod(f)

	def _set_attrs(self, attrs):
		# FIXME: should only call this from init to retain immutable arch
		for k, v in attrs.items():
			self.set_attr(k, v)
		return self

	def get_attrs(self):
		return {k: self.get_attr(k) for k, v in self.attrs.items()}

	def set_attr(self, attr, value):
		self.attrs[attr] = value / self.scale_factor(attr)

	def get_attr(self, attr):
		return self.attrs[attr] * self.scale_factor(attr)

	# add dict interface
	def keys(self):
		return self.attrs.keys()
	def values(self):
		return self.get_attrs().values()
	def items(self):
		return self.get_attrs().items()
	def __getitem__(self, item):
		return self.get_attr(item)
	# and attr-dict interface
	def __getattr__(self, attr):
		if attr.startswith('__') and attr.endswith('__'):
			raise AttributeError	# need this check to make pickle work
		return self.get_attr(attr)


import pickle	# fixme: can we drop pickle for json here? perhaps some small changes to Base dataclass?
import codecs

def pickle_decode(obj):
	return pickle.loads(codecs.decode(obj.encode(), "base64"))
def pickle_encode(obj):
	return codecs.encode(pickle.dumps(obj), "base64").decode()
