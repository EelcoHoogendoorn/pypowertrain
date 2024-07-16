from pypowertrain.utils import *


def test_replace():
	@dataclass
	class Bar(Base):
		a: int=1
	@dataclass
	class Foo(Base):
		b: Bar
		a:int=0
	f = Foo(b=Bar(a=2))
	print()
	print(list(expand_paths('.a', f)))
	r = (f.replace(__a=3))
	print()
	print(f.a)
	print(r.b.a)
	print(r.a)
	r = (f.replace(a=3))
	print(r.a)
