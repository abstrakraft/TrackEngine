import numpy
import math

class Matrix(object):
	__slots__ = ['m', 'size']

	def __init__(self, x):
		if isinstance(x, numpy.ndarray):
			self.m = x
		else:
			self.m = numpy.array(x)
		self.size = self.m.shape

	def __neg__(self):
		return self*-1

	def __add__(self, y):
		return self.__class__(self.m + getattr(y, 'm', y))

	def __radd__(self, x):
		return self.__class__(getattr(x, 'm', x) + self.m)

	def __sub__(self, y):
		return self.__add__(y.__neg__())

	def __rsub__(self, x):
		return self.__neg__().__radd__(x)

	def __mul__(self, y):
		val = numpy.dot(self.m, getattr(y, 'm', y))

		if isinstance(y, Vector):
			return y.__class__(val)
		else:
			return self.__class__(val)

	def __rmul__(self, x):
		#x must be a scalar?
		val = numpy.dot(getattr(x, 'm', x), self.m)
		return self.__class__(val)

	def __div__(self, y):
		if isinstance(y, Matrix):
			raise TypeError('Cannot divide matrices')
		else:
			return self.__class__(self.m/y)

	def __getitem__(self, index):
		x = self.m.__getitem__(index)
		if isinstance(x, numpy.ndarray):
			return self.__class__(x)
		else:
			return x

	def __setitem__(self, index, value):
		if isinstance(value, Matrix):
			value = value.m
		return self.m.__setitem__(index, value)

	def __repr__(self):
		header = self.__class__.__name__ + '(['
		header_len = len(header)
		return header + (',\n' + ' '*header_len).join( \
		  [str(self.m[i].tolist()) for i in range(self.size[0])]) + '])'

	def __str__(self):
		return str(self.m)

	def eigen(self):
		(eigval, eigvec) = numpy.linalg.eig(self.m)
		eigval = eigval.tolist()
		eigvec = [Vector(eigvec[:,i]) for i in range(eigvec.shape[1])]
		return (eigval, eigvec)

	def inverse(self):
		return self.__class__(numpy.linalg.inv(self.m))

	def norm(self):
		return numpy.linalg.norm(self.m)

	def transpose(self):
		return self.__class__(self.m.T)

	def tolist(self):
		return self.m.tolist()

	@classmethod
	def rand(cls, r, c):
		return cls(numpy.random.rand(r, c))

	@classmethod
	def randn(cls, r, c):
		return cls(numpy.random.randn(r, c))

	@classmethod
	def identity(cls, dim):
		return cls(numpy.eye(dim))

	@classmethod
	def zero(cls, dim):
		return cls(numpy.zeros((dim, dim)))

class Vector(Matrix):
	def __init__(self, x):
		Matrix.__init__(self, x)

	def __len__(self):
		return self.m.size

	def __repr__(self):
		return self.__class__.__name__ + '([' + \
		       ', '.join([str(el) for el in self.m]) + '])'

	def __str__(self):
		return '[' + '\n '.join([str(el) for el in self.m]) + ']'

	dim = __len__

	def dot(x, y):
		if not (isinstance(x, Vector) and isinstance(y, Vector)):
			raise TypeError('Vector expected')
		return float(numpy.dot(x.m, y.m))

	def cross(x, y):
		if not (isinstance(x, Vector) and isinstance(y, Vector)):
			raise TypeError('Vector expected')
		return Vector(numpy.cross(x.m, y.m))

	@classmethod
	def random(cls, mean, cov):
		return cls(numpy.random.multivariate_normal(mean.m, cov.m))

	@classmethod
	def zero(cls, dim):
		return cls(numpy.zeros(dim))

def sgn(x):
	if x >= 0:
		return type(x)(1)
	else:
		return type(x)(-1)

class Quaternion(Vector):
	def __init__(self, a, b = None):
		if b is None:
			#a is the full quaternion vector
			if len(a) != 4:
				raise TypeError('quaternions must have length 4')
			Vector.__init__(self, a)
		else:
			#a is the angle (radians)
			#b is the axis vector
			if len(b) != 3:
				raise TypeError('rotation axis must have length 3')
			if not isinstance(b, Vector):
				b = Vector(b)
			w = math.cos(a/2)*sgn(a)
			if b.norm() == 0:
				Vector.__init__(self, [w, 0.0, 0.0, 0.0])
			else:
				Vector.__init__(self, [w] + (b/b.norm()*math.sqrt((1-w**2))).tolist())

	def __mul__(x, y):
		if isinstance(y, Vector):
			if len(y) == 3:
				y = quaternion([0.0] + y.tolist())

			s = x[0]; v = x[1:]
			t = y[0]; w = y[1:]

			a = s*t - Vector.dot(v, w)
			b = s*w + t*v + Vector.cross(v,w)

			return x.__class__([a] + b.tolist())
		else:
			return Vector.__mul__(x, y)

	def __rmul__(y, x):
		if isinstance(x, Vector):
			if len(x) == 3:
				x = quaternion([0.0] + x.tolist())

			s = x[0]; v = x[1:]
			t = y[0]; w = y[1:]

			a = s*t - Vector.dot(v, w)
			b = s*w + t*v + Vector.cross(v,w)

			return y.__class__([a] + b.tolist())
		else:
			return Vector.__rmul__(y, x)

	def __getitem__(self, index):
		return Vector(self.m).__getitem__(index)

	def inverse(self):
		x = -self
		x[0] = -x[0]
		#x = x/x.norm()
		return x

	def transform(self, y):
		return (self*y*self.inverse())[1:]
		#return self.matrix() * y

	def matrix(self):
		w = self[0]
		x = self[1]
		y = self[2]
		z = self[3]
		return Matrix([[w**2 + x**2 - y**2 - z**2, 2*x*y - 2*w*z,             2*x*z + 2*w*y],
		               [2*x*y + 2*w*z,             w**2 - x**2 + y**2 - z**2, 2*y*z - 2*w*x],
		               [2*x*z - 2*w*y,             2*y*z + 2*w*x,             w**2 - x**2 - y**2 + z**2]])
	
	def angles(self):
		#Angle Definition:
		#theta -> yaw, about y axis
		#phi -> pitch, about x axis
		#psi -> roll, about z axis
		#all right-handed rotations, q = R_theta * R_phi * R_psi
		w = self[0]
		x = self[1]
		y = self[2]
		z = self[3]
		theta = math.atan2(2*w*y + 2*x*z, w**2 - x**2 - y**2 + z**2)
		phi = math.asin(2*w*x - 2*y*z)
		psi = math.atan2(2*w*z + 2*x*y, w**2 - x**2 + y**2 - z**2)
		return (theta, phi, psi)

	def angles2(self):
		#Angle Definition:
		#psi -> about z axis
		#phi-> about y axis
		#theta -> about x axis
		#all right-handed rotations, q = R_psi * R_phi * R_theta
		w, x, y, z = self
		psi = math.atan2(2*(x*y + w*z), 1-2*(y**2 + z**2))
		phi = math.asin(2*(w*y-x*z))
		theta = math.atan2(2*(w*x + y*z), 1-2*(x**2 + y**2))
		return (psi, phi, theta)

