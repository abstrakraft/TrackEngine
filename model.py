from linalg import Matrix

class Model(object):
	def __init__(self):
		pass

	def F(self, T):
		raise NotImplementedError('Model.F is a virtual function.')

	def B(self, T):
		raise NotImplementedError('Model.B is a virtual function.')

	def G(self, T):
		raise NotImplementedError('Model.G is a virtual function.')

class Inertial2DModel(Model):
	def __init__(self, Q):
		Model.__init__(self)
		self.Q = Q

	def F(self, T):
		return Matrix([[1.0,   T, 0.0, 0.0],
		               [0.0, 1.0, 0.0, 0.0],
		               [0.0, 0.0, 1.0,   T],
		               [0.0, 0.0, 0.0, 1.0]])

	def B(self, T):
		return Matrix([[T**2/2,      0],
		               [     T,      0],
		               [     0, T**2/2],
		               [     0,      T]])

	def G(self, T):
		return Matrix([[T**2/2,      0],
		               [     T,      0],
		               [     0, T**2/2],
		               [     0,      T]])
