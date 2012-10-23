from numpy import *

class Model(object):
	def F(self, dt):
		raise NotImplementedError('Model.F is a virtual function.')

	def B(self, dt):
		raise NotImplementedError('Model.B is a virtual function.')

	def G(self, dt):
		raise NotImplementedError('Model.G is a virtual function.')

class Inertial2DModel(Model):
	def __init__(self, Q):
		Model.__init__(self)
		self.Q = Q

	def F(self, dt):
		return mat([[1.0,  dt, 0.0, 0.0],
		            [0.0, 1.0, 0.0, 0.0],
		            [0.0, 0.0, 1.0,  dt],
		            [0.0, 0.0, 0.0, 1.0]])

	def B(self, dt):
		return mat([[dt**2/2,       0],
		            [     dt,       0],
		            [      0, dt**2/2],
		            [      0,      dt]])

	def G(self, dt):
		return mat([[dt**2/2,       0],
		            [     dt,       0],
		            [      0, dt**2/2],
		            [      0,      dt]])

class Grav2DModel(Model):
	def __init__(self, Q):
		Model.__init__(self)
		self.Q = Q

	def F(self, dt):
		return mat([[1.0,  dt, 0.0, 0.0],
		            [0.0, 1.0, 0.0, 0.0],
		            [0.0, 0.0, 1.0,  dt],
		            [0.0, 0.0, 0.0, 1.0]])

	def B(self, T):
		return mat([[dt**2/2,       0],
		            [     dt,       0],
		            [      0, dt**2/2],
		            [      0,      dt]])

	def G(self, T):
		return mat([[dt**2/2,       0],
		            [     dt,       0],
		            [      0, dt**2/2],
		            [      0,      dt]])
