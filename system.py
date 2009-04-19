from linalg import Vector
import numpy

class System(object):
	def __init__(self, model, x_init, t_init):
		self.model = model
		self.x = x_init
		self.t = t_init
		self.__mean = numpy.array([0]*model.Q.size[0])

	def propogate(self, t, u):
		T = t - self.t
		v = Vector(numpy.random.multivariate_normal(self.__mean,
		                                            self.model.Q.m))
		self.x = self.model.F(T)*self.x + self.model.B(T)*u + self.model.G(T)*v
		self.t = t
	
	def copy(self):
		return type(self)(self.model, self.x, self.t)
