from numpy import *

class System(object):
	def __init__(self, model, x_init, t_init):
		self.model = model
		self.x = mat(x_init)
		self.t = t_init
		self.__mean = [0]*model.Q.shape[0]

	def propogate(self, t, u):
		T = t - self.t
		v = mat(random.multivariate_normal(self.__mean,
		                                   self.model.Q)).T
		self.x = self.model.F(T)*self.x + self.model.B(T)*u + self.model.G(T)*v
		self.t = t

	def copy(self):
		return type(self)(self.model, self.x, self.t)
