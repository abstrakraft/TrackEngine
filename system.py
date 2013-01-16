from numpy import *

class System(object):
	def __init__(self, plant, observer, x_init, t_init):
		self.plant = plant
		self.observer = observer
		self.x = mat(x_init)
		self.t = t_init

	def propagate(self, t, u=None):
		dt = t - self.t
		self.x = self.plant.propagate(self.x, dt, u)
		self.t = t

	def observe(self):
		return self.observer.observe(self.x)
