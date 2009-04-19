from linalg import Matrix, Vector

import system

import math
import operator
import random

class Filter(object):
	def __init__(self):
		pass

	def update(self, t, measurement):
		raise NotImplementedError('Filter.update is a virtual function.')

	def extrap(self, t):
		raise NotImplementedError('Filter.extrap is a virtual function.')

class KalmanFilter(Filter):
	def __init__(self, model, H, x_init, P_init, t_init):
		Filter.__init__(self)
		self.model = model
		self.H = H
		self.x = x_init
		self.P = P_init
		self.t = t_init

	def update(self, t, (z, R), u):
		self.extrap(t, u)
		y = z - self.H * self.x
		S = self.H * self.P * self.H.transpose() + R
		K = self.P * self.H.transpose() * S.inverse()

		self.x = self.x + K * y
		self.P = (Matrix.identity(self.x.dim()) - K*self.H)*self.P

	def extrap(self, t, u):
		F = self.model.F(t - self.t)
		B = self.model.B(t - self.t)
		G = self.model.G(t - self.t)
		self.x = F*self.x + B*u
		self.P = F*self.P*F.transpose() + G*self.model.Q*G.transpose()
		self.t = t

#unfinished
class ExtendedKalmanFilter(Filter):
	def __init__(self, model, H, x_init, P_init, t_init):
		Filter.__init__(self)
		self.model = model
		self.H = H
		self.x = x_init
		self.P = P_init
		self.t = t_init

	def update(self, t, (z, R)):
		(x_ext, P_ext) = self.extrap(t)
		y = z - self.H * x_ext
		S = self.H * P_ext * self.H.transpose() + R
		K = P_ext * self.H.transpose() * S.inverse()

		self.x = x_ext + K * y
		self.P = (Matrix.identity(self.x.dim()) - K*self.H)*P_ext
		self.t = t

	def extrap(self, t):
		F = self.model.F(self.t, t)
		Q = self.model.Q(self.t, t)
		x = self.model.F(self.t, self.x, t)
		P = F*self.P*F.transpose() + Q

class ParticleFilter(Filter):
	particle_count = 100

	def __init__(self, model, H, x_init, P_init, t_init):
		Filter.__init__(self)
		self.model = model
		self.H = H
		self.t = t_init

		self.particles = [system.System(self.model, Vector.random(x_init, P_init), self.t)
		                  for i in xrange(self.particle_count)]
		self.weights = [1.0/self.particle_count]*self.particle_count
		self.update_estirixmate()

	def update(self, t, (z, R), u):
		self.extrap(t, u)
		Rinv = R.inverse()
		self.weights = [(lambda diff: w*math.exp(int((diff.transpose()*Rinv*diff).m)*-0.5))(z - self.H*p.x)
		                for (p,w) in zip(self.particles, self.weights)]
		weight_sum = sum(self.weights)
		#print weight_sum
		self.weights = map(lambda w: w/weight_sum, self.weights)
		self.update_estimate()
		#print self.x[0:2]
		self.resample()
		#self.update_estimate()
		#print self.x[0:2]

	def extrap(self, t, u):
		self.extrap_particles(t, u)
		self.update_estimate()

	def extrap_particles(self, t, u):
		map(lambda p: p.propogate(t, u), self.particles)
		self.t = t

	def update_estimate(self):
		self.x = reduce(operator.add,
		                [p.x*w for (p,w) in zip(self.particles, self.weights)])
		self.P = reduce(operator.add,
		                [(p.x-self.x)*(p.x-self.x).transpose()*w
		                 for (p,w) in zip(self.particles, self.weights)])

	def resample(self):
		cumsum = []
		accum = 0
		for w in self.weights:
			accum += w
			cumsum.append(accum)

		new_particles = []
		for i in xrange(self.particle_count):
			r = random.random()
			for j in xrange(len(cumsum)):
				if r < cumsum[j]:
					break
			new_particles.append(self.particles[j].copy())
		self.weights = [1.0/self.particle_count]*self.particle_count
		self.particles = new_particles

class NewFilter(Filter):
	def __init__(self, model, H, x_init, P_init, t_init):
		Filter.__init__(self)
		self.model = model
		self.H = H
		self.x = x_init
		self.t = t_init

		self.M = Matrix([[500.0, 0.0, 0.0, 0.0],
		                 [0.0, 500.0, 0.0, 0.0],
		                 [0.0, 0.0, 500.0, 0.0],
		                 [0.0, 0.0, 0.0, 500.0]])
		#self.M = P_init
		self.D = Matrix([[0.0, 0.0],
		                 [0.0, 0.0],
		                 [0.0, 0.0],
		                 [0.0, 0.0]])
		self.Lambda = Matrix([[100.0, 0.0],
		                      [0.0, 100.0]])
		self.H = Matrix([[1.0, 0.0, 0.0, 0.0],
		                 [0.0, 0.0, 1.0, 0.0]])

	def update(self, t, (z, N), u):
		del u
		self.extrap(t, None)
		S = self.M + self.D*self.Lambda*self.D.transpose()
		Q = self.H * S * self.H.transpose() + N
		K = S * self.H.transpose() * Q.inverse()
		L = Matrix.identity(4) - K*self.H

		self.M = L*self.M*L.transpose() + K*N*K.transpose()
		self.D = L*self.D
		self.x = self.x + K * (z - self.H*self.x)

	def extrap(self, t, u):
		del u

		dT = t - self.t
		F = Matrix([[1.0,  dT, 0.0, 0.0],
		            [0.0, 1.0, 0.0, 0.0],
		            [0.0, 0.0, 1.0,  dT],
		            [0.0, 0.0, 0.0, 1.0]])

		a = (dT**2)/2
		G = Matrix([[  a, 0.0],
		            [ dT, 0.0],
		            [0.0,   a],
		            [0.0,  dT]])

		self.M = F*self.M*F.transpose()
		self.D = F*self.D+G
		self.x = F*self.x
		self.t = t
