from numpy import *
from math import *

import system

import operator

class Filter(object):
	def __init__(self):
		pass

	def update(self, t, measurement):
		raise NotImplementedError('Filter.update is a virtual function.')

	def extrap(self, t):
		raise NotImplementedError('Filter.extrap is a virtual function.')

class KalmanFilter(Filter):
	def __init__(self, model, observer, x_init, P_init, t_init):
		Filter.__init__(self)
		self.model = model
		self.observer = observer
		self.x = mat(x_init)
		self.P = mat(P_init)
		self.t = t_init

	def update(self, t, z, R, u=None):
		self.extrap(t, u)

		(x, P) = (self.x, self.P)

		H = self.observer.get_H(self.model.D, self.model.N)

		y = z - H*x
		S = H*P*H.T + R
		K = P*H.T*S.I #got "ValueError: array must not contain infs or NaNs" once

		self.x = x + K*y
		self.P = (eye(len(x)) - K*H)*P

		return {'y':y, 'K':K, 'S':S}

	def extrap(self, t, u=None):
		F = self.model.F(t - self.t)
		B = self.model.B(t - self.t)
		G = self.model.G(t - self.t)
		self.x = F*self.x
		if u is not None:
			self.x += B*u
		self.P = F*self.P*F.T + G*self.model.Q*G.T
		self.t = t

class IMM(Filter):
	def __init__(self, filters, pie, mu_init):
		Filter.__init__(self)

		self.R = len(filters)

		#list of length R
		self.filters = filters

		#Rx1 vector of (initial) mode probabilities
		self.mu = mu_init

		#RxR switching probability matrix, usually denoted with a capital pi
		#called pie here to avoid confusion with 3.14...
		#pie[i,j] = P(M(k) = M_j | M(k-1) = M_i)
		self.pie = pie

		#Output dimensions,derivatives of the IMM (min of each over all filters)
		self.D = min([f.model.D for f in filters])
		self.N = min([f.model.N for f in filters])

		#Mix initial estimates
		self.x, self.P = self.mix(self.mu, self.D, self.N)

	def update(self, t, z, R, u=None):
		#Compute mixing probabilities
		mu_i_j = multiply(self.pie, tile(self.mu, (1, self.R)))
		c_j = sum(mu_i_j, 0)
		mu_i_j = mu_i_j / tile(c_j, (self.R, 1))

		#Mix
		#For each filter f_j, sum over filters f_i
		for jdx, f_j in enumerate(self.filters):
			# TODO: unbiased mixing
			f_j.x, f_j.P = self.mix(mu_i_j[:,jdx], f_j.model.D, f_j.model.N)

		#Filter
		update_params = []
		for f in self.filters:
			update_params.append(f.update(t, z, R, u))

		#Mode Probability Update
		L = mat(zeros(self.mu.shape))
		for fdx in range(self.R):
			y,S = update_params[fdx]['y'], update_params[fdx]['S']
			L[fdx] = 1/sqrt(linalg.det(2*pi*S))*exp(-0.5*y.T*S.I*y)

		self.mu = multiply(L,c_j.T)
		self.mu = self.mu/sum(self.mu)

		#Update output
		self.x, self.P = self.mix(self.mu, self.D, self.N)

	def extrap(self, t, u=None):
		for f in self.filters:
			f.extrap(t, u)
		self.x, self.P = self.mix(self.mu, self.D, self.N)

	def mix(self, mu, D, N):
		proj_x = bmat([f.model.project_state(f.x, D, N) for f in self.filters])
		x = bmat(proj_x)*self.mu

		P = mat(zeros((D*N, D*N)))
		for fdx, f in enumerate(self.filters):
			P += mu[fdx,0]*(
					f.model.project_cov(f.P, D, N) +
					(proj_x[:,fdx] - x)*(proj_x[:,fdx] - x).T)

		return x, P

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

	def __init__(self, model, observer, x_init, P_init, t_init):
		Filter.__init__(self)
		self.model = model
		self.observer = observer
		self.x = mat(x_init)
		self.P = mat(P_init)
		self.t = t_init

		#TODO: Really? A separate system for every particle?  Fix this.
		self.particles = mat(random.multivariate_normal(array(x_init).squeeze(), P_init, particle_count)).T
		#self.particles = [system.System(self.model, random.multivariate_normal(x_init, P_init), self.t)
		#                  for i in xrange(self.particle_count)]
		self.weights = mat([1.0/self.particle_count]*self.particle_count)
		self.update_estimate()

	def update(self, t, (z, R), u):
		self.extrap(t, u)
		Rinv = linalg.inv(R)
		self.weights = [(lambda diff: w*exp(-0.5*diff*Rinv*diff))(z - self.H*p.x)
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
		                [(p.x-self.x)*(p.x-self.x).T*w
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
