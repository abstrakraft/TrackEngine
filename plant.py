from numpy import *
from scipy.linalg import block_diag

def CV_block(dt):
	return mat([[1.0, dt],
	            [0.0, 1.0]])

def CA_block(dt):
	return mat([[1.0,  dt, dt**2/2],
	            [0.0, 1.0,      dt],
	            [0.0, 0.0,     1.0]])

def PWCV_block(dt):
	return mat([dt, 1, 0]).T

def PWCA_block(dt):
	return mat([dt**2/2, dt]).T


def CV(dt, D):
	return block_diag(*[CV_block(dt)]*D)

def CA(dt, D):
	return block_diag(*[CA_block(dt)]*D)

def PWCV(dt, D):
	block_diag(*[PWCV_block(dt)]*D)

def PWCA(dt, D):
	return block_diag(*[PWCA_block(dt)]*D)

#D is the number of dimensions, N is the number of derivatives
class LinearPlant(object):
	def __init__(self, D, N, F, B, G, Q=None):
		self.D = D
		self.N = N
		self.F_func = F
		self.B_func = B
		self.G_func = G
		if Q is not None:
			self.Q = Q
		else:
			self.Q = mat(eye(D*N))
		self.__mean = [0]*self.Q.shape[0]

	def F(self, dt):
		return self.F_func(dt, self.D)

	def B(self, dt):
		if self.B_func is None:
			return self.B_func
		else:
			return self.B_func(dt, self.D)

	def G(self, dt):
		return self.G_func(dt, self.D)

	def propagate(self, x, dt, u=None):
		v = mat(random.multivariate_normal(self.__mean,
		                                   self.Q)).T
		ret_x = self.F(dt)*x + self.G(dt)*v
		if u is not None:
			ret_x += self.B(dt)*u
		return ret_x

class CV_PWCA_Plant(LinearPlant):
	def __init__(self, D, q, B=None):
		LinearPlant.__init__(self, D, 2, CV, B, PWCA, q*mat(eye(D)))
