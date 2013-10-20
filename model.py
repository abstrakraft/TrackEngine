from numpy import *
from scipy.linalg import block_diag

import transform

# Propogation Models
def CV_block(dt):
	return mat([[1.0, dt],
	            [0.0, 1.0]])

def CV(dt, D):
	return block_diag(*[CV_block(dt)]*D)

def CA_block(dt):
	return mat([[1.0,  dt, dt**2/2],
	            [0.0, 1.0,      dt],
	            [0.0, 0.0,     1.0]])

def CA(dt, D):
	return block_diag(*[CA_block(dt)]*D)

# Noise/Control models
def PWCV_block(dt):
	return mat([dt, 1]).T
def PWCA_block(dt):
	return mat([dt**2/2, dt, 1]).T
def PWCJ_block(dt):
	return mat([dt**3/6, dt**2/2, dt, 1]).T

def block2mat(block, D, N):
	b = mat(zeros((N,1)))
	nz_terms = min(len(block), N)
	b[0:nz_terms] = block[0:nz_terms]
	return block_diag(*[b]*D)

def PWCV(dt, D, N):
	return block2mat(PWCV_block(dt), D, N)

def PWCA(dt, D, N):
	return block2mat(PWCA_block(dt), D, N)

def PWCJ(dt, D, N):
	return block2mat(PWCJ_block(dt), D, N)


#D is the number of dimensions, N is the number of derivatives
class LinearModel(object):
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
			return None
		else:
			return self.B_func(dt, self.D, self.N)

	def G(self, dt):
		return self.G_func(dt, self.D, self.N)

	def propagate(self, x, dt, u=None):
		v = mat(random.multivariate_normal(self.__mean,
		                                   self.Q)).T
		ret_x = self.F(dt)*x + self.G(dt)*v
		if u is not None:
			ret_x += self.B(dt)*u
		return ret_x

	# Project a state from this model to another
	def project_state(self, x, D, N):
		return transform.project_state(x, self.D, self.N, D, N)

	def project_cov(self, Px, D, N):
		return transform.project_cov(Px, self.D, self.N, D, N)

class CV_PWCA_Model(LinearModel):
	def __init__(self, D, q, B=None):
		LinearModel.__init__(self, D, 2, CV, B, PWCA, q*mat(eye(D)))

class CA_PWCJ_Model(LinearModel):
	def __init__(self, D, q, B=None):
		LinearModel.__init__(self, D, 3, CA, B, PWCJ, q*mat(eye(D)))
