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

	#TODO: optimize?
	def project_indices(self, D, N):
		x = []
		y = []
		min_N = min(self.N, N)
		for d in range(min(self.D, D)):
			x.extend(range(self.N*d,self.N*d+min_N))
			y.extend(range(     N*d,     N*d+min_N))
		return x, y

	# Projects this filter's state to different D,N
	def project_state(self, x, D, N):
		xdx, ydx = self.project_indices(D, N)
		y = mat(zeros((D*N, 1)))
		y[ydx] = x[xdx]
		return y

	def project_cov(self, Px, D, N):
		xdx, ydx = self.project_indices(D, N)
		Py = mat(zeros((D*N, D*N)))
		Py[ix_(ydx,ydx)] = Px[ix_(xdx,xdx)]
		return Py

class CV_PWCA_Model(LinearModel):
	def __init__(self, D, q, B=None):
		LinearModel.__init__(self, D, 2, CV, B, PWCA, q*mat(eye(D)))