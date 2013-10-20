from numpy import *
from scipy.linalg import block_diag

def CP_block(N):
	return mat([1] + [0]*(N-1))

def CP(D, N):
	return block_diag(*[CP_block(N)]*D)

class LinearObserver(object):
	def __init__(self, H, R):
		self.H = H
		self.R = R
		self.__mean = [0]*self.R.shape[0]

	def observe(self, x):
		return self.H*x + mat(random.multivariate_normal(self.__mean, self.R)).T

class CP_Observer(LinearObserver):
	def __init__(self, D, N, R):
		LinearObserver.__init__(self, CP(D, N), R)
		self.D = D
		self.N = N

	def get_H(self, D, N):
		'''Return H of the correct size for a filter described by D,N'''
		if self.D > D:
			raise ValueError('not enough filter dimensions to produce H matrix')
		H = CP(self.D, N)
		if D > self.D:
			H = bmat([H, mat(zeros((self.D, (D-self.D)*N)))])
		return H
