from numpy import *
import filter
import model
import observer
import system
from math import *
import matplotlib.pyplot as plt

def frange(start, stop, step):
	x = start
	while x < stop:
		yield x
		x += step

def main():
	T_start = 0.0
	T_stop = 20.0
	T_step = 1.0

	#############################################################################
	# System (CA/PWCJ)
	#############################################################################
	d = 1   #dimension
	n = 3   #derivatives
	q = 1.0 #process noise variance

	#input control implementation
	B = model.PWCA
	u_1 = mat([0.0]).T
	u_2 = mat([-9.8]).T #gravity
	u_transition = 10.0
	u = u_1

	# measurement variance
	R = mat(10.0)

	# initial truth
	x_init = mat([0.0, 100.0, 1.0]).T

	# Objects
	truth_model = model.CA_PWCJ_Model(d, q, B)
	truth_observer = observer.CP_Observer(d, n, R)
	s = system.System(truth_model, truth_observer, x_init, 0)

	#Generate an initial measurement for filter initialization
	msmt_init = s.observe()

	#############################################################################
	#Filters
	#############################################################################
	# initial state estimate (CA)
	kf_x_init = mat([msmt_init[0,0], x_init[1,0], x_init[2,0]]).T

	# initial covariance estimate (CA)
	P_init = mat([[R[0,0],  0.0,  0.0],
	              [ 0.0, 10.0,  0.0],
								[ 0.0,  0.0, 10.0]])
	kf = filter.KalmanFilter(truth_model, truth_observer, kf_x_init, P_init, T_start)

	#############################################################################
	# Record keeping
	#############################################################################
	T = mat(list(frange(T_start, T_stop, T_step)))
	truth = x_init
	msmt = msmt_init
	track = kf.x

	#############################################################################
	#Simulation
	#############################################################################
	for t in frange(T_start + T_step, T_stop, T_step):
		if abs(t-u_transition) < 1e-6: u = u_2
		s.propagate(t,u)
		truth = bmat([truth, s.x])
		msmt = bmat([msmt, s.observe()])
		kf.update(t, msmt[:,-1], R)
		track = bmat([track, kf.x])
		#mode = bmat([mode

	print truth.T
	print track.T

	plt.plot(T.T, truth[0,:].T, 'b-')
	plt.plot(T.T, track[0,:].T, 'r+-')
	plt.plot(T.T, msmt[0,:].T, 'kx',)
	plt.show()

if __name__ == '__main__':
	main()
