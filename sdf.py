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

	random.seed(1)

	T_start = 0.0
	T_stop = 20.0
	T_step = 1.0

	#############################################################################
	# System (CA/PWCJ)
	#############################################################################
	d = 1   #dimension
	n = 3   #derivatives
	truth_q = 0.1 #process noise variance

	#input control implementation
	B = model.PWCA
	u_1 = mat([0.0]).T
	u_2 = mat([-9.8]).T #gravity
	u_transition = 10.0
	u = u_1

	# measurement variance
	R = mat(10.0)

	# initial truth
	truth_x_init = mat([0.0, 100.0, 1.0]).T

	# Objects
	truth_model = model.CA_PWCJ_Model(d, truth_q, B)
	truth_observer = observer.CP_Observer(d, n, R)
	s = system.System(truth_model, truth_observer, truth_x_init, 0)

	#Generate an initial measurement for filter initialization
	msmt_init = s.observe()

	#############################################################################
	#Filters
	#############################################################################

	imm_filters = []

	#CV
	cv_q = 0.1
	cv_model = model.CV_PWCA_Model(d, cv_q)
	cv_x_init = mat([msmt_init[0,0], truth_x_init[1,0]]).T
	cv_P_init = mat([[R[0,0], 0.0],
	                 [   0.0, 0.0]])

	cv_filter = filter.KalmanFilter(cv_model, truth_observer, cv_x_init, cv_P_init, T_start)
	imm_filters.append(filter.KalmanFilter(cv_model, truth_observer, cv_x_init, cv_P_init, T_start))

	#CA
	ca_q = 0.1
	ca_model = model.CA_PWCJ_Model(d, ca_q)
	ca_x_init = mat([msmt_init[0,0], truth_x_init[1,0], truth_x_init[2,0]]).T
	ca_P_init = mat([[R[0,0],  0.0,  0.0],
	                 [   0.0, 10.0,  0.0],
	                 [   0.0,  0.0, 10.0]])

	ca_filter = filter.KalmanFilter(ca_model, truth_observer, ca_x_init, ca_P_init, T_start)
	imm_filters.append(filter.KalmanFilter(ca_model, truth_observer, ca_x_init, ca_P_init, T_start))

	#IMM
	imm_pie = mat([[0.9, 0.1],
	               [0.1, 0.9]])
	imm_mu_init = mat([0.5, 0.5]).T
	imm_filter = filter.IMM(imm_filters, imm_pie, imm_mu_init)

	#############################################################################
	# Record keeping
	#############################################################################
	T = mat(list(frange(T_start, T_stop, T_step)))
	truth = truth_x_init
	msmt = msmt_init
	cv_hist = cv_filter.x
	ca_hist = ca_filter.x
	imm_hist = imm_filter.x
	mu_hist = imm_filter.mu

	#############################################################################
	#Simulation
	#############################################################################
	for t in frange(T_start + T_step, T_stop, T_step):
		#Switch modes
		if abs(t-u_transition) < 1e-6: u = u_2

		#Propagate plant
		s.propagate(t,u)
		truth = bmat([truth, s.x])

		#Measure
		msmt = bmat([msmt, s.observe()])

		#Track Update
		#import pdb; pdb.set_trace()
		cv_filter.update(t, msmt[:,-1], R)
		cv_hist = bmat([cv_hist, cv_filter.x])

		ca_filter.update(t, msmt[:,-1], R)
		ca_hist = bmat([ca_hist, ca_filter.x])

		imm_filter.update(t, msmt[:,-1], R)
		imm_hist = bmat([imm_hist, imm_filter.x])
		mu_hist = bmat([mu_hist, imm_filter.mu])

	print truth.T
	#print cv_hist.T

	plt.subplot(211)
	plt.plot(T.T, truth[0,:].T, 'b-')
	plt.plot(T.T, cv_hist[0,:].T, 'r+-')
	plt.plot(T.T, ca_hist[0,:].T, 'g+-')
	plt.plot(T.T, imm_hist[0,:].T, 'm+-')
	plt.plot(T.T, msmt[0,:].T, 'kx',)
	plt.subplot(212)
	plt.plot(T.T, mu_hist[0,:].T, 'r+-')
	plt.plot(T.T, mu_hist[1,:].T, 'g+-')
	plt.show()

if __name__ == '__main__':
	main()
