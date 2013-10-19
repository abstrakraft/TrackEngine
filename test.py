from numpy import *
import filter
import model
import observer
import system
import math
import matplotlib.pyplot as plt

def main():
	res = 10

	d = 2
	n = 2
	q = 4
	B = model.PWCA
	u = mat([0.0, -9.8]).T
	R = mat([[1000.0, 0.0],[0.0, 1000.0]])

	x_init = mat([0.0, 50.0, 0.0, 100.0]).T
	P_init = mat([[0.2, 0.0, 0.0, 0.0],
	              [0.0, 1.0, 0.0, 0.0],
	              [0.0, 0.0, 0.2, 0.0],
	              [0.0, 0.0, 0.0, 1.0]])

	km = model.CV_PWCA_Model(d, q, B)
	ko = observer.CP_Observer(d, n, R)
	s = system.System(km, ko, x_init, 0)

	truth = x_init
	msmt = s.observe()
	kf = filter.KalmanFilter(km, ko, matrix([msmt[0,0], 50.0, msmt[1,0], 100.0]).T, P_init, 0)

	track1 = kf.x

	for j in range(1,res):
		t = float(j)/res
		s.propagate(t, u)
		truth = bmat([truth, s.x])

	for i in range(1,25):
		if i==13: u = u*-1
		s.propagate(i,u)
		truth = bmat([truth, s.x])
		msmt = bmat([msmt, s.observe()])
		kf.update(i, msmt[:,-1], R, u)
		track1 = bmat([track1, kf.x])

		for j in range(1, res):
			t = i + float(j)/res
			s.propagate(t, u)
			truth = bmat([truth, s.x])
			#kf.extrap(t, u)
			#track1 = bmat([track1, kf.x])

	plt.plot(truth[0,:].T, truth[2,:].T, 'b-')
	plt.plot(track1[0,:].T, track1[2,:].T, 'r+-')
	plt.plot(msmt[0,:].T, msmt[1,:].T, 'kx',)
	plt.show()

if __name__ == '__main__':
	main()
