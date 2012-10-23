from numpy import *
import filter
import model
import system
import math
import matplotlib.pyplot as plt

def main():
	Q = matrix([[1.0, 1.0],[1.0, 2.0]])
	H = matrix([[1.0, 0.0, 0.0, 0.0],
	            [0.0, 0.0, 1.0, 0.0]])
	R = matrix([[10.0, 0.0],[0.0, 10.0]])
	u = matrix([0, -9.8]).T

	x_init = matrix([0.0, 50.0, 0.0, 100.0]).T
	P_init = matrix([[0.2, 0.0, 0.0, 0.0],
	                 [0.0, 1.0, 0.0, 0.0],
	                 [0.0, 0.0, 0.2, 0.0],
	                 [0.0, 0.0, 0.0, 1.0]])
	m = model.Inertial2DModel(Q)
	s = system.System(m, x_init, 0)

	truth = [x_init]
	msmnt = [H*x_init + matrix([random.normal(0, math.sqrt(R[0,0])), random.normal(0, math.sqrt(R[1,1]))]).T]
	kf = filter.KalmanFilter(m, H, matrix([msmnt[0][0], 50.0, msmnt[0][1], 100.0]).T, P_init, 0)
	pf = filter.KalmanFilter(m, H, matrix([msmnt[0][0], 50.0, msmnt[0][1], 100.0]).T, P_init, 0)
	#pf = filter.ParticleFilter(m, H, Vector([msmnt[0][0], 50.0, msmnt[0][1], 100.0]), P_init, 0)
	#pf = filter.NewFilter(m, H, Vector([msmnt[0][0], 50.0, msmnt[0][1], 100.0]), P_init, 0)

	track1 = [(kf.x, kf.P)]
	track2 = [(pf.x, pf.P)]

	for i in range(1,25):
		if i==13:
			u = u*-1
		s.propogate(i, u)
		truth.append(s.x)
		msmnt.append(H*s.x + matrix([random.normal(0, math.sqrt(R[0,0])), random.normal(0, math.sqrt(R[1,1]))]).T)
		kf.update(i, (msmnt[-1], R), u)
		track1.append((kf.x, kf.P))
		pf.update(i, (msmnt[-1], R), u)
		track2.append((pf.x, pf.P))
		res = 20
		for j in range(res):
			t = i + float(j)/res
			s.propogate(t, u)
			truth.append(s.x)
			kf.extrap(t, u)
			track1.append((kf.x, kf.P))
			pf.extrap(t, u)
			track2.append((pf.x, pf.P))

	t1 = truth[0]
	for x in truth[1:]:
		t1 = bmat([t1,x])

	print t1

	fig = plt.figure()
	ax = fig.add_subplot(111)#, projection='2d')
	ax.scatter(t1[1,:], t1[3,:])

	plt.show()

if __name__ == '__main__':
	main()
