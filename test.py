from linalg import Matrix, Vector
import filter
import model
import system
import math
import random
import cairowindow

def main():
	Q = Matrix([[1.0, 1.0],[1.0, 2.0]])
	H = Matrix([[1.0, 0.0, 0.0, 0.0],
	            [0.0, 0.0, 1.0, 0.0]])
	R = Matrix([[10.0, 0.0],[0.0, 10.0]])
	u = Vector([0, -9.8])

	x_init = Vector([0.0, 50.0, 0.0, 100.0])
	P_init = Matrix([[0.2, 0.0, 0.0, 0.0],
	                 [0.0, 1.0, 0.0, 0.0],
	                 [0.0, 0.0, 0.2, 0.0],
	                 [0.0, 0.0, 0.0, 1.0]])
	m = model.Inertial2DModel(Q)
	s = system.System(m, x_init, 0)

	truth = [x_init]
	msmnt = [H*x_init + Vector([random.gauss(0, R[0,0]), random.gauss(0, R[1,1])])]
	kf = filter.KalmanFilter(m, H, Vector([msmnt[0][0], 50.0, msmnt[0][1], 100.0]), P_init, 0)
	pf = filter.KalmanFilter(m, H, Vector([msmnt[0][0], 50.0, msmnt[0][1], 100.0]), P_init, 0)
	#pf = filter.ParticleFilter(m, H, Vector([msmnt[0][0], 50.0, msmnt[0][1], 100.0]), P_init, 0)
	#pf = filter.NewFilter(m, H, Vector([msmnt[0][0], 50.0, msmnt[0][1], 100.0]), P_init, 0)

	track1 = [(kf.x, kf.P)]
	track2 = [(pf.x, pf.P)]
	print kf.x
	print pf.x

	for i in range(1,25):
		print i
		if i==13:
			u = u*-1
		s.propogate(i, u)
		truth.append(s.x)
		msmnt.append(H*s.x + Vector([random.gauss(0, R.m[0,0]), random.gauss(0, R.m[1,1])]))
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

	tw = TrackWindow(truth, msmnt, track1, track2)
	tw.run()

class TrackWindow(cairowindow.CairoWindowSurface):
	def __init__(self, truth, msmnt, track1, track2):
		cairowindow.CairoWindowSurface.__init__(self)
		self.truth = truth
		self.msmnt = msmnt
		self.track1 = track1
		self.track2 = track2

	def draw(self, cr, width, height):
		cairowindow.CairoWindowSurface.draw(self, cr, width, height)
		offset = 8.0
		cr.translate(offset, height-offset)
		cr.scale(1.0, -1.0)

		cr.set_source_rgb(0.0, 0.0, 1.0)
		for t in self.truth:
			cr.arc(t.m[0], t.m[2], 8.0, 0, 2*math.pi)
			cr.fill()

		cr.set_source_rgb(0.5, 0.5, 0.5)
		for t in self.msmnt:
			cr.arc(t.m[0], t.m[1], 6.0, 0, 2*math.pi)
			cr.fill()

		cr.set_source_rgb(0.0, 1.0, 0.0)
		for (x,P) in self.track1:
			cr.arc(x.m[0], x.m[2], 4.0, 0, 2*math.pi)
			cr.fill()

		cr.set_source_rgb(1.0, 0.0, 0.0)
		for (x,P) in self.track2:
			cr.arc(x.m[0], x.m[2], 3.0, 0, 2*math.pi)
			cr.fill()

if __name__ == "__main__":
	main()
