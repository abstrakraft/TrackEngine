from numpy import *
import math

#TODO: optimize?
def project_indices(Dx, Nx, Dy, Ny):
	x = []
	y = []
	min_N = min(Nx, Ny)
	for d in range(min(Dx, Dy)):
		x.extend(range(Nx*d, Nx*d+min_N))
		y.extend(range(Ny*d, Ny*d+min_N))
	return x, y

def project_state(x, Dx, Nx, Dy, Ny):
	xdx, ydx = project_indices(Dx, Nx, Dy, Ny)
	y = mat(zeros((Dy*Ny, 1)))
	y[ydx] = x[xdx]
	return y

def project_cov(Px, Dx, Nx, Dy, Ny):
	xdx, ydx = project_indices(Dx, Nx, Dy, Ny)
	Py = mat(zeros((Dy*Ny, Dy*Ny)))
	Py[ix_(ydx,ydx)] = Px[ix_(xdx,xdx)]
	return Py

# everything below is most likely broken

def cart2cart(v, src, dst):
	return dst.rot.inverse().transform(src.origin + src.rot.transform(v) - dst.origin)

#0 bearing points down y axis
def colocated_rbe2cart(v):
	r, b, e = v
	return Vector([r*math.sin(b)*math.cos(e), r*math.cos(b)*math.cos(e), r*math.sin(e)])

def rbe2cart(v, src, dst):
	w = colocated_rbe2cart(v)
	return cart2cart(w, src, dst)

def colocated_cart2rbe(v):
	x, y, z = v
	r = v.norm()
	return Vector([r, math.atan2(x, y), math.asin(z/r)])

def cart2rbe(v, src, dst):
	w = cart2cart(v, src, dst)
	return colocated_cart2rbe(w)
