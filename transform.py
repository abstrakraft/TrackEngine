import math
from linalg import Vector

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
