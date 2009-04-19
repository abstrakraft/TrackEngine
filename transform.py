def cart2cart(v, src, dst):
	return dst.rot.inverse().transform(src.origin + src.rot.transform(v) - dst.origin)
