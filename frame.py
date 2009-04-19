import transform

class Frame(object):
	def transform(self, coord):
		return Coordinate(self.transform_dict[type(coord.frame)](coord.vector, coord.frame, self), self)

class CartesianFrame(Frame):
	transform_dict = {CartesianFrame: transform.cart2cart}

	def __init__(self, origin, rot):
		self.origin = origin
		self.rot = rot

class Coordinate(object):
	def __init__(self, vector, frame):
		self.vector = vector
		self.frame = frame

class Measurement(Coordinate):
	def __init__(self, vector, cov, frame):
		self.cov = cov

