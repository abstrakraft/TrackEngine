import transform
from linalg import Vector

class CoordSystem(object):
	def __init__(self, origin, rot):
		self.origin = Vector(origin)
		self.rot = rot

	def transform(self, coord):
		return Coordinate(self.transform_dict[type(coord.frame).__name__](coord.vector, coord.frame, self), self)

	def transfer(self, coord):
		return Coordinate(coor.vector, self)

class Cartesian(CoordSystem):
	transform_dict = {'Cartesian': transform.cart2cart,
	                  'RBE': transform.rbe2cart}

class RBE(CoordSystem):
	transform_dict = {'Cartesian': transform.cart2rbe}

class Coordinate(object):
	def __init__(self, vector, frame):
		self.vector = Vector(vector)
		self.frame = frame

class Measurement(Coordinate):
	def __init__(self, vector, cov, frame):
		Coordinate.__init__(self, vector, frame)
		self.cov = cov

