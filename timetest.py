from linalg import Matrix

x = Matrix.randn(3,3)
for i in xrange(1000000):
	x = x*x
