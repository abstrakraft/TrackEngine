from numpy import *
import model

m = model.CV_PWCA_Model(3, 1)
x = mat([1.0, 2.0, 3.0, 4.0, 5.0, 6.0]).T
P = mat([
	[1,2,3,4,5,6],
	[2,3,4,5,6,7],
	[3,4,5,6,7,8],
	[4,5,6,7,8,9],
	[5,6,7,8,9,10],
	[6,7,8,9,10,11]])

print m.project_state(x, 3,2)
print m.project_state(x, 3,3)
print m.project_state(x, 4,3)
print m.project_state(x, 2,2)
print m.project_state(x, 2,1)

print m.project_cov(P, 3,2)
print m.project_cov(P, 3,3)
print m.project_cov(P, 4,3)
print m.project_cov(P, 2,2)
print m.project_cov(P, 2,1)
