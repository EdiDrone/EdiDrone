import numpy as np

def euler_to_dcm(psi, theta, phi):
	cT = np.cos(theta)
	sT = np.sin(theta)
	cPhi = np.cos(phi)
	sPhi = np.sin(phi)
	cPsi = np.cos(psi)
	sPsi = np.sin(psi)
	a1 = (cT*cPsi)
	a2 = (sPhi*sT*cPsi-cPhi*sinPsi)
	a3 = (cPhi*sT*cPsi+sPhi*sinPsi)

	b1 = cT*sPhi
	b2 = sinPsi*sT*sPhi+cPsi*cPhi
	b3 = cPsi*sT*sPhi-sinPsi*cPhi

	c1 = (-sT)
	c2 = sPhi*cT
	c3 = cPhi*cT

	return  np.matrix('a1 b1 c1; a2 b2 c2; a3 b3 c3')
