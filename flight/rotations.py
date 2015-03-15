import numpy as np

'''
Find Rbe, that rotates a vector from earth fixed to body frame, from euler angles, using Tait-Bryan
  convention, i.e. Rbe=Rot_x(roll)*Rot_y(pitch)*Rot_z(yaw)
'''
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

'''
From Rbe, which rotates a vector from earth fixed to body frame, find the  euler angles, using Tait-Bryan
  convention, i.e. Rbe=Rot_x(roll)*Rot_y(pitch)*Rot_z(yaw)
'''
def dcm_to_euler(M, cy_thresh=None):
    M = np.asarray(M)
    if cy_thresh is None:
        try:
            cy_thresh = np.finfo(M.dtype).eps * 4
        except ValueError:
            cy_thresh = _FLOAT_EPS_4
    r11, r12, r13, r21, r22, r23, r31, r32, r33 = M.flat

    cy = math.sqrt(r33*r33 + r23*r23)

    if cy > cy_thresh: # cos(y) not close to zero, standard form
        z = math.atan2(r12,  r11) # atan2(-cos(y)*sin(z), cos(y)*cos(z))
        y = math.atan2(-r13,  cy) # atan2(sin(y), cy)
        x = math.atan2(r23, r33) # atan2(-cos(y)*sin(x), cos(x)*cos(y))
    else: # cos(y) (close to) zero, so x -> 0.0 (see above)
        # so r21 -> sin(z), r22 -> cos(z) and
        z = math.atan2(-r21,  r22) # atan2(sin(z), cos(z))
        y = math.atan2(-r13,  cy) # atan2(sin(y), cy)
        x = 0.0
    return z, y, x
