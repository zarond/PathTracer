from math import *
import numpy as np
import matplotlib.pyplot as mp

def Refraction_angle(n1,n2,omega1):
    omega2 = sin(omega1)*n1/n2
    if omega2 <= 1.0:
        omega2 = asin(omega2)
        return omega2
    else:
        return pi/2

def Fresnel(n1,n2,omega1):
    omega2 = sin(omega1)*n1/n2
    if omega2 <= 1.0:
        omega2 = asin(omega2)
        Rs = (n1*cos(omega1)-n2*cos(omega2))/(n1*cos(omega1)+n2*cos(omega2))
        Rp = (n2*cos(omega1)-n1*cos(omega2))/(n2*cos(omega1)+n1*cos(omega2))
        return ((Rs**2+Rp**2)*0.5)
    else:
        return 1.0

def Shlick(F0,omega):
    return (F0+(1.0-F0)*pow((1-cos(omega)),5))

X_d = np.array(range(0,91))
X = X_d * pi/(2*90)

n1 = 1.0
n2 = 1.5

f1 = lambda x: Fresnel(n1,n2,x)
vf1 = np.vectorize(f1)

f2 = lambda x: Fresnel(n2,n1,x)
vf2 = np.vectorize(f2)

F0 = pow((n1-n2)/(n1+n2),2)
f_shlick = lambda x: Shlick(F0,x)
vf_shlick = np.vectorize(f_shlick)

f_shlick_reverse = lambda x: Shlick(F0,Refraction_angle(n2,n1,x))
vf_shlick_reverse = np.vectorize(f_shlick_reverse)

f_reverse = lambda x: Fresnel(n1,n2,Refraction_angle(n2,n1,x))
vf_reverse = np.vectorize(f_reverse)

f_angle1 = lambda x: Refraction_angle(n1,n2,x)
f_angle2 = lambda x: Refraction_angle(n2,n1,x)
vf_angle1 = np.vectorize(f_angle1)
vf_angle2 = np.vectorize(f_angle2)

Y1 = vf1(X)
Y2 = vf2(X)
Y_shlick = vf_shlick(X)

mp.xlim((0.0, 90.0)) 
mp.plot(X_d,Y1, label='True Fresnel')
mp.plot(X_d,Y_shlick, label='Shlick')
mp.title('Air -> Glass')
mp.legend()
mp.figure()
#mp.show()

Y_shlick = vf_shlick_reverse(X)
mp.xlim((0.0, 90.0)) 
mp.plot(X_d,Y2, label='True Fresnel')
mp.plot(X_d,Y_shlick, label='Shlick')
mp.title('Glass -> Air')
mp.legend()
mp.figure()
#mp.show()

Y3 = vf_reverse(X)
mp.xlim((0.0, 90.0)) 
mp.plot(X_d,Y2, label='True Fresnel')
mp.plot(X_d,Y3, label='Reverse Fresnel')
mp.title('Glass -> Air')
mp.legend()
mp.figure()
#mp.show()

Y_angle = vf_angle1(X)
Z = (1-Y1) * (1.0 - vf2(Y_angle))
mp.xlim((0.0, 90.0))
mp.plot(X_d,Z, label='True Fresnel Energy')
mp.plot(X_d,(1.0-vf_shlick(X))**2, label='Shlick Energy')
mp.title('Thin Glass energy')
mp.legend()
mp.show()
