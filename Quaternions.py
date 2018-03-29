#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 27 23:45:00 2018

@author: yudzhi
"""
import numpy as np

class qt(object):
    """
    Quaternions
    
    INPUT: qt(a, b, c, d) - float
                OR
           qt(angle, axis): u0 - float, u - vector
    
    Attributes:
        _a: q0,   0th element of quaternion
        _b: q1*i, 1st element of quaternion
        _c: q2*j, 2nd element of quaternion
        _d: q3*k, 3rd element of quaternion
        _axis:  axis of rotation [qi/sqrt(1-q0^2)]
        _angle: angle of rotation 2*acos(q0)
        _array: array representation 
                |a+id   -b-ic| _ |alpha  -beta |
                |b-ic    a-id| - |beta*  alpha*|
    """
    
    def __init__(self, *args):
        if len(args) == 4:
            self.init_abc(args[0], args[1], args[2], args[3])
        if len(args) == 2:
            self.init_AngAx(args[0], args[1])
            
    def __str__(self):
#        return('<' + str(self._a) + " + " + str(self._b) + "i + " +
#                str(self._c) + "j + " + str(self._d) + "k >")
        return("(" + str(self.q0) + ", " + str(self.q1) + "," +
                 str(self.q2) + "," + str(self.q3) + ")")
        
    def init_abc(self, a,b,c,d):
        self._a = a
        self._b = b
        self._c = c
        self._d = d

    def init_AngAx(self, ang, ax):
        """angle in rad and normalized vector"""
        self._a = np.sin(ang/2)
        self._b = np.cos(ang/2) * ax[0]
        self._c = np.cos(ang/2) * ax[1]
        self._d = np.cos(ang/2) * ax[2]  
                
    def qdot(self, qt2):
        """Quaternion multiplication"""
        qt3_array = np.dot(self.qarray, qt2.qarray)
        qt3 = self.qt_from_array(qt3_array)
        return qt3
    
    def qt_from_array(self,qarray):
        """find a,b,c,d from qt array"""
        a3 = qarray.real[0][0]
        b3 = qarray.real[1][0]
        c3 = - qarray.imag[0][1] 
        d3 = qarray.imag[1][0]
        qt3 = qt(a3,b3,c3,d3)
        return qt3
        
    
    @property
    def qarray(self):
        """array quanternion representation"""
        self._array = np.array([[self.q0 + 1j * self.q3, - self.q1 - 1j * self.q2],
                                [self.q1 - 1j * self.q2, self.q0 - 1j * self.q3]])
        return self._array
                
    @property
    def q0(self):
        """float: 0th element of quaternion"""
        return self._a
    
    @property
    def q1(self):
        """float: 1st element of quaternion"""
        return self._b
    
    @property
    def q2(self):
        """float: 2nd element of quaternion"""
        return self._c
    
    @property
    def q3(self):
        """float: 3rd element of quaternion"""
        return self._d
    """
    quaternion multiplication
    INPUT: qt1 = [a1,b1,c1,d1]
           qt2 = [a2,b2,c2,d2]
           qt = a + bi + cj + dk
    OUTPUT: qt = qt1 * qt2
        Re(q) = a = a2*a1 - b2*b1 - c2c1 - d2d1
        Im(q) = a2*vec(q1) + a1*vec(q2) + [vec(qt1),vec(qt2)]
    """
    