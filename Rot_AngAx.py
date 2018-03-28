# -*- coding: utf-8 -*-
"""
YuDzhi 27.03.2018
"""
from sympy import Eijk, KroneckerDelta, sin, cos, pi
#from sympy.abc import i,j#, theta
import sympy as sp
#import numpy as np
sp.init_printing(use_unicode=True)

def Rot_AngAx(n, theta):
    """
    Rotation matrix
    Rodrigues formula
    angle-axis parametrization
    INPUT: n - vector [nx,ny,nz]
           0 <= theta <= pi
    OUTPUT: Rot - matrix(3x3)
    nx,ny,nz = symbols('nx,ny,nz')
    n = [nx,ny,nz]
    """

    dimens = 3
    """
    R = sp.MatrixSymbol('R', dimens, dimens)
    """
    i,j = sp.symbols('i,j')
    R = sp.FunctionMatrix(dimens, dimens, sp.Lambda((i,j), 
                                                  cos(theta)*KroneckerDelta(i,j)))
    for k in range(dimens):
        R -= sp.FunctionMatrix(dimens,dimens, sp.Lambda((i,j), sin(theta) * Eijk(i,j,k) * n[k]))
        
    Rot = sp.Matrix(R)

    R2 = sp.zeros(dimens)
    for i in range(dimens):
        R2.row_op(i, lambda v,j: (1-cos(theta)) * n[i]*n[j])
    Rot += R2
    Rot = sp.N(Rot,4)
    #Rot = sp.matrix2numpy(Rot)
    return Rot
"""
nx,ny,nz = sp.symbols('nx,ny,nz')
n = [nx,ny,nz]
theta = sp.symbols('theta')
Rot = Rot_AngAx (n, theta)
#sp.init_printing(use_unicode=True)
print(Rot)
"""