#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 27 23:45:00 2018

@author: yudzhi
"""
import nummpy as np

class qt():
    """
    Quaternions
    """
    
    def __init__(self, a, b, c, d):
        self.vec = [b, c, d]
        self.
def qt_dot(qt1,qt2):
    """
    quaternion multiplication
    INPUT: qt1 = [a1,b1,c1,d1]
           qt2 = [a2,b2,c2,d2]
           qt = a + bi + cj + dk
    OUTPUT: qt = qt1 * qt2
        Re(q) = a = a2*a1 - b2*b1 - c2c1 - d2d1
        Im(q) = a2*vec(q1) + a1*vec(q2) + [vec(qt1),vec(qt2)]
    """
    