"""
Run this file once to generate a file:

    $ python gen-ik.py

which can be loaded using:

    >>> import dill
    >>> ik_xy2th = fn_loaded = dill.load(open('scripts/inverse-kinematics.pyfunc', 'rb'))
    >>> ik_xy2th(1, 2)  # (x, y) -> (th1, th2)
"""

import sympy as sp

## define constants
l1 = 0.137
l2 = 0.250
l3 = 0.030

## define symbols
th1, th2, r, phi = sp.symbols('th1 th2 r phi', real=True)  # type: ignore

## the trick is to use polar coordinates
phi_eqn = (th1 + th2) / 2
d = (th1 - th2) / 2
r_eqn = l1 * sp.cos(d) + sp.sqrt(l2**2 - l1**2 * sp.sin(d)**2)

## get two solutions (since th1 and th2 are pretty much interchangeable)
sol1, sol2 = sp.solve(
    [sp.Eq(phi, phi_eqn), sp.Eq(r, r_eqn)],
    [th1, th2]
)
assert sol1[0] == sol2[1] and sol1[1] == sol2[0]

## create a python function from the symbolic expression
# (phi, r) -> (th1, th2)
ik_pr2th = sp.lambdify([phi, r], sol1)

# (xe, ye) -> (th1, th2)
from math import atan2, sqrt
ik_xy2th = lambda xe, ye: ik_pr2th(atan2(ye, xe), sqrt(xe**2 + ye**2)) 

## check that it works
def forward_kinematics(th1, th2):
    from math import sin, cos, sqrt

    # common subexpressions
    d = (th1 - th2)/2
    s = (th1 + th2)/2
    common = l1 * cos(d) + sqrt(l2**2 - l1**2 * sin(d)**2)

    # end points
    xe = common * cos(s)
    ye = common * sin(s)
    return (xe, ye)

import random
from math import isclose
for i in range(100):
    angles_in = random.random()*3, random.random()*3
    
    if angles_in[0] > angles_in[1]:
        angles_in = (angles_in[1], angles_in[0])
    
    angles_out = ik_xy2th(*forward_kinematics(*angles_in))
    
    close = all(isclose(i,o) for i,o in zip(angles_in, angles_out))
    if not close:
        print(f'{angles_in=}, {angles_out=}')


## save to disk!
import dill
filename = 'scripts/inverse-kinematics.pyfunc'
dill.dump(ik_xy2th, open(filename, 'wb'), recurse=True)

# load using:
fn_loaded = dill.load(open(filename, 'rb'))
assert fn_loaded(1., 2.) == ik_xy2th(1., 2.)
