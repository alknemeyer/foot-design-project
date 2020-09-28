from scipy import optimize
import numpy as np

def forward_kinematics(q):
    from math import sin, cos, sqrt

    l1 = 10
    l2 = 10
    th1, th2 = q

    # common subexpressions
    d = (th1 - th2)/2
    s = (th1 + th2)/2
    common = l1 * cos(d) + sqrt(l2**2 - l1**2 * sin(d)**2)

    # end points
    xe = common * cos(s)
    ye = common * sin(s)
    return [xe - 5, ye - 5]

def forward_kinematics_jac(q):
    from math import sin, cos, sqrt
    l1 = 10
    l2 = 10
    th1, th2 = q
    d = (th1 - th2)/2
    s = (th1 + th2)/2

    cs = cos(s)
    ss = sin(s)
    cd = cos(d)
    sd = sin(d)
    ex = 1/2 * l1 * sd + l1**2 * sin(d)*cos(d) / (2 * sqrt(l2**2 - l1**2 * sin(d)**2))
    ex2 = l1*cos(d) + sqrt(l2**2 - l1**2 * sin(d)**2)

    return np.array([
        [- cs * ex - 1/2*ss*ex2,
         + cs * ex - 1/2*ss*ex2],
        [- ss * ex + 1/2*cs*ex2,
         + ss * ex + 1/2*cs*ex2],
    ])

# https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.root.html
sol = optimize.root(
    forward_kinematics, [1., 2.], jac=forward_kinematics_jac, method='hybr',
)

forward_kinematics(sol.x)
