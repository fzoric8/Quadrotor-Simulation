import numpy as np
from collections import namedtuple

DesiredState = namedtuple('DesiredState', 'pos vel acc jerk yaw yawdot')

def polyder(t, k = 0, order = 10):
    if k == 'all':
        terms = np.array([polyder(t,k,order) for k in range(1,5)])
    else:
        terms = np.zeros(order)
        coeffs = np.polyder([1]*order,k)[::-1]
        pows = t**np.arange(0,order-k,1)
        terms[k:] = coeffs*pows
    return terms

def Hessian(T,order = 10,opt = 4):
    n = len(T)
    Q = np.zeros((order*n,order*n))
    for k in range(n):
        m = np.arange(0,opt,1)
        for i in range(order):
            for j in range(order):
                if i >= opt and j >= opt:
                    pow = i+j-2*opt+1
                    Q[order*k+i,order*k+j] = 2*np.prod((i-m)*(j-m))*T[k]**pow/pow
    return Q

def Circle_waypoints(n,Tmax = 2*np.pi):
    t = np.linspace(0,Tmax, n)
    x = 1+0.5*np.cos(t)
    y = 1+0.5*np.sin(t)
    z = 1+0*t
    return np.stack((x, y, z), axis=-1)

def Helix_waypoints(n,Tmax = 2*np.pi):

    t = np.linspace(0, Tmax, n)
    x = 1+0.5*np.cos(t)
    y = 1+0.5*np.sin(t)
    z = t/Tmax*2

    return np.stack((x, y, z), axis=-1)

def Linear_waypoints(ps, pg, n):
    
    x = np.linspace(ps[0], pg[0], n)
    y = np.linspace(ps[1], pg[1], n)
    z = np.linspace(ps[2], pg[2], n)

    return np.stack((x, y, z), axis=-1)

def Parabola_waypoints():
    # todo: Add parabola waypoints
    pass

def Third_order_polynomial(): 
    # TODO: Add method to generate third order polynomial 
    pass

# Generate 2d zig_zag path
def gen_zigzag(x_start, y_start, n_points, x_d): 
    pts = [(x_start, y_start)]
    for i in range(0, n_points):
        if i > 0:
            pt_ = (x_start+i*x_d, (-1)**i*y_start)
            pts.append(pt_)
    return np.array(pts)

def interp_zigzag(pts, z_val=0.5): 
    wpts_ = np.array([pts[0, 0], pts[0, 1], z_val])
    for i, pt in enumerate(pts):
        if i < len(pts)-1: 
            pt_start = (pts[i, 0], pts[i, 1], z_val)
            pt_goal = (pts[i+1, 0], pts[i+1, 1],z_val)
            new_pts = Linear_waypoints(pt_start, pt_goal, 5)
            wpts_ = np.vstack((wpts_, new_pts[1:, :]))   
    return wpts_