import numpy as np
import constants as con

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

param = con.param

'''
param = np.array([m,g,A,d,rho,Ir,In,CL0,CLa,CD0,CDa,CM0,CMa,CMq,CRr,CRp,
                  0 1 2 3 4   5  6  7   8   9   10  11  12  13  14  15 
                  CNr,wind,w_v,wind_type,wind_params,gusting,gusting_params 
                  16  17   18  19        20          21      22
state = np.array([x0, y0, z0, vx0, vy0, vz0, phi0, theta0, gamma0,
                  0   1   2   3    4    5    6     7       8   
                                          phidot0, thetadot0, gammadot0])
                                          9        10         11
'''

def draw_disc(N,theta,phi): 
    '''
    Takes in the number of points making up the disc (N), and the orientation
    of the disc (theta and phi) and generates a 3D representation of the 
    disc rotated according to the orientation
    '''
    
    # define sine and cosine of theta and phi for frame transformations
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    
    # create a ring of points the size of a frisbee
    eta = np.arange(0,N)*2.*np.pi/(N-1)
    r = param[3]*np.ones(len(eta))/2.
    
    # convert cylindrical coordinates to cartesian coordinates
    x = r*np.cos(eta)
    y = r*np.sin(eta)
    z = np.zeros(N)
    
    # these are in the disc-frame. We can then convert them into the n-frame 
    # using transformation matrix
    list1 = np.zeros((N,3))
    
    # Define transformation matrix from intertial frame to disc frame
    # Transformation is in the z axis then the y axis, rotating by a 
    # negative angle due to our defined axes
    T_c_n = np.array([[c_theta, s_phi*s_theta, -c_phi*s_theta],
                      [0, c_phi, s_phi],
                      [s_theta, -s_phi*c_theta, c_phi*c_theta]])
    
    for i in range(0,N):
        # convert to array of [[x1,y1,z1],[x2,y2,z2],...] 
        # for coordinate transformation
        list1[i][0] = x[i]
        list1[i][1] = y[i]
        list1[i][2] = z[i]
        # move from disc frame to observer frame
        list1[i] = T_c_n.transpose().dot(list1[i])
        
    for j in range(0,N):
        # convert back to [x1,x2,...],[y1,y2,...],[z1,z2,...]
        x[j] = list1[j][0]
        y[j] = list1[j][1]
        z[j] = list1[j][2]
    
    # define arrays for generating the filled disc
    solid_x1 = np.zeros(3*N+1)
    solid_y1 = np.zeros(3*N+1)
    solid_z1 = np.zeros(3*N+1)
    
    for k in range(0,N):
        # First creates ring around the edge of disc, then creates lines 
        # from every point around the ring to the center to fill the body
        solid_x1[k] = x[k]
        solid_x1[2*k + 1 + N] = 0
        solid_x1[2*k + N] = x[k]
        solid_y1[k] = y[k]
        solid_y1[2*k + 1 + N] = 0
        solid_y1[2*k + N] = y[k]
        solid_z1[k] = z[k]
        solid_z1[2*k + 1 + N] = 0
        solid_z1[2*k + N] = z[k] 
        
    return solid_x1,solid_y1,solid_z1