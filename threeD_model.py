import numpy as np
import constants as con
import wind as w


def threeD_frisbee_rk(s,t):
    '''
    Takes in a state (s) and the current time (t) and returns the derivative 
    of s (s_dot) for use in an adaptive Runge-Kutta method
    
    param = np.array([m,g,A,d,rho,Ir,In,CL0,CLa,CD0,CDa,CM0,CMa,CMq,CRr,CRp,
                      0 1 2 3 4   5  6  7   8   9   10  11  12  13  14  15 
                      CNr,wind,w_v,wind_type,wind_params,gusting,gusting_params 
                      16  17   18  19        20          21      22
    state = np.array([x0, y0, z0, vx0, vy0, vz0, phi0, theta0, gamma0,
                      0   1   2   3    4    5    6     7       8   
                                              phidot0, thetadot0, gammadot0])
                                              9        10         11
    '''
    
    # import param from constants
    param = con.param
    
    # extract values from state vector for ease of interpretation
    vx = s[3]; vy = s[4]; vz = s[5]
    phi = s[6]; theta = s[7];
    dphi = s[9]; dtheta = s[10]; dgamma = s[11]
    
    # define sin and cos of theta and phi for later use
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    
    # if wind is included, change velocity vectors to be in still frame
    wind = param[17]; 
    # implement wind conditions if specified
    if wind == 1:
        # recalculate spiraling wind conditions
        if param[19] == 6:
            wind_strength = param[20][0]; sign = param[20][1]; 
            origin = param[20][2]
            w_v = w.spiralwind(s[0:3],wind_strength,sign,origin)
        else:
            w_v = param[18]
        
        # add gusting if specified
        if param[21] == 1:
            gusting_params = param[22]
            w_v=w.sinusiodal_gusting(gusting_params[0],gusting_params[1],t)*w_v
        
        # wind adjusted velocities
        vx = vx - w_v[0]
        vy = vy - w_v[1]
        vz = vz - w_v[2]
    
    # Define transformation matrix from intertial frame to disc frame
    # Transformation matrix is in the z axis then the y axis, rotating by 
    # a negative angle due to our defined axes
    #Tz = np.array([[1, 0, 0],[0, c_phi, s_phi],[0, -s_phi, c_phi]])
    #Ty = np.array([[c_theta, 0, -s_theta],[0, 1, 0],
    #                           [s_theta, 0, np.cos(theta)]])
    #T_c_n = Ty * Tz # trnsformation matrix from n to c
    T_c_n = np.array([[c_theta, s_phi*s_theta, -c_phi*s_theta],
                      [0, c_phi, s_phi],
                      [s_theta, -s_phi*c_theta, c_phi*c_theta]])

    # c-vector expressed in n-frame
    c3 = T_c_n[2,:]

    
    # velocity and speed in n-frame
    v = np.array([vx, vy, vz])
    v_mag = np.sqrt(v.dot(v))
    
    vc3 = v.dot(c3) # speed in the c3 direction
    vp = v - vc3*c3 # velocity along just the plane of the disc
    vp_mag = np.sqrt(vp.dot(vp)) # speed along the plane of the disc
    
    alpha = np.arctan(vc3/vp_mag) # angle of attack
    const = 1./2. * param[2] * param[4] * v_mag**2 # constant used often
    
    # define unit vectors for moment calculations
    unit_x_c = v/v_mag # unit vector of velocity in n-frame
    unit_y_c = vp/vp_mag # unit vector of velocity in plane
    unit_z_c = np.cross(c3,unit_y_c) # unit vector to perpendicular to 
                                     # c3 and unit_y_c

    # calculate angular momentum on the disc in c-frame then convert to n-frame
    omega_n_c = np.array([[dphi*c_theta], [dtheta], [dphi*s_theta + dgamma]])
    omega_n_n = np.reshape(T_c_n.transpose().dot(omega_n_c),(1,3))
    
    omega_vp = np.dot(omega_n_n,unit_y_c)
    omega_perp = np.dot(omega_n_n,unit_z_c)
    omega_spin = np.dot(omega_n_n,c3)
    
    # calculate angle of attack dependent coefficients
    CL0 = param[7]; CLa = param[8]
    CL = CL0 + CLa * alpha # lift coefficient
    
    CD0 = param[9]; CDa = param[10]
    CD = CD0 + CDa * (alpha + CL0/CLa)**2 # drag coefficient
    
    CM0 = param[11]; CMa = param[12]
    CM = CM0 + CMa * alpha # pitch coefficient
    
    # calculate moments
    # Roll moment in n-frame
    CRr = param[14]; CRp = param[15]
    M_vp = const * param[3] * (CRr * omega_spin + CRp * omega_vp) * unit_y_c
    # Pitch moment in n-frame
    CMq = param[13]
    M_perp = const * param[3] * (CM + CMq * omega_perp) * unit_z_c
    # Spin moment
    CNr = param[16]
    M_spin = np.array([0, 0, CNr*omega_spin]) # in c-frame
    
    # total moment in c-frame
    M_tot = (T_c_n.dot(np.reshape(M_vp,(3,1))) + 
             T_c_n.dot(np.reshape(M_perp,(3,1))) + np.reshape(M_spin,(3,1)))

    # lift and drag forces and their directions
    lift = const * CL
    drag = const * CD
    unit_lift = -np.cross(unit_x_c, unit_z_c)
    unit_drag = -1*unit_x_c


    # total COM force
    F_tot = lift*unit_lift + drag*unit_drag + np.array([0,0,param[0]*param[1]])

    # Calculate the total derivative of the state s
    s_dot = np.copy(s)
    
    s_dot[:3] = s[3:6]              # velocity
    s_dot[3] = F_tot[0]/param[0]    # x-acceleration
    s_dot[4] = F_tot[1]/param[0]    # y-acceleration
    s_dot[5] = F_tot[2]/param[0]    # z-acceleration
    s_dot[6] = dphi                 # angular velocity
    s_dot[7] = dtheta               # angular velocity
    s_dot[8] = dgamma               # angular velocity
    
    # Calculate angular accelerations
    Ir = param[5]; In = param[6]
    s_dot[9] = (M_tot[0,0] + 2*Ir*dtheta*dphi*s_theta - 
         In*dtheta*(dphi*s_theta+dgamma))*c_theta/(Ir)
    s_dot[10] = (M_tot[1,0] + In*dphi*c_theta*(dphi*s_theta+dgamma) - 
         Ir * dphi**2 *c_theta*s_theta)/Ir
    s_dot[11] = (M_tot[2,0] - In*(s_dot[9]*s_theta + dtheta*dphi*c_theta))/In

    return s_dot