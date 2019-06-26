import numpy as np

def initial_cond():
    cond = int(input(' Enter initial conditions:\n'+
                     '\t\t1-Long flight backhand, \n'+
                     '\t\t2-Long flight forehand with flip, \n'+
                     '\t\t3-Freefall high spin, \n'+
                     '\t\t4-Other conditions : '))
    if cond == 1:
        '''
        # Long flight conditions used in the Hummel thesis for comparison
        theta0 = 0.21             # initial pitch angle about y-axis
        phi0 = -0.07               # initial roll angle about x-axis
        gamma0 = 5.03               # initial spin angle (doesn't matter) about z-axis
        speed0 = 13.5             # initial speed
        beta0 = 0.03                # angle between velocity and horizontal
        gammadot0 = 54.0                # initial spin
        phidot0 = -14.94                     # initial roll instability
        thetadot0 = -1.48                    # initial pitch instability
        x0 = -0.9; y0 = -0.63; z0 = -0.91;   # initial position
        '''
        
        # long flight backhand conditions 
        theta0 = 0.05             # initial pitch angle about y-axis
        phi0 = -0.2               # initial roll angle about x-axis
        gamma0 = 0.               # initial spin angle (doesn't matter) about z-axis
        speed0 = 15.0             # initial speed
        beta0 = 0.                # angle between velocity and horizontal
        gammadot0 = 100.0                # initial spin
        phidot0 = 0.                     # initial roll instability
        thetadot0 = 0                    # initial pitch instability
        x0 = 0.1; y0 = 0.1; z0 = -1.0;   # initial position
        
    if cond == 2:  
        # long flight flip forehand conditions
        theta0 = 0.05             # initial pitch angle about y-axis
        phi0 = 0.2                # initial roll angle about x-axis
        gamma0 = 0.               # initial spin angle (doesn't matter) about z-axis
        speed0 = 50.0             # initial speed
        beta0 = 0.                # angle between velocity and horizontal
        gammadot0 = -100.0               # initial spin
        phidot0 = 0.                     # initial roll instability
        thetadot0 = 0                    # initial pitch instability
        x0 = 0.1; y0 = 0.1; z0 = -1.0;   # initial position
        
    if cond == 3:        
        # freefall high spin
        theta0 = 0.               # initial pitch angle about y-axis
        phi0 = 0.                 # initial roll angle about x-axis
        gamma0 = 0.               # initial spin angle (doesn't matter) about z-axis
        speed0 = 0.01             # initial speed
        beta0 = 0.                # angle between velocity and horizontal
        gammadot0 = 100.0                # initial spin
        phidot0 = 0.                     # initial roll instability
        thetadot0 = 0.                   # initial pitch instability
        x0 = 0.; y0 = 0.; z0 = -300.0;   # initial position
        
    if cond == 4:
        theta0 = np.radians(float(input(' Enter initial pitch angle'+
                                        '(degrees): ')))
        phi0 = np.radians(float(input(' Enter initial roll angle'+
                                      '(degrees): ')))
        gamma0 = 0.
        speed0 = float(input(' Enter initial speed (m/s): '))
        beta0 = np.radians(float(input(' Enter angle between pitch '+
                                       'and velocity vector (degrees): ')))
        gammadot0 = float(input(' Enter initial spin (rad/s) (|spin| > 0): '))
        phidot0 = float(input(' Enter initial roll angular velocity'+
                              ' (rad/sec): '))
        thetadot0 = float(input(' Enter initial pitch angular '+
                                'velocity (rad/sec): '))
        x0 = float(input(' Enter initial x-position (m): '))
        y0 = float(input(' Enter initial y-position (m): '))
        z0 = float(input(' Enter initial z-position (m). Enter negative number: '))
    
    return theta0,phi0,gamma0,speed0,beta0,gammadot0,phidot0,thetadot0,x0,y0,z0