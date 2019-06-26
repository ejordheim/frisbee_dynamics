import numpy as np
import constants as con

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

def tailwind(strength):
    '''
    Returns wind from the -x direction with velocity (strength)
    '''
    return strength*np.array([1,0,0])

def headwind(strength):
    '''
    Returns wind from the +x direction with velocity (strength)
    '''
    return strength*np.array([-1,0,0])

def crosswind(strength,sign):
    '''
    Returns wind from the y direction with velocity (strength) 
    and direction (sign)
    '''
    return strength*np.array([0,sign,0])

def vertwind(strength,sign):
    '''
    Returns wind from the z direction with velocity (strength) 
    and direction (sign)
    '''
    return strength*np.array([0,0,sign])

def constwind(strength,direction):
    '''
    Returns wind from the arbitrary direction (direction) 
    with velocity (strength)
    '''
    return strength*direction

def spiralwind(position,strength,sign,origin):
    '''
    Returns wind that spirals around the point (origin), 
    with magnitude (strength) and rotational direction (sign)
    '''
    phi = np.arctan((position[1]-origin[1])/(position[0]-origin[0]))
    return strength*np.array([np.sin(phi),sign*np.cos(phi),0])

def sinusoidal_gusting(magnitude,frequency,time):
    '''
    Returns strength multiplication factor for gusting wind. Varies accrding to
    sin^2 with amplitude (magnitude) and frequency (frequency)
    '''
    return magnitude * np.sin(time * frequency)**2

def wind(wind,pos):
    '''
    Function uses if the user specified adding wind conditions and 
    determines the desired type and parameters to update the 
    param array as needed
    '''
    if wind == 1: # if wind is enabled
        wind_type = int(input(' Wind type: \n\t\t1-Tailwind,\n\t\t '+
                              '2-Headwind,\n\t\t'+
                              ' 3-Crosswind,\n\t\t 4-Vertical wind,\n\t\t'+
                              ' 5-Arbitrary constant wind,\n\t\t 6-Spiral wind: '))
        wind_strength = float(input(' Enter wind strength (m/s): '))
        gusting = int(input(' Enter a 1 to add gusting wind conditions: '))
        if gusting == 1: # if gusting is enabled
            magnitude = float(input(' Enter a gusting multiplication factor: '))
            frequency = float(input(' Enter the gusting frequency'+
                                    ' (cycles/second): '))
            con.param[20] = 1
            con.param[21] = [magnitude, frequency]
        # Tailwind
        if wind_type == 1: 
            con.param[17] = 1
            con.param[18] = tailwind(wind_strength).tolist()
            con.param[19] = 1
        # Headwind
        if wind_type == 2:
            con.param[17] = 1
            con.param[18] = headwind(wind_strength).tolist()
            con.param[19] = 2
        # Crosswind
        if wind_type == 3:
            sign = int(input(' Enter direction of wind: 1-Right, -1-Left: '))
            con.param[17] = 1
            con.param[18] = crosswind(wind_strength,sign).tolist()
            con.param[19] = 3
        # Vetrical wind
        if wind_type == 4:
            sign = int(input(' Enter direction of wind: 1-Down, -1-Up: '))
            con.param[17] = 1
            con.param[18] = vertwind(wind_strength,sign).tolist()
            con.param[19] = 4
        # Arbitrary wind
        if wind_type == 5:
            direction = np.array(input(' Enter direction of wind: [x, y, z]: '))
            con.param[17] = 1
            con.param[18] = constwind(wind_strength,direction).tolist()
            con.param[19] = 3
        # Spiraling wind
        if wind_type == 6:
            origin = [int(n) for n in input(' Enter origin of wind'+
                      ' rotation: [x,y,z]: ').replace(' ','').replace('[',
                                  '').replace(']','').split(',')]
            sign = int(input(' Enter direction of wind: 1-Clockwise,'+
                             ' -1-Anticlockwise: '))
            con.param[17] = 1
            con.param[18] = spiralwind(pos,wind_strength,sign,origin)
            con.param[19] = 6
            con.param[20] = [wind_strength,sign,origin]
    else:
        # If they do not want to have wind
        wind_type = 0
    return 