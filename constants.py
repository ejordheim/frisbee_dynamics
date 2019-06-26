import numpy as np

# Variables that are global:
m = 0.175               # mass of disc
g = 9.7935              # acceleration due to gravity
A = 0.057               # platform area
d = 0.2694              # diameter of disc
rho = 1.23              # air density
Ir = 0.001219           # Moment of inertia matrix for radial values
In = 0.002352           # Moment of inertia matrix for normal vector

# Coefficients:
CL0 = 0.3331            # lift intercept
CLa = 1.9124            # lift slope
CD0 = 0.1769            # drag intercept
CDa = 0.685             # drag slope
CM0 = -0.0821            # pitch moment
CMa = 0.4338            # pitch moment
CRr = 0.00171 * np.sqrt(d/g)           # roll moment due to spin
# Approximated coefficients
CMq = -0.0144             # pitch damping
CRp = -0.0125             # roll damping
CNr = -0.0000341          # decrease in spin rate

# Wind parameters initialized to zero unless otherwise needed
wind = 0;
w_v = 0
wind_type = 0
wind_params = 0
gusting = 0
gusting_params = 0

# define constant
param = [m,g,A,d,rho,Ir,In,CL0,CLa,CD0,CDa,CM0,CMa,CMq,CRr,CRp,CNr,wind,
         w_v,wind_type,wind_params,gusting,gusting_params]

# getter function
def constants():
    global param 
    param = [m,g,A,d,rho,Ir,In,CL0,CLa,CD0,CDa,CM0,CMa,CMq,CRr,CRp,CNr,wind,
             w_v,wind_type,wind_params,gusting,gusting_params]