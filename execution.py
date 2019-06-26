'''
Created by: Errol Jordheim
Final Project for Computational Physics PHYS 416
April 17, 2018
View README.txt for details of the program and theoretical background
'''

# Import packages for mathematics and plotting
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

# Import necessary functions
import rungekutta as rk
import threeD_model as model
import wind as w
import draw_disc as dd
import initial_cond as ini
from interpf import intrpf

# Get inputs for run conditions
animate = int(input(' Enter a 1 to animate while iterating: '))
if animate == 1:
    animation_step = int(input(' Enter the number of iterative steps between'+
                               ' each animation step: '))

# Define initial conditions:
theta0,phi0,gamma0,speed0,beta0,gammadot0,phidot0,thetadot0,x0,y0,z0 = ini.initial_cond()

# Generate wind conditions
wind_true = int(input(' Enter a 1 to add wind to launch conditions: '))
w.wind(wind_true,[x0,y0,z0])
    
# We can then define:
alpha0 = theta0 - beta0         # angle between plane of disc and velocity
vx0 = speed0 * np.cos(beta0)    # initial x velocity in rest frame
vy0 = 0.0                       # initial y velocity in rest frame
vz0 = -speed0 * np.sin(beta0)    # initial z velocity in rest frame

# Set the initial state of the system
#x0 = np.array[x, y, z, vx, vy, vz, phi, theta, gamma,
#                 phidot, thetadot, gammadot]
state = np.array([x0, y0, z0, vx0, vy0, vz0, phi0, theta0, gamma0, 
                  phidot0, thetadot0, gammadot0])

# Set time constraints on the simulation
time = 0.0        # initial time
iter_max = 10000        # maximum simulation time in seconds
tau = 1.e-4       # initial guess of timestep
err = 1.e-5       # maximum error tolerance

# Initialize arrays for plotting
tplot = np.array([]); 
xplot = np.array([]); 
yplot = np.array([]);
zplot = np.array([]);
vxplot = np.array([]); 
vyplot = np.array([]);
vzplot = np.array([]);
phiplot = np.array([]);
thetaplot = np.array([]);
gammadotplot = np.array([]);
vmagplot = np.array([])

# Initalize iteration counter
iteration = 0

# Loops while the height of the disc is greater than zero and the max iteration
# is not exceeded
while state[2] < 0.0 and iteration < iter_max:
    
    # record values for plotting
    tplot = np.append(tplot,time);
    xplot = np.append(xplot,state[0]); 
    yplot = np.append(yplot,-state[1]);
    zplot = np.append(zplot,-state[2]);
    vxplot = np.append(vxplot,state[3]); 
    vyplot = np.append(vyplot,-state[4]);
    vzplot = np.append(vzplot,-state[5]);
    phiplot = np.append(phiplot,state[6]);
    thetaplot = np.append(thetaplot,state[7]);
    gammadotplot = np.append(gammadotplot,state[11]);
    vmagplot = np.append(vmagplot,np.sqrt(state[3:6].dot(state[3:6])))
    
    # find new state using adaptive Runge-Kutta
    [state, time, tau] = rk.rka(state,time,tau,err,model.threeD_frisbee_rk);
    
    # advance iteration number
    iteration = iteration + 1
    
    # display height and iteration number every certain number of iterations
    if (iteration % animation_step) < 1:
        print('Iteration = '+str(iteration))
        print('Height (m) = '+str(-state[2]))
        print('Time (s) = '+str(time))
        if animate == 1:
            # extract phi and theta from the current state
            phi = state[6]; theta = state[7];
        
            # velocity and speed in n-frame
            v = state[3:6]
            v_mag = np.sqrt(v.dot(v))
            
            # velocity values for plotting
            v_plot_x = np.array([state[0],state[0]+v[0]/v_mag])
            v_plot_y = np.array([-state[1],-state[1]-v[1]/v_mag])
            v_plot_z = np.array([-state[2],-state[2]-v[2]/v_mag])
            
            # disc orientation values for plotting
            disc_x,disc_y,disc_z = dd.draw_disc(50,theta,phi)
            
            # Plots the trajectory of the disc along with the orientation
            # Disc trajectory plot
            fig = plt.figure(1)
            plt.clf()
            ax = fig.add_subplot(1,2,1,projection='3d')
            ax.plot(xplot, yplot, zplot, label='Disc flight')
            ax.plot(v_plot_x, v_plot_y, v_plot_z, label='Disc velocity')
            ax.axis('equal')
            ax.legend()
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('z')
            ax.view_init(elev = 3, azim = 165)
            
            # Disc orientation plot
            ax = fig.add_subplot(1,2,2,projection='3d')
            ax.plot(disc_x, -disc_y, -disc_z)
            ax.axis('equal')
            ax.set_xlabel('x')
            ax.set_ylabel('y')
            ax.set_zlabel('z')
            ax.set_xlim(-0.15,0.15)
            ax.set_ylim(-0.15,0.15)
            ax.set_zlim(-0.15,0.15)
            ax.view_init(elev = 0.1,azim = 165)
            plt.draw()
            plt.show()

            plt.pause(0.1)

# add final state values for plotting
tplot = np.append(tplot,time); 
xplot = np.append(xplot,state[0]); 
yplot = np.append(yplot,-state[1]);
zplot = np.append(zplot,-state[2]);
vxplot = np.append(vxplot,state[3]); 
vyplot = np.append(vyplot,-state[4]);
vzplot = np.append(vzplot,-state[5]);
phiplot = np.append(phiplot,state[6]);
thetaplot = np.append(thetaplot,state[7]);
gammadotplot = np.append(gammadotplot,state[11]);
vmagplot = np.append(vmagplot,np.sqrt(state[3:6].dot(state[3:6])))

# use interpolation function to find values when z=0
tplot[-1] = intrpf(0,zplot[-3:],tplot[-3:])
xplot[-1] = intrpf(0,zplot[-3:],xplot[-3:])
yplot[-1] = intrpf(0,zplot[-3:],yplot[-3:])
zplot[-1] = 0.0
vxplot[-1] = intrpf(0,zplot[-3:],vxplot[-3:])
vyplot[-1] = intrpf(0,zplot[-3:],vyplot[-3:])
vzplot[-1] = intrpf(0,zplot[-3:],vzplot[-3:])
phiplot[-1] = intrpf(0,zplot[-3:],phiplot[-3:])
thetaplot[-1] = intrpf(0,zplot[-3:],thetaplot[-3:])
gammadotplot[-1] = intrpf(0,zplot[-3:],gammadotplot[-3:])
vmagplot[-1] = intrpf(0,zplot[-3:],vmagplot[-3:])
    
# print out final state values when z=0
print('\n Flight time = '+str(tplot[-1])+' seconds')
print(' X-distance traveled = '+str(xplot[-1]-xplot[0])+' meters')
print(' Y-distance traveled = '+str(yplot[-1]-yplot[0])+' meters')
print(' Max height = '+str(np.max(zplot))+' meters')

# plot final disc trajectory overwriting animation    
fig = plt.figure(1)
plt.clf()
ax = fig.gca(projection='3d')
ax.plot(xplot, yplot, zplot, label='Disc flight')
ax.axis('equal')
ax.legend()
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()

# adds spacing between plots in subplot
plt.subplots_adjust(hspace=0.5)
plt.subplots_adjust(wspace=0.5)

fig = plt.figure(2)
ax = fig.add_subplot(2,2,1)
ax.plot(tplot, xplot, label='x')
ax.plot(tplot, yplot, label='y')
ax.plot(tplot, zplot, label='z')
ax.legend()
ax.grid(1)
ax.set_xlabel('time (s)')
ax.set_ylabel('position (m)')

ax = fig.add_subplot(2,2,2)
ax.plot(tplot, thetaplot, label='theta (pitch)')
ax.plot(tplot, phiplot, label='phi (roll)')
ax.legend()
ax.grid(1)
ax.set_xlabel('time (s)')
ax.set_ylabel('angle (rad)')

ax = fig.add_subplot(2,2,3)
ax.plot(tplot, vxplot, label='x-velocity')
ax.plot(tplot, vyplot, label='y-velocity')
ax.plot(tplot, vzplot, label='z-velocity')
ax.plot(tplot, vmagplot, label='speed')
ax.legend()
ax.grid(1)
ax.set_xlabel('time (s)')
ax.set_ylabel('speed (m/s)')

ax = fig.add_subplot(2,2,4)
ax.plot(tplot, gammadotplot, label='gamma dot (spin)')
ax.legend()
ax.grid(1)
ax.set_xlabel('time (s)')
ax.set_ylabel('angular velocity (rad/sec)')