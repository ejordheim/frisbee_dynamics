Written by Errol Jordheim

This is a frisbee flight simulator written entirely in python 3

This was initially written as a final project for a computational physics course, but I have
continued to add to in since the completion of the course. I intend to continue adding features
and making the code more efficient as I find time to do so.

=======
Physics
=======

The flight of a frisbee cna be modeled as a spinning disc subject to three forces, lift, drag, 
and gravity. As defined, gravity always acts on the center of mass (COM) of the disc, and always 
points along the z-axis. The lift and drag forces, however, do not act on the COM of the disc,
instead acting on a point offset from the COM called the center of pressure (COP). As calculating
the actual location of the COP is not a well defined calculation for the conplex shape of a
frisbee, the offset COP is modeled by instead locating the location of the lift and drag forces
at the COM, and adding moments on the disc along all axes of rotation. Each moment is dependent 
on the orientation and shape of the disc, as well as the angle of attack (the angle between 
the pitch and the velocity vector) and the angular velocity around the axis. The coefficients 
for these contributions have been experimentally calculated by multiple groups from flight 
and wind-tunnel tests. These forces and moments can be used to full equations of motion of the
system.

Wind is implemented by performing the above calculations with a reference frame change from the 
observer frame to that with no wind at the COM of the disc. This is valid as the addition of wind
only changes the direction of the flow of air over the surface of the disc, assuming that the 
density of the air is unchanged. 

Not included in this model is the Magnus force, as the magnitude of the related coefficient is
small enough for the flight dynamics to be minimally effected. 

The base model of a flying frisbee is implemented according to a masters thesis by Sarah
Hummel written in 2003. The coding style and structure, along with the implementation of 
wind and animation is original work.

=========
Operation
=========

To run the code, all contained python files must be downloaded to the same folder. 
  execution.py file acts as the make file as well as taking care of all plotting
  threeD_model.py is the derivative calculator used by the adaptive Runge-Kutta method used
  wind.py contains all functions necessary to add wind to the system
  constants.py sets all of the constant values of the system as well as the wind parameters
  draw_disc.py creates a visual 3D disc for animating the motion
  rungekutta.py contains the Runge-Kutta and adaptive Runge-Kutta execution files 
  initial_cond.py contains various initial conditions and the ability to create user defined
                  definitions
  interpf.py contains a quadratic Lagrange polynomial interpolation function

To run the program, compile and run the execution.py file. It will prompt various user input
for animation, wind conditions, and initial launch conditions.
	If animation is enabled, the program will display the trajectory and disc orientation for every
	10 iterations of the adaptive Runge-Kutta method
Currently implemented wind conditions are headwind, tailwind, crosswind, vertical wind, arbitrary
direction constant wind, and spiraling wind around a user-defined origin. Gusting wind is also an 
option, generating a multiplicative factor to any of the mentioned wind conditions that varies 
according to a sin^2 (t) time dependence.

Output of the file is an animated graph showing the trajectory of the disc over time as well as
the orientation of the disc with respect to the observer reference frame. Once iteration is
complete, two figures are presented, an interactable flight trajectory plot, and a series of
subplots showing the time evolution of position, velocity, pitch and roll angles, and spin. The
program will also output to the console the total flight time, the x-distance traveled, the 
y-distance traveled, and the maximum height reached during flight.
