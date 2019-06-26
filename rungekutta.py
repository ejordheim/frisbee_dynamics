import numpy as np

def rk4(x,t,tau,derivsRK):
##  Runge-Kutta integrator (4th order)
## Input arguments -
##   x = current value of dependent variable
##   t = independent variable (usually time)
##   tau = step size (usually timestep)
##   derivsRK = right hand side of the ODE; derivsRK is the
##             name of the function which returns dx/dt
##             Calling format derivsRK(x,t).
## Output arguments -
##   xout = new value of x after a step of size tau
    half_tau = 0.5*tau
    F1 = derivsRK(x,t)
    t_half = t + half_tau
    xtemp = x + half_tau*F1
    F2 = derivsRK(xtemp,t_half)
    xtemp = x + half_tau*F2
    F3 = derivsRK(xtemp,t_half)
    t_full = t + tau
    xtemp = x + tau*F3
    F4 = derivsRK(xtemp,t_full)
    xout = x + tau/6.*(F1 + F4 + 2.*(F2+F3))
    return xout

def rka(x,t,tau,err,derivsRK):

## Adaptive Runge-Kutta routine
## Inputs
##   x          Current value of the dependent variable
##   t          Independent variable (usually time)
##   tau        Step size (usually time step)
##   err        Desired fractional local truncation error
##   derivsRK   Right hand side of the ODE; derivsRK is the
##              name of the function which returns dx/dt
##              Calling format derivsRK(x,t).
## Outputs
##   xSmall     New value of the dependent variable
##   t          New value of the independent variable
##   tau        Suggested step size for next call to rka

##* Set initial variables
    tSave = t;  xSave = x    # Save initial values
    safe1 = .9;  safe2 = 4.  # Safety factors
    eps = np.spacing(1) # smallest value

##* Loop over maximum number of attempts to satisfy error bound
    maxTry = 100

    for iTry in range(1,maxTry):
	
##* Take the two small time steps
        half_tau = 0.5 * tau
        xTemp = rk4(xSave,tSave,half_tau,derivsRK)
        t = tSave + half_tau
        xSmall = rk4(xTemp,t,half_tau,derivsRK)
  
  ##* Take the single big time step
        t = tSave + tau
        xBig = rk4(xSave,tSave,tau,derivsRK)
  
  ##* Compute the estimated truncation error
        scale = err * (np.abs(xSmall) + np.abs(xBig))/2.
        xDiff = xSmall - xBig
        errorRatio = np.max( [np.abs(xDiff)/(scale + eps)] )
        
        #print safe1,tau,errorRatio
  
  ##* Estimate news tau value (including safety factors)
        tau_old = tau

        tau = safe1*tau_old*errorRatio**(-0.20)
        tau = np.max([tau,tau_old/safe2])
        tau = np.min([tau,safe2*tau_old])
  
  ##* If error is acceptable, return computed values
        if errorRatio < 1 : 
            xSmall = xSmall ## +  (xDiff)/15
            return xSmall, t, tau  

##* Issue error message if error bound never satisfied
    print ('ERROR: Adaptive Runge-Kutta routine failed')
    return