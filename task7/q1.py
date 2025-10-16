from scipy.integrate import solve_ivp

import numpy as np
from math import radians

x0,y0,theta0=10,5,radians(30) #initial conditions
v0=5                          #initial velocity in m/s
a0=1                          #initial acceleration in m/s^2
omega0=radians(10)            #initial angular velocity in rad/s
dt=0.2                        #time step in seconds

def yaw_rate_model(t,state):  #function to define the ordinary differential equation system
    x,y,theta,v=state
    dx=v*np.cos(theta)
    dy=v*np.sin(theta) 
    dtheta=omega0
    dv=a0
    return [dx,dy,dtheta,dv]

t_span=(0,dt)
state0=[x0,y0,theta0,v0] #initial state vector
sol=solve_ivp(yaw_rate_model,t_span,state0,method='RK45',t_eval=[dt]) #solution object returned by solve_ivp
x,y,theta,v=sol.y[:,-1] #final state after time step dt
print(f"Position: ({x}, {y}), Heading: {np.degrees(theta)} degrees, Velocity: {v} m/s")

#defining cone positions in the global frame
cones = {
    "B1": np.array([12.0, 7.0]),
    "B2": np.array([8.0, 6.0]),
    "Y1": np.array([13.0, 4.0]),
    "Y2": np.array([9.0, 3.0])
}

# translating then rotating by -theta
R_init = np.array([[np.cos(theta0), np.sin(theta0)],
                   [-np.sin(theta0), np.cos(theta0)]])
R_final = np.array([[np.cos(theta), np.sin(theta)],
                    [-np.sin(theta), np.cos(theta)]])

car_init_col = np.array([x0, y0]).reshape(2,1)
car_final_col = np.array([x, y]).reshape(2,1)

for name, pos in cones.items():
    pos_col = pos.reshape(2,1)  # column vector (transpose)
    conepos_relative_initial_col = R_init @ (pos_col - car_init_col)
    conepos_relative_final_col   = R_final @ (pos_col - car_final_col)
    print(f"{name} -> Initial frame: {conepos_relative_initial_col}, Final frame: {conepos_relative_final_col}")

