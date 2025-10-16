import numpy as np
import casadi as ca
#Operations on symbols produce symbolic expressions (a computation graph)
#parameters
dt=0.02 # time step in seconds
a_const=1.3 # constant acceleration in m/s^2
omega_const=0.23 # constant yaw rate in rad/s 
v0=5.5 # initial velocity in m/s
v_des=10 # desired velocity in m/s
N = 3 # number of steps

#initial position
x0=0.1
y0=3.1
theta=np.arctan2(y0,x0)

#segment
x1,y1=0.9,3
x2,y2=1.4,4.1
theta_ref=np.arctan2(y2-y1,x2-x1)

#weights
w_heading=1
w_cross=1
w_vel=2
w_a=0.1
w_omega=0.1

#control variables
a=ca.SX.sym('a',N) # acceleration, n-vector symbolic variable named a is created, a[k] is the value at kth step
omega=ca.SX.sym('omega',N) # yaw rate
U=ca.vertcat(a,omega) #stacked the vectors into a 2NX1 column vector

#initial state
st=ca.vertcat(x0,y0,theta,v0) # initial state, symbolic 4x1 state vector, contains numbers now, becomes symbolic after the first step
J=0 # cost function

def cross_track(st):  #function to calculate perpendicular distance from point to line segment
    x_s,y_s=st[0],st[1]
    num=(y2-y1)*x_s-(x2-x1)*y_s+x2*y1-y2*x1
    den=ca.sqrt((y2-y1)**2+(x2-x1)**2) 
    return num/den

def step(st,a_k,w_k): #function to define dynamics of the vehicle
    x,y,theta,v=st[0],st[1],st[2],st[3]
    x_next=x+v*ca.cos(theta)*dt
    y_next=y+v*ca.sin(theta)*dt
    theta_next=theta+w_k*dt
    v_next=v+a_k*dt
    st_next=ca.vertcat(x_next,y_next,theta_next,v_next)
    return st_next #returns a 4x1 symbolic vector representing the next state

for k in range(N): # function to calculate the cost
    e_head=ca.atan2(ca.sin(st[2]-theta_ref), ca.cos(st[2]-theta_ref)) # heading error  
    e_cross=cross_track(st) # cross track error
    e_vel=st[3]-v_des # velocity error
    J=J+w_heading*e_head**2+w_cross*e_cross**2+w_vel*e_vel**2 +w_a*a[k]**2+w_omega*omega[k]**2 # cost function
    st=step(st,a[k],omega[k]) 

g=ca.vertcat(a[0]-a_const,omega[0]-omega_const)
#nlp-> Non linear programmming solver
nlp={'x':U,'f':J,'g':g} # nlp dictionary, x is the optimization variable, f is the objective to minimize
opts={'ipopt.print_level':0,'print_time':0}  
solver=ca.nlpsol('solver','ipopt',nlp,opts) # create an NLP solver instance using IPOPT, casadi supplies gradients, jacobiasn, hessian
sol=solver(x0=np.zeros(2*N)) # initial guess of 0 for all control inputs

a_opt = np.array(sol['x'][:N])
omega_opt = np.array(sol['x'][N:]).flatten()
J_opt = float(sol['f'])

print("Optimal accelerations (m/s^2):", a_opt)
print("Optimal yaw rates (rad/s):", omega_opt)
print("Minimum cost J:", J_opt)

st_num = np.array([x0, y0, theta, v0])
traj = [st_num.copy()]
for k in range(N):
    st_num[0] += st_num[3]*np.cos(st_num[2])*dt
    st_num[1] += st_num[3]*np.sin(st_num[2])*dt
    st_num[2] += omega_opt[k]*dt
    st_num[3] += a_opt[k]*dt
    traj.append(st_num.copy())
for i in range(len(traj)):
    print(f"Step {i}: x={traj[i][0]:.3f}, y={traj[i][1]:.3f}, theta={traj[i][2]:.3f}, v={traj[i][3]:.3f}")



