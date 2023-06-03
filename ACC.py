import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt

# Parameters
M = 1650
c = 10
gamma = 1
f_0, f_1, f_2 = 0.1, 5, 0.25
a, b = 1.11, 1.59
v_0, v_d = 27.7, 22
p_sc = 100
tau_d = 1.8

# the time interval between two time steps
delta = 0.01
# total simulation time
total_time = 100
# simulation iterations
iteration_num = int(total_time/delta)
# a small number which we assume it is small enough in our expriments
epsilon = 0.0001
# Initial state (v_f, v_l, D)
x = np.zeros((3,iteration_num))
x[:,0] = np.array([18, 10, 150])

# Definition for F_r
def F_r(x, t):
    return f_0 + f_1*x[0][t] + f_2*x[0][t]**2

# Safety constraints h(x)
def h(x, t):
    return x[2][t] - tau_d*x[0][t]

# Control barrier function B(x)
def B(x, t):
    return -np.log(h(x, t)/(1+h(x, t)))

# Control lyapunov function V(x)
def V(x, t):
    return (x[0][t] - v_d)**2

# L_g B(x, t)
def LB_g(x, t):
    return tau_d/(M*(1 + x[2][t] - tau_d*x[0][t])*(x[2][t] - tau_d*x[0][t]))

# L_f B(x)
def LB_f(x, t):
    return -tau_d*F_r(x, t)/(M*(1 + x[2][t] - tau_d*x[0][t])*(x[2][t] - tau_d*x[0][t]))

# L_g V(x)
def LV_g(x, t):
    return 2*(x[0][t] - v_d)/M

# L_f V(x)
def LV_f(x, t):
    return -2*F_r(x, t)*(x[0][t] - v_d)/M

# the acceleration of the lead car. it is a disturbance
def acceleration_of_leader(t):
    """
    [0,20)  a=0
    [20,30) a=1
    [30,40) a=0
    [40,50) a=0.5
    [50,60) a=0
    [60,70) a=-1
    [70,100)a=0
    """
    a=0
    if(t*delta<20):
        a=0
    elif(t*delta<30):
        a=1
    elif(t*delta<40):
        a=0
    elif(t*delta<50):
        a=0.5
    elif(t*delta<60):
        a=0
    elif(t*delta<70):
        a=-1
    else:
        a=0
    return a

u_function=[]
for t in range(iteration_num-1):
# Cost matrix
    H_acc = 2*np.array([[1/M**2, 0],
                        [0, p_sc]])

    F_acc = -2*np.array([F_r(x, t)/M**2, 0])

    A_clf = np.array([LV_g(x, t), -1])

    b_clf = -LV_f(x, t) - c*V(x, t)

    A_cbf = np.array([LB_g(x, t), 0])

    b_cbf = -LB_f(x, t) + gamma/B(x, t)
    
    # using cvxpy to solve the optimization problem
    u = cp.Variable(2)
    cons = [ A_clf @ u <= b_clf, A_cbf @ u <= b_cbf ]
    obj = cp.Minimize(0.5*cp.quad_form(u, H_acc) + F_acc.T @ u)
    prob = cp.Problem(obj, cons)
    prob.solve()

    a_L = acceleration_of_leader(t)

    x[0][t+1] = x[0][t] + delta*(-F_r(x, t)/M + u.value[0]/M)

    x[1][t+1] = x[1][t] + delta*a_L

    x[2][t+1] = x[2][t] + delta*(x[1][t] - x[0][t]) 
    uu=u.value[0]
    if(x[2][t+1] - tau_d*x[0][t+1]<epsilon):
        x[0][t+1] = (x[2][t+1]-epsilon)/tau_d
        uu =M*((x[0][t+1]-x[0][t])/delta+F_r(x,t)/M)

    x[0][t+1] = x[0][t] + delta*(-F_r(x, t)/M + uu/M)

    x[1][t+1] = x[1][t] + delta*a_L

    x[2][t+1] = x[2][t] + delta*(x[1][t] - x[0][t]) 

    u_function.append(uu)

T = np.arange(iteration_num)*delta

# plot the speed of the two cars
plt.figure()
plt.plot(T, x[0],color="blue")
plt.plot(T, x[1],color="red")
plt.plot(T, 0*T+v_d, color="black",linestyle="--")
plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.legend(["controlled car","lead car","reference speed"])
plt.axis([0,100,8,28])
plt.show()
# plot h(x)
plt.figure()
h_function = []
for i in range(iteration_num):
    h_function.append(h(x,i))
plt.plot(T, h_function,color="blue")
plt.plot(T, 0*T, color="black",linestyle="--")
plt.xlabel("Time (s)")
plt.ylabel("h (m)")
plt.axis([0,100,-10,120])
plt.show()
# plot u
plt.figure()
plt.plot(T[0:iteration_num-1],np.array(u_function)/(M*10),color="blue")
plt.xlabel("Time (s)")
plt.ylabel("u/(Mg)")
plt.axis([0,100,-1,1])
plt.show()
