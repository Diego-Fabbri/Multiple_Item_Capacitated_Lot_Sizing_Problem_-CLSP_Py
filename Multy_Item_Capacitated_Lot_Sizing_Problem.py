import sys
import pandas as pd
import time, numpy as np
import pyomo.environ as pyo
from pyomo.environ import *
from pyomo.opt import SolverFactory

#Parameters
T = 6 # Planning Horizon = n+1

J = 3 # Items

Io = 0 # Initial Inventory Level

In = 0 # Final Inventory Level

d = np.array([[30,0,0],
              [0,0,0],
              [80,30,40],
              [0,0,0],
              [40,70,60]]) # d_tj

K = np.array([400,150,100]) # K_j

h = np.array([[4,3,2],
              [4,3,2],
              [4,3,2],
              [4,3,2],
              [4,3,2]]) # h_tj

c = np.array([[0,0,0],
              [0,0,0],
              [0,0,0],
              [0,0,0],
              [0,0,0]]) # h_tj

C = np.array([[100,100,100],
              [100,100,100],
              [100,100,100],
              [100,100,100],
              [100,100,100]]) # h_tj

range_t = range(1,T) # Time t=0 not taken into account
range_j = range(0,J)

#Create Model
model = pyo.ConcreteModel()

#Define variables                           ### Do note that Variables' have T+1 element
                                            ### and parameters array have T elements therefore constraints are arranged accordingly in terms of indexing
model.I = pyo.Var(range(0,T), # index t
                  range(0,J), #index j
                  bounds = (0,None),
                  initialize=0)

model.y = pyo.Var(range(0,T), # index t
                  range(0,J), #index j
                  within=Binary,
                  initialize = 0)

model.q = pyo.Var(range(0,T), # index t
                  range(0,J), #index j
                  bounds = (0,None),
                  initialize = 0)

I = model.I
y = model.y
q = model.q

#Constraints 
model.C1 = pyo.ConstraintList() 
# for j in range_j:
#     model.C1.add = pyo.Constraint(expr= I[0,j] == Io) # Set Io
model.C1 = pyo.Constraint(expr= sum(I[0,j] for j in range_j)== Io) # Set Io

# model.C2 = pyo.ConstraintList() 
# for j in range_j:
#     model.C2.add = pyo.Constraint(expr= I[T-1,j] == In) # Set In
    
model.C2 = pyo.Constraint(expr= sum(I[T-1,j] for j in range_j)== In) # Set In

model.C3 = pyo.ConstraintList() 
for t in range_t:
    for j in range_j:
        model.C3.add(expr= q[t,j] + I[t-1,j] - I[t,j] == d[t-1][j])
        
model.C4 = pyo.ConstraintList() 
for t in range_t:
    for j in range_j:
        model.C4.add(expr= q[t,j] -C[t-1][j]*y[t,j] <= 0)
    
# Define Objective Function
model.obj = pyo.Objective(expr = sum(K[j]*y[t,j] +c[t-1][j]*q[t,j]+h[t-1][j]*I[t,j] for t in range_t for j in range_j) , 
                          sense = minimize)

begin = time.time()
opt = SolverFactory('cplex')
results = opt.solve(model)

deltaT = time.time() - begin # Compute Exection Duration

model.pprint()

sys.stdout = open("Multi_item_Capacitated_Lot-Sizing_(CLSP)_Problem_Results.txt", "w") #Print Results on a .txt file

print('Time =', np.round(deltaT,2))
if (results.solver.status == SolverStatus.ok) and (results.solver.termination_condition == TerminationCondition.optimal):

    print('Total Cost (Obj value) =', pyo.value(model.obj))
    print('Solver Status is =', results.solver.status)
    print('Termination Condition is =', results.solver.termination_condition)
    print(" " )
    
    for t in range(0,T):
        print("Time t = ", t )
        for j in range_j:
            print("Item j = ", j+1)
            print('---> q[', t ,'][',j+1,']=',pyo.value(q[t,j]))
            print('---> I[', t ,'][',j+1,']=',pyo.value(I[t,j]))
            print('---> y[', t ,'][',j+1,']=',pyo.value(y[t,j]))
            print(' ')
        print(" " )
elif (results.solver.termination_condition == TerminationCondition.infeasible):
   print('Model is unfeasible')
  #print('Solver Status is =', results.solver.status)
   print('Termination Condition is =', results.solver.termination_condition)
else:
    # Something else is wrong
    print ('Solver Status: ',  result.solver.status)
    print('Termination Condition is =', results.solver.termination_condition)
    
sys.stdout.close()