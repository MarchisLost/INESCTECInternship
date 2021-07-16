from ortools.sat.python import cp_model
import numpy as np

# CONSTANTES
Fi = 80 # euros
Ri = 20
Wi = 160000000 #Hz
Pti = 20 #dBm
GTi = 0 #dBm
GRi = 0 #dBm
luz = 300000000 #m/s
Pndbm = -95 #dBm
Pnw = 3.16e-13
fi = 2.4e9
BLER_eMBB = 0.1
BLER_URLLC= 0.001
N = 100
densidade = [[ 3, 4, 3, 5, 6], [2, 3, 3, 2, 2]] # densidade de pessoas em cada subárea de cada slice
S = 2 # nº slices
T = [13000000, 6.5e6]
UAVPOS = [( 5, 5, 20), 
            ( 5, 15, 20), 
            ( 5, 25, 20), 
            ( 5, 35, 20), 
            ( 5, 45, 20), 
            ( 5, 55, 20), 
            ( 5, 65, 20), 
            ( 5, 75, 20), 
            ( 5, 85, 20), 
            ( 5, 95, 20), 
            ( 15, 5, 20), 
            ( 15, 15, 20), 
            ( 15, 25, 20), 
            ( 15, 35, 20), 
            ( 15, 45, 20), 
            ( 15, 55, 20), 
            ( 15, 65, 20), 
            ( 15, 75, 20), 
            ( 15, 85, 20), 
            ( 15, 95, 20), 
            ( 25, 5, 20), 
            ( 25, 15, 20), 
            ( 25, 25, 20), 
            ( 25, 35, 20), 
            ( 25, 45, 20), 
            ( 25, 55, 20), 
            ( 25, 65, 20), 
            ( 25, 75, 20), 
            ( 25, 85, 20), 
            ( 25, 95, 20), 
            ( 35, 5, 20), 
            ( 35, 15, 20), 
            ( 35, 25, 20), 
            ( 35, 35, 20), 
            ( 35, 45, 20), 
            ( 35, 55, 20), 
            ( 35, 65, 20), 
            ( 35, 75, 20), 
            ( 35, 85, 20), 
            ( 35, 95, 20), 
            ( 45, 5, 20), 
            ( 45, 15, 20), 
            ( 45, 25, 20), 
            ( 45, 35, 20), 
            ( 45, 45, 20), 
            ( 45, 55, 20), 
            ( 45, 65, 20), 
            ( 45, 75, 20), 
            ( 45, 85, 20), 
            ( 45, 95, 20), 
            ( 55, 5, 20), 
            ( 55, 15, 20), 
            ( 55, 25, 20), 
            ( 55, 35, 20), 
            ( 55, 45, 20), 
            ( 55, 55, 20), 
            ( 55, 65, 20), 
            ( 55, 75, 20), 
            ( 55, 85, 20), 
            ( 55, 95, 20), 
            ( 65, 5, 20), 
            ( 65, 15, 20), 
            ( 65, 25, 20), 
            ( 65, 35, 20), 
            ( 65, 45, 20), 
            ( 65, 55, 20), 
            ( 65, 65, 20), 
            ( 65, 75, 20), 
            ( 65, 85, 20), 
            ( 65, 95, 20), 
            ( 75, 5, 20), 
            ( 75, 15, 20), 
            ( 75, 25, 20), 
            ( 75, 35, 20), 
            ( 75, 45, 20), 
            ( 75, 55, 20), 
            ( 75, 65, 20), 
            ( 75, 75, 20), 
            ( 75, 85, 20), 
            ( 75, 95, 20), 
            ( 85, 5, 20), 
            ( 85, 15, 20), 
            ( 85, 25, 20), 
            ( 85, 35, 20), 
            ( 85, 45, 20), 
            ( 85, 55, 20), 
            ( 85, 65, 20), 
            ( 85, 75, 20), 
            ( 85, 85, 20), 
            ( 85, 95, 20), 
            ( 95, 5, 20), 
            ( 95, 15, 20), 
            ( 95, 25, 20), 
            ( 95, 35, 20), 
            ( 95, 45, 20), 
            ( 95, 55, 20), 
            ( 95, 65, 20), 
            ( 95, 75, 20), 
            ( 95, 85, 20), 
            ( 95, 95, 20) ]

USER = (5, 5, 0)
# FIM CONSTANTES

# MODELAR PROBLEMA
model = cp_model.CpModel()
dist = 20

PL= 20*np.log10(dist) + 20*np.log10(fi) + 20*np.log10(4*np.pi/luz)

PR = np.power(10, (Pti + GTi + GRi - PL)/10)/1000

gamma = -np.log(5*BLER_eMBB)/0.45
c= Wi * np.log2(1 + PR/(Pnw * gamma))

# FIM MODELAR PROBLEMA 
# pi_til = model.NewBoolVar("pi_til")
ri = model.NewIntVar(0, Ri, "ri")
ri_til = model.NewBoolVar("ri_til")

# model.Add(pi_til == 1)
model.Add(ri <= Ri * ri_til)
model.Add(int(c)*ri >= T[0])

model.Minimize(Fi * ri_til)
# GERAL
solver = cp_model.CpSolver()
solver.parameters.log_search_progress = True
status = solver.Solve(model)

if status == cp_model.OPTIMAL:
    print(solver.Value(ri))
    print(solver.Value(ri_til))
    print(solver.Value(Fi))
# FIM GERAL
