from ortools.sat.python import cp_model
import math

# Variables
Wi = 1e6
Pti = 20
GTi = 0
GRi = 0
c = 3e8
Pn = 3.16e13
fi = 2.4e9
Ri = 20
BLER_eMBB = 1e-3
BLER_URLLC = 0.1
K_EMBB = -math.log(5*BLER_eMBB) / 0.45
K_URLLC = -math.log(5*BLER_URLLC) / 1.25
Ts = [6.5e6, 13e6]

UAV_possivel_pos = []
for i in range(5, 100, 5):
    for j in range(5, 100, 5):
        if i or j == 5 or 0:
            UAV_possivel_pos.append((i, j, 20))

eMBB_areas = [(45, 25, 0), (5, 45, 0), (55, 45, 0), (35, 75, 0), (45, 75, 0)]
URLLC_areas = [(65, 15, 0), (75, 15, 0), (15, 45, 0), (75, 55, 0), (25, 75, 0)]


# Get distance between the UAV and the center of the area
def distance(x1, y1, x2, y2, z1=0, z2=20):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)


# Get the path loss component to be used to get the received power (PRim)
def pathLossComponent(dim):
    return 20 * math.log10(dim)-20*math.log10(fi) + 20*math.log10(4*math.pi/c)


# Get the received power
def receivedPower(PLim):
    return ((10 ** ((Pti + GTi + GRi - PLim)/10))/1000)


# Get the Bidirectional network capacity provided by an RU of UAVi (Cim)
def networkCapacity(Prim, K):
    return Wi * math.log2(1 + (PRim / Pn * K))


def minimalUAVNumbers():
    model = cp_model.CpModel()

    for x1, y1, z1 in eMBB_areas + URLLC_areas:
        for x2, y2, z2 in UAV_possivel_pos:
            dim = distance(x1, y1, x2, y2, z1, z2)
            PLim = pathLossComponent(dim)
            PRim = receivedPower(PLim)




    # --- Constraints! ---
    # Add first contraint about "Number of RUs provided by UAVi"
    model.Add(rim <= Ri * ri)

    # Add second contraint about "one subarea being served by only 1 UAV"
    model.Add(Pim == 1)

    # Add third contraint about "Bidirectional network capacity provided by an RU of UAVi"
    model.Add(Cim * rim * Pim >= Ts)

    # Function to optimize
    #TODO add the damn function
    model.Minimize()

    solver = cp_model.Solver()
    status = solver.Solve(model)
    return status


#result = minimalUAVNumbers()

result = minimalUAVNumbers()
print(result)