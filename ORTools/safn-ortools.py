from ortools.sat.python import cp_model
import math
import random
import tracemalloc
import time
import numpy as np
import matplotlib.pyplot as plt
# Variables
Fi = 80
Wi = 1e6
Pti = 20
GTi = 0
GRi = 0
c = 3e8
Pn = 3.16e-13
fi = 2.4e9
Ri = 20
BLER_eMBB = 0.1
BLER_URLLC = 1e-3
K_EMBB = -math.log(5*BLER_eMBB) / 0.45
K_URLLC = -math.log(5*BLER_URLLC) / 1.25

Todos_Ts = [6.5e6, 13e6, 19.5e6, 26e6, 39e6, 52e6, 58.5e6, 65e6, 78e6]
T_escolhido = []
N_Areas = 10
N_UAV = 100
n_pessoas_area = [3, 7, 4, 4, 9, 4, 5, 2, 5, 6]
pos_uav_used = []

todas_Areas = []
for i in range(5, 100, 10):
    for j in range(5, 100, 10):
        todas_Areas.append((i, j, 0))
UAV_possivel_pos = []
for i in range(5, 100, 10):
    for j in range(5, 100, 10):
        UAV_possivel_pos.append((i, j, 10))
for i in range(5, 100, 10):
    for j in range(5, 100, 10):
        UAV_possivel_pos.append((i, j, 20))

Fi_UAV = []
Ri_UAV = []
J_number_UAV = []
for i in range(N_UAV):
    J_number_UAV.append(i)
    if i < 100:
        Fi_UAV.append(80)
        Ri_UAV.append(20)
    else:
        Fi_UAV.append(120)
        Ri_UAV.append(40)


# Get distance between the UAV and the center of the area
def distance(x1, y1, x2, y2, z1=0, z2=20):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2)


# Get the path loss component to be used to get the received power (PRim)
def pathLossComponent(dim):
    return 20 * math.log10(dim) + 20*math.log10(fi) + 20*math.log10(4*math.pi/c)


# Get the received power
def receivedPower(PLim):
    return ((10 ** ((Pti + GTi + GRi - PLim)/10))/1000)


# Get the Bidirectional network capacity provided by an RU of UAVi (Cim)
def networkCapacity(PRim, K):
    return Wi * math.log2(1 + (PRim / Pn / K))


def calculateNetworkBidirectionalCapacity(areas):
    all_Cim = {}
    k = 0
    for x1, y1, z1 in areas:
        for x2, y2, z2 in UAV_possivel_pos:
            dim = distance(x1, y1, x2, y2, z1, z2)
            PLim = pathLossComponent(dim)
            PRim = receivedPower(PLim)

            # The smaller value of T goes to the eMBB
            if d[k] <= T_escolhido[0]:
                if d[k] <= T_escolhido[1]:
                    Cim = networkCapacity(PRim, K_URLLC)
                else:
                    Cim = networkCapacity(PRim, K_EMBB)
            elif d[k] > T_escolhido[0]:
                Cim = networkCapacity(PRim, K_EMBB)

            # Dict with the keys as tuples with the index of the area and the index of the UAV and the value as the Cim (network capacity)
            all_Cim[areas.index((x1, y1, z1)), UAV_possivel_pos.index((x2, y2, z2))] = Cim
        k += 1
    return all_Cim


def chooseAreasAndTvalue():
    # Choose N_Areas random areas
    areas = []
    while len(areas) != N_Areas:
        a = todas_Areas[random.randint(0, 99)]
        if a not in areas:
            areas.append(a)
        else:
            continue
    # Choose slices for each area
    while len(T_escolhido) != 2:
        t_aux = Todos_Ts[random.randint(0, 8)]
        if t_aux not in T_escolhido:
            T_escolhido.append(int(t_aux))
        else:
            continue
    d = dict()
    I_number_areas = []
    tipo_area = []
    for i in range(N_Areas):
        d[i] = T_escolhido[random.randint(0, 1)]
        if d[i] == max(T_escolhido):
            tipo_area.append("emdbb")
        else:
            tipo_area.append("urcll")
        I_number_areas.append(i)
    return areas, d, I_number_areas, T_escolhido, tipo_area


def modelBuild(all_Cim, T_escolhido, d, I_number_areas):
    # --- Constraints! ---
    start_time = time.time()
    model = cp_model.CpModel()
    ri_til, Pim_til, rim = {}, {}, {}
    for j in J_number_UAV:
        ri_til[j] = model.NewBoolVar('ri_til[%d]' % j)
        for i in I_number_areas:
            rim[i, j] = model.NewIntVar(0, Ri_UAV[j], 'rim[%d, %d]' % (i, j))
            Pim_til[i, j] = model.NewBoolVar('Pim[%d, %d]' % (i, j))

    # Add first contraint about "Number of RUs provided by UAVi"
    for j in J_number_UAV:
        model.Add(sum(rim[i, j] for i in I_number_areas) <= Ri_UAV[j] * ri_til[j])

    # Add second contraint about "one subarea being served by only 1 UAV"
    for i in I_number_areas:
        model.Add(sum(Pim_til[i, j] for j in J_number_UAV) == 1)

    # Add third contraint about "Bidirectional network capacity provided by an RU of UAVi"
    for i in I_number_areas:
        model.Add(sum(int(all_Cim[i, j]) * rim[i, j] for j in J_number_UAV) >= d[i])  # n_pessoas_area[i])

    for i in I_number_areas:
        for j in J_number_UAV:
            model.Add(rim[i, j] == 0).OnlyEnforceIf(Pim_til[i, j].Not())
    # Function to optimize
    model.Minimize(sum(Fi_UAV[j] * ri_til[j] for j in J_number_UAV))

    solver = cp_model.CpSolver()
    solver.parameters.log_search_progress = False
    start_time = time.time()
    status = solver.Solve(model)
    x = time.time() - start_time
    links = []
    if status == cp_model.OPTIMAL:
        for i in I_number_areas:
            for j in J_number_UAV:
                if solver.Value(rim[i, j]):
                    print("RIs fornecidos pelo UAV %d " % j + str(UAV_possivel_pos[j]) + " ?? sub??rea %d " % i + str(areas[i]) + ": %d" % solver.Value(rim[i, j]))
                    pos_uav_used.append(UAV_possivel_pos[j])
                    links.append([UAV_possivel_pos[j], areas[i], solver.Value(rim[i, j])])
        for j in J_number_UAV:
            if solver.Value(ri_til[j]):
                print("ri_til[%d]: %d" % (j, solver.Value(ri_til[j])))
        print("Custo total: %d" % sum(solver.Value(Fi_UAV[j]) * solver.Value(ri_til[j]) for j in J_number_UAV))
        return x, sum(solver.Value(Fi_UAV[j]) * solver.Value(ri_til[j]) for j in J_number_UAV), links
    else:
        return -1, -1


areas, d, I_number_areas, T_escolhido, tipo_area = chooseAreasAndTvalue()
cim = calculateNetworkBidirectionalCapacity(areas)
tracemalloc.start()
Time_to_solve, custo, links = modelBuild(cim, areas, d, I_number_areas)
# MEMORY EFFICIENCY
current, peak = tracemalloc.get_traced_memory()
tracemalloc.stop()
print("Tempo           : %fs" % Time_to_solve)
print("Memoria de pico : %fMB" % (peak / 10**6))

# Plot
fig = plt.figure(figsize=(12, 7))

# Plot the areas
x_area_embb, y_area_embb, x_area_urcll, y_area_urcll = [], [], [], []
for i in range(len(areas)):
    if d[i] <= T_escolhido[0]:
        if d[i] <= T_escolhido[1]:
            x_area_urcll.append(areas[i][0])
            y_area_urcll.append(areas[i][1])
            value_urcll = d[i]
        else:
            x_area_embb.append(areas[i][0])
            y_area_embb.append(areas[i][1])
            value_embb = d[i]
    elif d[i] > T_escolhido[0]:
        x_area_embb.append(areas[i][0])
        y_area_embb.append(areas[i][1])
        value_embb = d[i]

ax = fig.add_subplot(1, 2, 1)
ax.plot(x_area_embb, y_area_embb, marker='s', linestyle='None', label='eMBB = ' + str(value_embb/10**6) + 'MB/s')
ax.plot(x_area_urcll, y_area_urcll, marker='o', linestyle='None', label='URCLL = ' + str(value_urcll/10**6) + 'MB/s')
ax.set_title("Slice-aware Coverage")
ax.set_xlabel("Position X of area (m)")
ax.set_ylabel("Position Y of area (m)")
ax.set_xticks(np.arange(5, 96, 10))
ax.set_yticks(np.arange(5, 96, 10))
ax.legend()
ax.grid()

# Plot the UAV positions
x_uav, y_uav, z_uav = [], [], []
for i in range(len(pos_uav_used)):
    x_uav.append(pos_uav_used[i][0])
    y_uav.append(pos_uav_used[i][1])
    z_uav.append(pos_uav_used[i][2])

ax = fig.add_subplot(1, 2, 2, projection='3d')
ax.plot(x_uav, y_uav, z_uav, marker='^', linestyle='None', label='Position 3D of UAV', c='m', markersize=15)
# ax.plot(x_uav, y_uav, marker='v', linestyle='None', label='Position 2D UAV', c = 'g')
for i in range(len(x_area_embb)):
    ax.plot(x_area_embb, y_area_embb, 0, marker='s', color='b', linestyle='None')
    ax.plot(x_area_urcll, y_area_urcll, 0, marker='.', color='Orange', linestyle='None')

for l in range(len(links)):
    if tipo_area[l] == "emdbb":
        ax.plot([links[l][0][0], links[l][1][0]], [links[l][0][1], links[l][1][1]], [links[l][0][2], links[l][1][2]], c='black', linestyle='-.', linewidth=links[l][2]/2)
    else:
        ax.plot([links[l][0][0], links[l][1][0]], [links[l][0][1], links[l][1][1]], [links[l][0][2], links[l][1][2]], c='b', linestyle='-.', linewidth=links[l][2]/2)
ax.set_title("Position of UAVs")
ax.set_xlabel("Position X of UAV (m)")
ax.set_ylabel("Position Y of UAV (m)")
ax.set_zlabel("Position Z of UAV (m)")
ax.set_xticks(np.arange(5, 96, 10))
ax.set_yticks(np.arange(5, 96, 10))
ax.set_zticks(np.arange(0, 30, 10))
ax.legend()
ax.view_init(20, 30)
plt.grid()
plt.show()
