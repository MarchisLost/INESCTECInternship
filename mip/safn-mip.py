import math
from mip import Model, xsum, BINARY, INTEGER
import tracemalloc
import time
import random
import matplotlib.pyplot as plt
import numpy as np

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
N_Areas = 20
N_UAV = 200
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
    i = 0
    for x1, y1, z1 in areas:
        for x2, y2, z2 in UAV_possivel_pos:
            dim = distance(x1, y1, x2, y2, z1, z2)
            PLim = pathLossComponent(dim)
            PRim = receivedPower(PLim)

            # The smaller value of T goes to the eMBB
            if d[i] <= T_escolhido[0]:
                if d[i] < T_escolhido[1]:
                    Cim = networkCapacity(PRim, K_EMBB)
                else:
                    Cim = networkCapacity(PRim, K_URLLC)
                    Cim = networkCapacity(PRim, K_URLLC)
            elif d[i] > T_escolhido[0]:
                Cim = networkCapacity(PRim, K_URLLC)
                Cim = networkCapacity(PRim, K_URLLC)

            # Dict with the keys as tuples with the index of the area and the index of the UAV and the value as the Cim (network capacity)
            all_Cim[areas.index((x1, y1, z1)), UAV_possivel_pos.index((x2, y2, z2))] = Cim
        i += 1
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
    print('Tescolhido:', T_escolhido)
    d = dict()
    I_number_areas = []
    for i in range(N_Areas):
        d[i] = T_escolhido[random.randint(0, 1)]
        I_number_areas.append(i)
    print('d:', d)
    return areas, d, I_number_areas, T_escolhido


def modelBuild(all_Cim, T_escolhido, d, I_number_areas):
    # --- Constraints! ---
    m = Model()
    start_time = time.time()
    ri_til = [m.add_var(var_type=BINARY) for j in J_number_UAV]
    rim = [[m.add_var(var_type=INTEGER) for j in J_number_UAV] for i in I_number_areas]
    Pim_til = [[m.add_var(var_type=BINARY) for j in J_number_UAV] for i in I_number_areas]

    # Add first contraint about "one subarea being served by only 1 UAV"
    for i in I_number_areas:
        m.add_constr(xsum(Pim_til[i][j] for j in J_number_UAV) == 1)

    # Add second contraint about "Number of RUs provided by UAVi"
    for j in J_number_UAV:
        m.add_constr(xsum(rim[i][j] for i in I_number_areas) <= Ri_UAV[j] * ri_til[j])
        for i in I_number_areas:
            m.add_constr(rim[i][j] <= Ri_UAV[j] * Pim_til[i][j])
    # Add third contraint about "Bidirectional network capacity provided by an RU of UAVi"
    for i in I_number_areas:
        m.add_constr(xsum(int(all_Cim[i, j]) * rim[i][j] for j in J_number_UAV) >= d[i])  # * n_pessoas_area[i])

    # Function to optimize
    m.objective = xsum(Fi_UAV[j] * ri_til[j] for j in J_number_UAV)
    start_time = time.time()
    m.optimize()
    x = time.time() - start_time
    for i in I_number_areas:
        for j in J_number_UAV:
            if rim[i][j].x:
                print("RIs fornecidos pelo UAV %d " % j + str(UAV_possivel_pos[j]) + " à subárea %d " % i + str(areas[i]) + ": %d" % rim[i][j].x)
                pos_uav_used.append(UAV_possivel_pos[j])
    for j in J_number_UAV:
        if ri_til[j].x:
            print("ri_til[%d]: %d" % (j, ri_til[j].x))

    return xsum(Fi_UAV[j] * ri_til[j].x for j in J_number_UAV).x, x


areas, d, I_number_areas, T_escolhido = chooseAreasAndTvalue()
cim = calculateNetworkBidirectionalCapacity(areas)
tracemalloc.start()
custo, x = modelBuild(cim, areas, d, I_number_areas)
# MEMORY EFFICIENCY
print("Custo total: %d" % custo)
print("Tempo      : %fs" % x)
current, peak = tracemalloc.get_traced_memory()
print(f"Current memory usage is {current / 10**6}MB; Peak was {peak / 10**6}MB")
tracemalloc.stop()

# Plot
fig = plt.figure(figsize=(12, 7))

# Plot the areas
x_area_embb, y_area_embb, x_area_urcll, y_area_urcll = [], [], [], []
for i in range(len(areas)):
    if d[i] <= T_escolhido[0]:
        if d[i] < T_escolhido[1]:
            x_area_embb.append(areas[i][0])
            y_area_embb.append(areas[i][1])
        else:
            x_area_urcll.append(areas[i][0])
            y_area_urcll.append(areas[i][1])
    elif d[i] > T_escolhido[0]:
        x_area_urcll.append(areas[i][0])
        y_area_urcll.append(areas[i][1])

ax = fig.add_subplot(1, 2, 1)
ax.plot(x_area_embb, y_area_embb, marker='s', linestyle='None', label='eMBB')
ax.plot(x_area_urcll, y_area_urcll, marker='o', linestyle='None', label='URCLL')
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
ax.plot(x_uav, y_uav, z_uav, marker='s', linestyle='None', label='Position 3D of UAV')
ax.plot(x_uav, y_uav, marker='.', linestyle='None', label='Position 2D UAV')
ax.set_title("Position of UAVs")
ax.set_xlabel("Position X of UAV (m)")
ax.set_ylabel("Position Y of UAV (m)")
ax.set_zlabel("Position Z of UAV (m)")
ax.set_xticks(np.arange(5, 96, 10))
ax.set_yticks(np.arange(5, 96, 10))
ax.set_zticks(np.arange(0, 30, 10))
ax.legend()
plt.grid()
plt.show()
