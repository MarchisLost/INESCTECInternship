import math
from numpy import *
from mip import Model, xsum, maximize, BINARY, INTEGER, OptimizationStatus
import time
import tracemalloc
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
N = 1000
BLER_eMBB = 0.1
BLER_URLLC = 1e-3
K_EMBB = -math.log(5*BLER_eMBB) / 0.45
K_URLLC = -math.log(5*BLER_URLLC) / 1.25
Ts = [int(6.5e6), int(13e6)]  # T[0] = URLLC; T[1] = eMBB
Todos_Ts = [6.5e6, 13e6, 19.5e6, 26e6, 39e6, 52e6, 58.5e6, 65e6, 78e6]
N_Areas = 10
N_UAV = 200
n_pessoas_area = [10, 7, 11, 4, 9, 4, 24, 25, 24, 6]

# If I is even it gets the eMBB slice, if odd gets the URLLC
d = dict()
I_number_areas = []
for i in range(N_Areas):
    if i % 2 == 0:
        d[i] = Ts[1]
    else:
        d[i] = Ts[0]
    I_number_areas.append(i)

# Even = eMBB; Odd = URLLC
areas = [(45, 25, 0), (65, 15, 0), (5, 45, 0), (75, 15, 0), (55, 45, 0), (15, 45, 0), (35, 75, 0), (75, 55, 0), (45, 75, 0), (25, 75, 0)]

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
# print(UAV_possivel_pos)

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


def minimalUAVNumbers():
    all_Cim = {}
    for x1, y1, z1 in areas:
        for x2, y2, z2 in UAV_possivel_pos:
            dim = distance(x1, y1, x2, y2, z1, z2)
            PLim = pathLossComponent(dim)
            PRim = receivedPower(PLim)
            # print(PRim)
            if areas.index((x1, y1, z1)) % 2 == 0:
                Cim = networkCapacity(PRim, K_EMBB)
                # print('Par:', Cim)
            else:
                Cim = networkCapacity(PRim, K_URLLC)
                # print('Impar:', Cim)

            # Dict with the keys as tuples with the index of the area and the index of the UAV and the value as the Cim (network capacity)
            all_Cim[areas.index((x1, y1, z1)), UAV_possivel_pos.index((x2, y2, z2))] = Cim
    # print(all_Cim)
    return all_Cim


def modelBuild(all_Cim):
    # --- Constraints! ---
    m = Model()

    ri_til = [m.add_var(var_type=BINARY) for j in J_number_UAV]
    rim = [[m.add_var(var_type=INTEGER) for j in J_number_UAV] for i in I_number_areas]
    Pim_til = [[m.add_var(var_type=BINARY) for j in J_number_UAV] for i in I_number_areas]

    # Add first contraint about "one subarea being served by only 1 UAV"
    for i in I_number_areas:
        m.add_constr(xsum(Pim_til[i][j] for j in J_number_UAV) == 1)

    # Add second contraint about "Number of RUs provided by UAVi"
    for j in J_number_UAV:
        m.add_constr(xsum(rim[i][j] for i in I_number_areas) <= Ri_UAV[j] * ri_til[j])

    T_escolhido = []
    while len(T_escolhido) != 2:
        t_aux = Todos_Ts[random.randint(0, 8)]
        if t_aux not in T_escolhido:
            T_escolhido.append(int(t_aux))
        else:
            continue

    # Add third contraint about "Bidirectional network capacity provided by an RU of UAVi"
    for i in I_number_areas:
        for k in J_number_UAV:
            if i % 2 == 0:
                m.add_constr(xsum(all_Cim[i, j] * rim[i][j] for j in J_number_UAV) >= Ts[1])  # * n_pessoas_area[i])
            else:
                m.add_constr(xsum(all_Cim[i, j] * rim[i][j] for j in J_number_UAV) >= Ts[0])  # * n_pessoas_area[i])
    for j in J_number_UAV:
        for i in I_number_areas:
            m.add_constr(rim[i][j] <= Ri_UAV[j] * Pim_til[i][j])

    # Function to optimize
    m.objective = xsum(Fi_UAV[j] * ri_til[j] for j in J_number_UAV)
    start_time = time.time()
    status = m.optimize(max_seconds=60)
    x = time.time() - start_time
    if status == OptimizationStatus.OPTIMAL:
        return xsum(Fi_UAV[j] * ri_til[j].x for j in J_number_UAV).x, x
    else:
        return -1, -1


tentativas = 10
t = 0
InfeasableOrNotOptimal = 0
for i in range(0, tentativas):
    result = minimalUAVNumbers()
    tracemalloc.start()
    aux, tempo = modelBuild(result)
    ###### Eficiencia de memoria #####
    current, peak = tracemalloc.get_traced_memory()
    print(f"Current memory usage is {current / 10**6}MB; Peak was {peak / 10**6}MB")
    tracemalloc.stop()
    ##################################
    if aux == -1:
        InfeasableOrNotOptimal = InfeasableOrNotOptimal + 1
        print("Não conseguiu tentativa %d/%d." % (i+1, tentativas))
    else:
        print("Conseguiu tentativa %d/%d" % (i+1, tentativas))
        print("Custo: %d" % aux)
        t = t + tempo
    areas = []
    while len(areas) != 10:
        a = 0
        a = todas_Areas[random.randint(0, 99)]
        if a not in areas:
            areas.append(a)
        else:
            continue

media = t / tentativas
print("Conseguiu: %d" % (tentativas - InfeasableOrNotOptimal))
print("Não conseguiu: %d" % InfeasableOrNotOptimal)
print("Taxa de sucesso: %f %%" % (((tentativas - InfeasableOrNotOptimal)/tentativas) * 100))
print("Media tempo = %f" % media)
