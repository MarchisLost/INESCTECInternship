from ortools.sat.python import cp_model
import numpy as np


def minimalUAVNumbers():
    model = cp_model.CpModel()

    UAV_possivel_pos = []
    for i in range(5, 100, 5):
        for j in range(5, 100, 5):
            if i or j == 5 or 0:
                UAV_possivel_pos.append((i, j, 20))
    
    eMBB_areas = [(45, 25, 0), (5, 45, 0), (55, 45, 0), (35, 75, 0), (45, 75, 0)]
    URLLC_areas = [(65, 15, 0), (75, 15, 0), (15, 45, 0), (75, 55, 0), (25, 75, 0)]


    # Add first contraint about "Number of RUs provided by UAVi"
    ri = model.BoolVar('ri')
    Ri = 20
    model.Add(rim <= Ri * ri).OnlyEnforceIf(ri.Not())

    # Add second contraint about "one subarea being served by only 1 UAV"
    Pim = model.BoolVar(Pim)
    model.Add(Pim == 1)

    # Add third contraint about "Bidirectional network capacity provided by an RU of UAVi" eMBB
    TeMBB = 13000000
    model.Add(Cim * rim * Pim >= TeMBB)

    # Add third contraint about "Bidirectional network capacity provided by an RU of UAVi" URLLC
    TURLLC = 6500000
    model.Add(Cim * rim * Pim >= TURLLC)

    
# minimalUAVNumbers()