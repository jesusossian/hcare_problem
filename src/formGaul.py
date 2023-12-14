from pathlib import Path
import os
import networkx as nx
import numpy as np
import gurobipy as gp
from gurobipy import GRB
import time as trun
import igraph as ig

def form_cordeau(method_,data_,out_path_,instance_):

    file_in = open(data_)
	
    line = file_in.readline().split()
    K = int(line[0]) # number of vehicles
    N = int(line[1]) # number of nodes
    MRD = int(line[2]) # maximum route duration for vehicle, T
    VehCap = int(line[3]) # vehicles capacity, Q
    MRT = int(line[4]) # maximum ride time of user
    
    # total duration of its route cannot exceed T_k
    
    print(K,N,MRD,VehCap,MRT)
 
    lst_VehCap = []  # capacity of vehicle, Q_k
    for t in range(0,K):
        lst_VehCap.append(VehCap)

    lst_MRD = []  # maximum ride times, T_k    
    for t in range(0,K):
        lst_MRD.append(MRD)
    
    lst_MRT = []  # maximum ride times, L_i    
    for i in range(0,2*(N+1)):
        lst_MRT.append(MRT)

    lst_id = [] # id nodes, id
    lst_x = [] # coord x, cx
    lst_y = [] # coord y, cy 
    lst_st = [] # service_time,  q_i
    lst_load = [] # load, d_i
    lst_stw = [] # start_tw, e_i 
    lst_etw = [] # end_tw, l_i
    lst_tw = [] # tw = end_tw - start_tw 
    
    for t in range(0,2*(N+1)):
        line = file_in.readline().split()
        lst_id.append(int(line[0]))
        lst_x.append(float(line[1]))
        lst_y.append(float(line[2]))
        lst_st.append(int(line[3]))
        lst_demand.append(int(line[4]))
        lst_stw.append(int(line[5]))
        lst_etw.append(int(line[6]))
        
    for t in range(0,2*(N+1)):
        print(lst_id[t]," ",lst_x[t]," ",lst_y[t]," ",lst_st[t]," ",lst_demand[t]," ",lst_stw[t]," ",lst_etw[t])
        
    file_in.close()
    
    for i in range(1,2*N+1):
        lst_tw = lst_etw[i] - lst_stw[i]
    
    # check if requested load is greater than the vehicle capacity
    for i in range(1,N+1):
        if lst_demand[i] > VehCap:
            print("Problem instance is infeasible due to excess load")
    for i in range(N+1,2*N+1):
        if lst_demand[i] < -VehCap:
            print("Problem instance is infeasible due to excess load")


    cost = np.zeros((2*(N+1),2*(N+1)),dtype=float)
    ctim = np.zeros((2*(N+1),2*(N+1)),dtype=float)
    for i in range(0,2*(N+1)):
        for j in range(0,2*(N+1)):
            dist = (lst_x[i]-lst_x[j])*(lst_x[i]-lst_x[j]) + (lst_y[i]-lst_y[j])*(lst_y[i]-lst_y[j])
            dist = math.sqrt(dist)
            rcost[i,j] = dist
            ttime[i,j] = dist

    #M = np.zeros((2*(N+1),2*(N+1),K), dtype=float)
    #W = np.zeros((2*(N+1),2*(N+1),K), dtype=float)

    model = gp.Model(f"{data_}")
        
    # configurando parametros
    model.Params.TimeLimit = 3600
    model.Params.MIPGap = 1.e-6
    model.Params.Threads = 1
    model.Params.Presolve = 0
    #model.Params.Cuts = 0
 
    # Turn off display and heuristics
    #gp.setParam('OutputFlag', 0)
    #gp.setParam('Heuristics', 0)

    tuple0 = list(tuple())
    for t in range(0,K):
        for i in range(0,2*(N+1)):
            for j in range(0,2*(N+1)):
                tuple0.append((i,j,t))

    tuple1 = list(tuple())
    for t in range(0,K):
        for i in range(0,2*(N+1)):
            tuple1.append((i,t))

#    if (method_=="mip"):
#        x = model.addVars(tuple0,vtype=GRB.BINARY,name="x")
#    else:
#        x = model.addVars(tuple0,lb=0.0,ub=1.0,vtype=GRB.CONTINUOUS,name="x")

#    B = model.addVars(tuple1,vtype=GRB.CONTINUOUS,name="B")
#    vQ = model.addVars(tuple1,vtype=GRB.CONTINUOUS,name="Q")
#    vL = model.addVars(tuple1,vtype=GRB.CONTINUOUS,name="L")
    
#    obj = 0
#    for t in range(0,K):
#        for i in range(0,2*(N+1)):
#            for j in range(0,2*(N+1)):
#                obj += cost[i][j]*x[i,j,t]
            
#    model.setObjective(obj, GRB.MINIMIZE)
    
    # P = {1, ..., N }
    # D = {N+1, ..., 2*N }
    # 0, 2*N+1
    
    # constraint 2
#    for i in range(1,N):
#        con = 0
#        for t in range(0,K):
#            for j in range(0,2*(N+1)):
#                con += x[i,j,t]
#        model.addConstr(con == 1)

    # constraint 3
#    for i in range(1,N):
#        for t in range(0,K):
#            con = 0
#            for j in range(0,2*(N+1)):
#                con += x[i,j,t]
#            for j in range(0,2*(N+1)):
#                con -= x[N+i,j,t]
#            model.addConstr(con == 0)

    # constraint 4
#    for t in range(0,K):
#        con = 0
#        for j in range(0,2*(N+1)):
#            con += x[0,j,t]
#        model.addConstr(con == 1)

    # constraint 5
#    for i in range(1,2*N+1):
#        for t in range(0,K):
#            con = 0
#            for j in range(0,2*(N+1)):
#                con += x[j,i,t]
#            for j in range(0,2*(N+1)):
#                con -= x[i,j,t]
#            model.addConstr(con == 0)

    # constraint 6
#    for t in range(0,K):
#        con = 0
#        for i in range(0,2*(N+1)):
#            con += x[i,2*N+1,t]
#        model.addConstr(con == 1)
        
    # constraint 7
    #for i in range(0,2*(N+1)):
    #    for j in range(0,2*(N+1)):
    #        for t in range(0,K):
    #            model.addConstr(B[j,t] <= (B[i,t] + lst_demand[i] + ctim[i,j] )*x[i,j,t])

    # constraint 8
    #for i in range(0,2*(N+1)):
    #    for j in range(0,2*(N+1)):
    #        for t in range(0,K):
    #            model.addConstr(vQ[j,t] <= (vQ[i,t] + lst_st[j] )*x[i,j,t])

    # constraint 15
#    for i in range(0,2*(N+1)):
#        for j in range(0,2*(N+1)):
#            for t in range(0,K):
#                M[i,j,t] >= max(0,lst_etw[i] + lst_demand[i] + ctim[i,j] - lst_stw[j])

#    for i in range(0,2*(N+1)):
#        for j in range(0,2*(N+1)):
#            for t in range(0,K):
#                model.addConstr(B[j,t] >= B[i,t] + lst_demand[i] + ctim[i,j] - M[i,j,t]*(1 - x[i,j,t]))

    # constraint 16
#    for i in range(0,2*(N+1)):
#        for j in range(0,2*(N+1)):
#            for t in range(0,K):
#                W[i,j,t] >= min(lst_Q[t], lst_Q[t] + lst_st[t])

#    for i in range(0,2*(N+1)):
#        for j in range(0,2*(N+1)):
#            for t in range(0,K):
#                model.addConstr(vQ[j,t] <= vQ[i,t] + lst_st[j] - W[i,j,t]*(1-x[i,j,t]))
               
    # constraint 9
#    for i in range(1,N):
#        for t in range(0,K):
#            model.addConstr(vL[i,t] == B[N+i,t] - (B[i,t] + lst_demand[i]) )

    # constraint 10
#    for t in range(0,K):
#        model.addConstr(B[2*N+1,t] - B[0,t] <=  lst_MRD[t])
    
    # constraint 11_1
#    for i in range(0,2*(N+1)):
#        for t in range(0,K):
#            model.addConstr(B[i,t] >= lst_stw[i])

    # constraint 11_2
#    for i in range(0,2*(N+1)):
#        for t in range(0,K):
#            model.addConstr(B[i,t] <= lst_etw[i])

    # constraint 12_1
#    for i in range(1,N):
#        for t in range(0,K):
#            model.addConstr(vL[i,t] >= ctim[i,N+i])

    # constraint 12_2
#    for i in range(1,N):
#        for t in range(0,K):
#            model.addConstr(vL[i,t] <= MRT)

    # constraint 13_1
#    for i in range(0,2*(N+1)):
#        for t in range(0,K):
#            model.addConstr(max(0,lst_st[i]) <= vQ[i,t])

    # constraint 13_1
#    for i in range(0,2*(N+1)):
#        for t in range(0,K):
#            model.addConstr(vQ[i,t] <= min(lst_Q[t],lst_Q[t]+lst_st[i]))

#    model.write(f"{data_}.lp")

#    model.optimize()

#    tmp = 0
#    if model.status == GRB.OPTIMAL:
#        tmp = 1
     
#    if method_ == "mip":
#        lb = model.objBound
#        ub = model.objVal
#        gap = model.MIPGap
#        time = model.Runtime
#        nodes = model.NodeCount
#        status = tmp
#    else:
#        ub= model.objVal
#        time = model.Runtime
#        status = tmp
  
#    if (method_=="mip"):
#        arquivo = open(os.path.join(out_path_,instance_),'a')
#        arquivo.write(
#            str(tmp)+';'
#            +str(round(lb,1))+';'
#            +str(round(ub,1))+';'
#            +str(round(gap,2))+';'
#            +str(round(time,2))+';'
#            +str(round(nodes,1))+';'
#            +str(round(status,1))+'\n'
#        )
#        arquivo.close()
#    else:
#        arquivo = open(os.path.join(out_path_,instance_),'a')
#        arquivo.write(
#            str(tmp)+';'
#            +str(round(ub[i],1))+';'
#            +str(round(time[i],2))+';'
#            +str(round(status[i],1))+'\n'
#        )
#        arquivo.close()
