from pathlib import Path
import os
import networkx as nx
import numpy as np
import gurobipy as gp
from gurobipy import GRB
import time as trun
import igraph as ig
import math

def form_cordeau(method_,data_,out_path_,instance_):

    file_in = open(data_)
	
    line = file_in.readline().split()
    K = int(line[0]) # number of vehicles
    N = int(line[1]) # number of nodes
    maxRouDur = int(line[2]) # maximum route duration for vehicle, T, maximum duration of service
    vehCap = int(line[3]) # vehicles capacity, Q
    maxRidTime = int(line[4]) # maximum ride time of user, L
    
    # total duration of its route cannot exceed T_k
    
    print(K,N,maxRouDur,vehCap,maxRidTime)
 
    vehCapk = []  # vehicle capacity, Q_k
    for t in range(0,K):
        vehCapk.append(vehCap)

    maxRouDurk = []  # maximum route duration, T_k    
    for t in range(0,K):
        maxRouDurk.append(maxRouDur)
    
    maxRidTimei = []  # maximum ride times, L_i    
    for i in range(0,2*(N+1)):
        maxRidTimei.append(maxRidTime)

    idn = [] # id nodes, id
    cox = [] # coord x, cx
    coy = [] # coord y, cy 
    serv_time = [] # service_time, d_i, service duration
    load = [] # load, q_i
    start_tw = [] # start_tw, e_i 
    end_tw = [] # end_tw, l_i
    
    for t in range(0,2*(N+1)):
        line = file_in.readline().split()
        idn.append(int(line[0]))
        cox.append(float(line[1]))
        coy.append(float(line[2]))
        serv_time.append(int(line[3]))
        load.append(int(line[4]))
        start_tw.append(int(line[5]))
        end_tw.append(int(line[6]))
        
    #for t in range(0,2*(N+1)):
    #    print(idn[t]," ",cox[t]," ",coy[t]," ",serv_time[t]," ",load[t]," ",start_tw[t]," ",end_tw[t])
        
    file_in.close()

    route_cost = np.zeros((2*(N+1),2*(N+1)),dtype=float)
    travel_time = np.zeros((2*(N+1),2*(N+1)),dtype=float)
    for i in range(0,2*(N+1)):
        for j in range(0,2*(N+1)):
            #dist = np.linalg.norm(cox[i]-coy[j])
            dist = (cox[i]-cox[j])*(cox[i]-cox[j]) + (coy[i]-coy[j])*(coy[i]-coy[j])
            dist = math.sqrt(dist)
            #print(dist)
            route_cost[i,j] = dist
            travel_time[i,j] = dist

    time_win = np.zeros((2*(N+1)),dtype=float)   # time windows     
    for i in range(0,2*(N+1)):
        time_win[i] = end_tw[i] - start_tw[i]
        print(time_win[i])

    M = np.zeros((2*(N+1),2*(N+1),K), dtype=float)
    W = np.zeros((2*(N+1),2*(N+1),K), dtype=float)

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

    if (method_=="mip"):
        x = model.addVars(tuple0,vtype=GRB.BINARY,name="x")
    else:
        x = model.addVars(tuple0,lb=0.0,ub=1.0,vtype=GRB.CONTINUOUS,name="x")

    B = model.addVars(tuple1,lb=start_tw[t],ub=start_tw[t],vtype=GRB.CONTINUOUS,name="B")
    
    Q = model.addVars(tuple1,vtype=GRB.CONTINUOUS,name="Q")
    L = model.addVars(tuple1,vtype=GRB.CONTINUOUS,name="L")
    
    obj = 0
    for t in range(0,K):
        for i in range(0,2*(N+1)):
            for j in range(0,2*(N+1)):
                obj += route_cost[i][j]*x[i,j,t]
        
    model.setObjective(obj, GRB.MINIMIZE)
    
    # P = { 1, ..., N }
    # D = { N+1, ..., 2*N }
    # 0, 2*N+1
    
    # constraint 2
    for i in range(1,N+1):
        con = 0
        for t in range(0,K):
            for j in range(0,2*(N+1)):
                con += x[i,j,t]
        model.addConstr(con == 1)

    # constraint 3
    for i in range(1,N+1):
        for t in range(0,K):
            con = 0
            for j in range(0,2*(N+1)):
                con += x[i,j,t]
            for j in range(0,2*(N+1)):
                con -= x[N+i,j,t]
            model.addConstr(con == 0)

    # constraint 4
    for t in range(0,K):
        con = 0
        for j in range(0,2*(N+1)):
            con += x[0,j,t]
        model.addConstr(con == 1)

    # constraint 5
    for i in range(1,2*N+1):
        for t in range(0,K):
            con = 0
            for j in range(0,2*(N+1)):
                con += x[j,i,t]
            for j in range(0,2*(N+1)):
                con -= x[i,j,t]
            model.addConstr(con == 0)

    # constraint 6
    for t in range(0,K):
        con = 0
        for i in range(0,2*(N+1)):
            con += x[i,2*N+1,t]
        model.addConstr(con == 1)
        
    # constraint 7
    for i in range(0,2*(N+1)):
        for j in range(0,2*(N+1)):
            for t in range(0,K):
                model.addConstr(B[j,t] >= (B[i,t] + serv_time[i] + travel_time[i,j])*x[i,j,t])

    # constraint 8
    for i in range(0,2*(N+1)):
        for j in range(0,2*(N+1)):
            for t in range(0,K):
                model.addConstr(Q[j,t] >= (Q[i,t] + load[j])*x[i,j,t])

    # constraint 15
#    for i in range(0,2*(N+1)):
#        for j in range(0,2*(N+1)):
#            for t in range(0,K):
#                M[i,j,t] >= max(0,end_tw[i] + serv_time[i] + travel_time[i,j] - start_tw[j])

#    for i in range(0,2*(N+1)):
#        for j in range(0,2*(N+1)):
#            for t in range(0,K):
#                model.addConstr(
#                    B[j,t] >= B[i,t] + serv_time[i] + travel_time[i,j] - M[i,j,t]*(1 - x[i,j,t])
#                )

    # constraint 16
#    for i in range(0,2*(N+1)):
#        for j in range(0,2*(N+1)):
#            for t in range(0,K):
#                W[i,j,t] >= min(vehCapk[t], vehCapk[t] + load[t])

#    for i in range(0,2*(N+1)):
#        for j in range(0,2*(N+1)):
#            for t in range(0,K):
#                model.addConstr(vehCapk[j,t] <= vehCapk[i,t] + load[j] - W[i,j,t]*(1 - x[i,j,t]))
               
    # constraint 9
    for i in range(1,N+1):
        for t in range(0,K):
            model.addConstr(L[i,t] == B[N+i,t] - (B[i,t] + serv_time[i]))

    # constraint 10
    for t in range(0,K):
        model.addConstr(B[2*N+1,t] - B[0,t] <=  maxRouDurk[t])
    
    # constraint 11_1
    for i in range(0,2*(N+1)):
        for t in range(0,K):
            model.addConstr(B[i,t] >= start_tw[i])

    # constraint 11_2
    for i in range(0,2*(N+1)):
        for t in range(0,K):
            model.addConstr(B[i,t] <= end_tw[i])

    # constraint 12_1
    for i in range(1,N+1):
        for t in range(0,K):
            model.addConstr(L[i,t] >= travel_time[i,N+i])

    # constraint 12_2
    for i in range(1,N+1):
        for t in range(0,K):
            model.addConstr(L[i,t] <= maxRidTimei[i])

    # constraint 13_1
    for i in range(0,2*(N+1)):
        for t in range(0,K):
            model.addConstr(max(0,load[i]) <= Q[i,t])

    # constraint 13_1
    for i in range(0,2*(N+1)):
        for t in range(0,K):
            model.addConstr(Q[i,t] <= min(vehCapk[t],vehCapk[t] + load[i]))

    model.write(f"{data_}.lp")

    model.optimize()

    tmp = 0
    if model.status == GRB.OPTIMAL:
        tmp = 1
     
    if method_ == "mip":
        lb = model.objBound
        ub = model.objVal
        gap = model.MIPGap
        time = model.Runtime
        nodes = model.NodeCount
        status = tmp
    else:
        ub= model.objVal
        time = model.Runtime
        status = tmp
  
    if (method_=="mip"):
        arquivo = open(os.path.join(out_path_,instance_),'a')
        arquivo.write(
            str(tmp)+';'
            +str(round(lb,1))+';'
            +str(round(ub,1))+';'
            +str(round(gap,2))+';'
            +str(round(time,2))+';'
            +str(round(nodes,1))+';'
            +str(round(status,1))+'\n'
        )
        arquivo.close()
    else:
        arquivo = open(os.path.join(out_path_,instance_),'a')
        arquivo.write(
            str(tmp)+';'
            +str(round(ub[i],1))+';'
            +str(round(time[i],2))+';'
            +str(round(status[i],1))+'\n'
        )
        arquivo.close()
