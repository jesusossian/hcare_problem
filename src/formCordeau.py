from pathlib import Path
import os
import networkx as nx
import numpy as np
import gurobipy as gp
from gurobipy import GRB
import time as trun
import igraph as ig
import math

def form_cordeau(method_,data_,out_path_,instance_,inst_,form_):

    file_in = open(data_)
    file_out = f"darp_{method_}_{form_}.txt"
	
    line = file_in.readline().split()
    K = int(line[0]) # number of vehicles
    N = int(line[1]) # number of nodes
    max_route_duration = int(line[2]) # maximum route duration for vehicle, T
    veh_cap = int(line[3]) # vehicles capacity, Q_k
    max_ride_time = int(line[4]) # maximum ride time of user, L
    
    # total duration of its route cannot exceed max_route_duration
    
    #print(K,N,max_route_duration,veh_cap,max_ride_time)
 
    Li = []  # maximum ride times, L_i    
    for i in range(0,2*(N+1)):
        Li.append(max_ride_time)

    idn = [] # id nodes, id
    cox = [] # coord x, cx
    coy = [] # coord y, cy 
    serv_time = [] # service_time, d_i, service duration
    load = [] # load, q_i, demand
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
    #    print(idn[t]," ")
    #    print(cox[t]," ")
    #    print(coy[t]," ")
    #    print(serv_time[t]," ")
    #    print(load[t]," ")
    #    print(start_tw[t]," ")
    #    print(end_tw[t])
        
    file_in.close()

    route_cost = np.zeros((2*(N+1),2*(N+1)),dtype=float)
    travel_time = np.zeros((2*(N+1),2*(N+1)),dtype=float)
    for i in range(0,2*N+1):
        for j in range(0,2*N+1):
            dist = (cox[i]-cox[j])*(cox[i]-cox[j]) + (coy[i]-coy[j])*(coy[i]-coy[j])
            dist = math.sqrt(dist)
            #print(dist)
            #dist = round(dist,2)
            route_cost[i,j] = dist
            travel_time[i,j] = dist
            #print(route_cost[i,j],end=" ")
        #print("\n")

    #time_win = np.zeros((2*(N+1)),dtype=float)   # time windows     
    #for i in range(0,2*(N+1)):
    #    time_win[i] = end_tw[i] - start_tw[i]
    #    print(time_win[i])

    model = gp.Model(f"{data_}")
        
    # configurando parametros
    model.Params.TimeLimit = 3600
    model.Params.MIPGap = 1.e-6
    model.Params.Threads = 1
    model.Params.Presolve = 0
    #model.Params.Cuts = 0
 
    # Turn off display and heuristics
    gp.setParam('OutputFlag', 0)
    #gp.setParam('Heuristics', 0)

    tuple3 = list(tuple())
    for i in range(0,2*(N+1)):
        for j in range(0,2*(N+1)):
            for k in range(0,K):
                tuple3.append((i,j,k))
    
    if (method_=="mip"):
        # x_{i,j,k} = 1 if and only if arc (i,j) is travesed by vehicle k \in K
        x = model.addVars(tuple3,lb=0.0,ub=1.0,vtype=GRB.BINARY,name="x")
    else:
        x = model.addVars(tuple3,lb=0.0,ub=1.0,vtype=GRB.CONTINUOUS,name="x")    

    # time at which vehicle k starts servicing at vertex i 
    B_ini = {}
    for k in range(0,K):
        B_ini[k] = model.addVar(vtype=GRB.CONTINUOUS,lb=start_tw[0],ub=end_tw[0], name="B_ini(%s)"%k)

    B_end = {}
    for k in range(0,K):
        B_end[k] = model.addVar(vtype=GRB.CONTINUOUS,lb=start_tw[0],ub=end_tw[0], name="B_end(%s)"%k)

    B = {}
    for i in range(1,2*N+1):
        B[i] = model.addVar(vtype=GRB.CONTINUOUS,lb=start_tw[i],ub=end_tw[i], name="B(%s)"%i)
        
    # ride time of use i (corresponding to request (i,n+i) on vehicle k)
    L = {}
    for i in range(1,N+1):
        L[i] = model.addVar(vtype=GRB.CONTINUOUS,lb=travel_time[i,N+i],ub=Li[i], name="L(%s)"%i)

    # the load of vehicle k upon leaving vertex i
    Q_ini = {}
    for k in range(0,K):
        Q_ini[k] = model.addVar(vtype=GRB.CONTINUOUS,lb=0,ub=veh_cap,name="Q_ini(%s)"%k)

    Q_end = {}
    for k in range(0,K):
        Q_end[k] = model.addVar(vtype=GRB.CONTINUOUS,lb=0,ub=veh_cap,name="Q_end(%s)"%k)

    Q = {}
    for i in range(1,2*N+1):
        Q[i] = model.addVar(vtype=GRB.CONTINUOUS,lb=max(0,load[i]),ub=min(veh_cap,veh_cap+load[i]),name="Q(%s)"%i)
    
    obj = 0
    for i in range(0,2*N+1):
        # j \in {0, 2n}
        for j in range(0,2*N+1):
            for k in range(0,K):
                obj += route_cost[i][j]*x[i,j,k]
        # j = 2n+1   
        for k in range(0,K):
            obj += route_cost[i,0] * x[i,2*N+1,k]
            
    # i = 2n+1
    # j \in {0, 2n}
    for j in range(0,2*N+1) :
        for k in range(0,K):
            obj += route_cost[0,j] * x[2*N+1,j,k]
        
    model.setObjective(obj, GRB.MINIMIZE)
    
    # P = { 1, ..., N }
    # D = { N+1, ..., 2*N }
    # 0, 2*N+1
    
    for k in range(0,K):
        # x_{i,i} = 0 for all i
        for i in range(0,2*N+2):
            model.addConstr(x[i,i,k] == 0)
            
        #// x_{i,0} = 0 for all i \in P U D and i = 2*n+1
        for i in range(1,2*N+2):
            model.addConstr(x[i,0,k] == 0)

        # x_{2*n+1,i} = 0 for all i \in P U D and i = 0
        for i in range(0,2*N+1):
            model.addConstr(x[2*N+1,i,k] == 0)

        # x_{i,2*n+1} = 0 for all i \in P 
        # x_{n+i,i} = 0 for all i \in P 
        for i in range(1,N+1):
            model.addConstr(x[i,2*N+1,k] == 0)
            model.addConstr(x[N+i,i,k] == 0)

        # x_{0,i} = 0 for all i \in D
        for i in range(N+1,2*N+1):
            model.addConstr(x[0,i,k] == 0)
            
    # pick_up 
    # each user is picked up by exactly one vehicle and then travels to exactly one different node
    for i in range(1,N+1):
        expr = 0
        for j in range(0,2*N+2):
            if (j != i):
                for k in range(0,K):
                    expr += x[i,j,k];
        model.addConstr(expr == 1,name = "pick_up")
                    
    # drop_off
    # everyone who is picked up must also be dropped off again (by the same vehicle)
    for k in range(0,K):
        for i in range(1,N+1):
        
            expr1 = 0
            for j in range(0,2*N+2):
                if (j!=i):
                    expr1 += x[i,j,k]
                    
            expr2 = 0
            for j in range(0,2*N+2):
                if (j!=N+i):
                    expr2 += x[N+i,j,k]
                    
            model.addConstr(expr1 - expr2 == 0, name = "drop_off")
    
    # leave_depot - every vehicle leaves the depot
    for k in range(0,K):
        expr = 0
        for j in range(1,2*N+2):
            expr += x[0,j,k];
        model.addConstr(expr == 1, name = "leave_depot")

    # flow preservation
    for k in range(0,K):
        for i in range(1,2*N+1):
            expr = 0
            for j in range(0,2*N+2):
                if (j!=i):
                    expr += x[j,i,k] - x[i,j,k]
            model.addConstr(expr == 0, name = "flow preservation")
            

    # return_depot_0
    for k in range(0,K):
        expr = 0
        for i in range(0,2*N+1):
            expr += x[i,2*N+1,k]
        model.addConstr(expr == 1, name = "return_depot_0")
        
    # return_depot_1
    for k in range(0,K):
        for j in range(1,2*N+1):
            expr = -B[j] + B_ini[k] + serv_time[0] + travel_time[0,j] - max(0, end_tw[0] + serv_time[0] + travel_time[0,j] - start_tw[j]) * (1 - x[0,j,k])
            model.addConstr(expr <= 0, name = "return_depot_1")

    # return_depot_2
    for i in range(1,2*N+1):
        for j in range(1,2*N+1):
            if (j!=i):
                expr2 = 0
                for k in range(0,K):
                    expr2 += x[i,j,k]                    
                expr = -B[j] + B[i] + serv_time[i] + travel_time[i,j] - max(0, end_tw[i] + serv_time[i] + travel_time[i,j] - start_tw[j]) * (1 - expr2)
                model.addConstr(expr <= 0, name = "return_depot_2")

    # return_depot_3
    for k in range(0,K):
        for i in range(1,2*N+1):
            expr = -B_end[k] + B[i] + serv_time[i] + travel_time[i,0] - max(0, end_tw[i] + serv_time[i] + travel_time[i,0] - start_tw[0]) * (1 - x[i,2*N+1,k])
            model.addConstr(expr <= 0, name = "return_depot_3")

    # return_depot_4
    for k in range(0,K):
        expr = -B_end[k] + B_ini[k] + serv_time[0] + travel_time[0,0] - max(0, end_tw[0] + serv_time[0] + travel_time[0,0] - start_tw[0]) * (1 - x[0,2*N+1,k])
        model.addConstr(expr <= 0, name = "return_depot_4")
        
    # ride time
    for i in range(1,N+1):
        expr = L[i] - B[N+i] + (B[i] + serv_time[i])
        model.addConstr(expr == 0, name = "ride_time")

    # capacity_0
    for k in range(0,K):
        for j in range(1,2*N+1):
            expr = -Q[j] + Q_ini[k] + load[j] - min(veh_cap, veh_cap + load[0]) * (1-x[0,j,k])
            model.addConstr(expr <= 0, name = "capacity_0")

    # capacity_1
    for i in range(1,2*N+1):
        for j in range(1,2*N+1):
            if (j!=i):
                expr2 = 0
                expr3 = 0
                for k in range(0,K):
                    expr2 += x[i,j,k]
                    expr3 += x[j,i,k]

                expr = -Q[j] + Q[i] + load[j] - min(veh_cap, veh_cap + load[i]) * (1 - expr2) + (min(veh_cap, veh_cap + load[i]) - load[i] - load[j]) * expr3
                model.addConstr(expr <= 0, name = "capacity_1")

    # capacity_2
    for k in range(0,K):
        for i in range(1,2*N+1):
            expr = -Q_end[k] + Q[i] + load[0] - min(veh_cap, veh_cap + load[i]) * (1 - x[i,2*N+1,k])
            model.addConstr(expr <= 0, name = "capacity_2")

    # capacity_3
    for k in range(0,K):
        expr = -Q_end[k] + Q_ini[k] + load[0] - min(veh_cap, veh_cap + load[0]) * (1 - x[0,2*N+1,k])
        model.addConstr(expr <= 0, name = "capacity_3")
        
    # maximum duration of vehicle tour
    for k in range(0,K):
        model.addConstr(B_end[k] - B_ini[k] <= max_route_duration, name = "max_route_duration_veh")                   

    model.write(f"lps/{inst_}.lp")

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
        arquivo = open(os.path.join(out_path_,file_out),'a')
        arquivo.write(
            str(inst_)+';'
            +str(round(lb,2))+';'
            +str(round(ub,2))+';'
            +str(round(gap,2))+';'
            +str(round(time,2))+';'
            +str(round(nodes,1))+';'
            +str(round(status,1))+'\n'
        )
        arquivo.close()
    else:
        arquivo = open(os.path.join(out_path_,file_out),'a')
        arquivo.write(
            str(tmp)+';'
            +str(round(ub[i],2))+';'
            +str(round(time[i],2))+';'
            +str(round(status[i],2))+'\n'
        )
        arquivo.close()
