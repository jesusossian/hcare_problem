import urllib.request
import io
import zipfile

from pathlib import Path
import os

import matplotlib.pyplot as plt
import networkx as nx
import numpy as np
import pandas as pd
import gurobipy as gp
from gurobipy import GRB

import igraph as ig

from itertools import combinations

import sys
from datetime import datetime, date

import formGaul as gaul
import formCordeau0 as cordeau0
import formCordeau1 as cordeau1
import formCordeau2 as cordeau2

if __name__ == "__main__":

    if len(sys.argv) < 1:
        print("digite dados!")
    else:
        data_ = sys.argv[1]
        method_ = sys.argv[2]
        inst_ = sys.argv[3]
        form_ = sys.argv[4]
        

    out_path_ = Path(f"../result")
    instance_ = f"{method_}_{form_}_{inst_}.txt"
	
    if form_ == "cordeau0":
        cordeau0.form_cordeau(method_,data_,out_path_,instance_,inst_,form_)
    elif form_ == "cordeau1":
        cordeau1.form_cordeau(method_,data_,out_path_,instance_)
    elif form_ == "cordeau2":
        cordeau2.form_cordeau(method_,data_,out_path_,instance_)
    else:
    	print("parameters errado!")
