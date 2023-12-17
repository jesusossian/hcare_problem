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

import formCordeau as cordeau

if __name__ == "__main__":

    if len(sys.argv) < 1:
        print("digite dados!")
    else:
        data_ = sys.argv[1]
        method_ = sys.argv[2]
        inst_ = sys.argv[3]
        form_ = sys.argv[4]
        

    out_path_ = Path(f"/home/jossian/repository/hcare_problem/result")
    instance_ = f"{method_}_{form_}_{inst_}.txt"
	
    if form_ == "cordeau":
        cordeau.form_cordeau(method_,data_,out_path_,instance_,inst_,form_)
    else:
    	print("parameters errado!")
