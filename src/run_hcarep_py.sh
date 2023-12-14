#!/bin/bash

method_=mip
form_=cordeau0

mkdir -p ../result
mkdir -p /home/jossian/Downloads/develop/report/hcarep/${form_}

#a2: 16 20 24
#a3: 18 24 30 36

for veh_ in a2
do
    for no_ in 16 20 24 
    do
        python3 darp.py ../instances/darp_bc/${veh_}-${no_}.txt ${method_} ${veh_}_${no_} ${form_} \
        >> /home/jossian/Downloads/develop/report/darp/${form_}/${method_}_${veh_}_${no_}_${form_}.txt
    done
done
