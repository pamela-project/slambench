#!/usr/bin/python3
import math
import sys
import re
import numpy as np


import matplotlib
import pylab as plot
import datetime as  datetime
import os
import argparse
import pprint
from progress.bar import Bar

from utils import *
from slamlog import *
from plotutils import *





accuracy_threshold = 0.05




def select_all (data_array,i) :
    return True


def select_threshold(data_array,i) :
    ate_value = float( data_array["AbsoluteError"][i]) 
    return ate_value < accuracy_threshold

def select_twicethreshold(data_array,i) :
    ate_value = float( data_array["max_ATE"][i]) 
    return ate_value < 1.25*accuracy_threshold




    
def generate_data_array (data) :
    data_array = {
        "AbsoluteError"  : [],
        "Duration" : [],
        "Dataset" : [],
        "Library" : []
    }

    for filename,temp in data.items() :
        if (temp == None) :
            print (filename)
        
        data_array["AbsoluteError"].append(temp[SUMMARY_SECTION]["algo"]["AbsoluteError"][MEAN_FIELD])
        data_array["Duration"].append(temp[SUMMARY_SECTION]["algo"]["Duration_Frame"][MEAN_FIELD])
        data_array["Dataset"].append(temp[PROPERTIES_SECTION]["input"])
        data_array["Library"].append(temp[PROPERTIES_SECTION]["load-slam-library"])

    return data_array
            
            
def generate_accuracy_record (data) :
    strlog = ""
    for filename,temp in data.items() :
        
        count = len(temp[STATISTICS_SECTION]['algo-AbsoluteError'])
        
        if strlog == "" :
            strlog = " ".join ( [ ("ATE-%d RPE-%d" % (idx,idx)) for idx in range(count) ] ) + "\n"
        
        for idx in range(count) :
            strlog += ( "%f %f " % (   temp[STATISTICS_SECTION]['algo-AbsoluteError'][idx] ,  temp[STATISTICS_SECTION]['algo-RPE_RMSE'][idx]))
        strlog += ( "\n" )
    return strlog


def generate_duplicate_list (data, values, maxdup = 3) :
    strlog = ""
    duplications = {}
    bar = Bar('Loading', fill='@', suffix='%(percent)d%%', max=len(data.keys()))
    for filename,temp in data.items() :
        bar.next()
        hashkey = ""
        for v in values :
            hashkey += str(temp[PROPERTIES_SECTION][v]) + "@"
        if not hashkey in duplications :
            duplications[hashkey] = []
        duplications[hashkey].append(filename)
    bar.finish()
            
    for key,cases in duplications.items() :
        if len(cases) > maxdup :
            strlog += "%s : %s\n" %  ( key, str(cases))
    return strlog




#########################################################################################
#  MAIN
#########################################################################################


def main():       

    # GET ARGUMENTS
    parser = argparse.ArgumentParser()
    #parser.add_argument('-i','--input', action='append', help='<Required> Set flag', required=True)
    parser.add_argument('inputs', metavar='N', type=str, nargs='+',  help='Log/Directory/Summary to process')
    parser.add_argument('-v','--verbose', action='store_true', help="turn verbose on")
    parser.add_argument('-a','--accuracy', action='store_true', help="print accuracy record")
    parser.add_argument('-d','--duplicate', action='append', help="Look for duplicate")
    parser.add_argument('-s','--save', type=str, help="Save summary into summary file")
    parser.add_argument('-p','--plot', action='store_true', help="Plot the results")
    
    args = parser.parse_args()
    
    if args.verbose :
        setverbose(True)
    else :
        setverbose(False)


    data = load_inputs(args.inputs)

    if args.save :
        save_summary_file (data, args.save)
    
    if args.accuracy :
        sys.stdout.write(generate_accuracy_record (args.files))
     
    if args.duplicate :
        sys.stdout.write(generate_duplicate_list (data, args.duplicate))

    if args.plot :
        data_array = generate_data_array (data) 
        plot_data(
            data_array = data_array , coloring="Dataset", print_legend = False,  
            xelem="Duration"    ,  xoperator=operator.lt,  
            yelem="AbsoluteError",  yoperator=operator.lt,   filter_function = select_all ,
            filename="test.png" )
        
    
main()
