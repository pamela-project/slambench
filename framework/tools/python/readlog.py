#!/usr/bin/python
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




def generate_accuracy_record (logfiles) :
    strlog = ""
    for filename in logfiles :
        
        start_time = time.time()
        temp = load_data_from_file(filename)
        load_time = (time.time() - start_time) * 1000
        
        count = len(temp[STATISTICS_SECTION]['algo-AbsoluteError'])
        
        if strlog == "" :
            strlog = " ".join ( [ ("ATE-%d RPE-%d" % (idx,idx)) for idx in xrange(count) ] ) + "\n"
        
        for idx in xrange(count) :
            strlog += ( "%f %f " % (   temp[STATISTICS_SECTION]['algo-AbsoluteError'][idx] ,  temp[STATISTICS_SECTION]['algo-RPE_RMSE'][idx]))
        strlog += ( "\n" )
    return strlog


def generate_duplicate_list (logfiles, values) :
    strlog = ""
    all_files    = {}
    duplications = {}
    bar = Bar('Processing', max=len(logfiles))
    bar = Bar('Loading', fill='@', suffix='%(percent)d%%', max=len(logfiles))
    for filename in logfiles :
        bar.next()
        temp = load_data_from_file(filename)
        hashkey = ""
        for v in values :
            hashkey += str(temp[PROPERTIES_SECTION][v]) + "@"
        all_files[filename] = temp
        if not hashkey in duplications :
            duplications[hashkey] = []
        duplications[hashkey].append(filename)
    bar.finish()
            
    for key,cases in duplications.items() :
        if len(cases) > 3 :
            strlog += "%s : %s\n" %  ( key, str(cases))
    return strlog




#########################################################################################
#  MAIN
#########################################################################################


def main():       

    # GET ARGUMENTS
    parser = argparse.ArgumentParser()
    #parser.add_argument('-i','--input', action='append', help='<Required> Set flag', required=True)
    parser.add_argument('files', metavar='N', type=str, nargs='+',  help='Files to process')
    parser.add_argument('-v','--verbose', action='store_true', help="turn verbose on")
    parser.add_argument('-a','--accuracy', action='store_true', help="print accuracy record")
    parser.add_argument('-d','--duplicate', action='append', help="Look for duplicate")
    
    args = parser.parse_args()
    if args.verbose :
        setverbose(True)
    else :
        setverbose(False)
    
    
    if args.accuracy :
        sys.stdout.write(generate_accuracy_record (args.files))
     
    if len(args.duplicate) > 0 :
        sys.stdout.write(generate_duplicate_list (args.files, args.duplicate))

        
    
main()
