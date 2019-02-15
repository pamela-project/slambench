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


from slamlog import *
from plotutils import *




#########################################################################################
# rendering setting
#########################################################################################




default_order = [
           "KF-CUDA",
           "EF-CUDA",
           "IT-CPP",
           "OS2-CPP",
]



#########################################################################################
# UTILS PLOT
#########################################################################################


def generate_latex_doc (containt) :

    latex_str  = "" 
    latex_str +=  "\\documentclass[landscape]{article}\n"
    latex_str +=  "\\usepackage[table]{xcolor}\n"
    latex_str +=  "\\usepackage[ margin=0cm]{geometry}\n"
    latex_str +=  "\\setlength{\\arrayrulewidth}{0.2mm}\n"
    latex_str +=  "\\setlength{\\tabcolsep}{2pt}\n"
    latex_str +=  "\\renewcommand{\\arraystretch}{1}\n"
    latex_str +=  "\\begin{document}\n"
    
    latex_str +=  containt
    
    latex_str +=  "\\end{document}\n"

    return latex_str
    
def generate_latex_table (data , table_title, value_name, value_type , precision = "%.2f", high_threshold = None , low_threshold = None) :

    dataset_labels = []

    for a in data :
        dataset_labels = list ( set (  dataset_labels + data[a].keys()  )  ) 

    algorithm_labels = data.keys()
    
        
    latex_str  = "" 
    latex_str +=  table_title + "\\\\\n"
    latex_str +=  "{\\rowcolors{3}{black!10}{black!2}\n"
    latex_str +=  "\\tiny\n"
    latex_str +=  "\\begin{tabular}{%s}\n" % ( "|l|" + "|".join(["l" for x in dataset_labels]) + "|")
    latex_str +=  "\\hline\n"
#    latex_str +=  "Dataset name & \\multicolumn{4}{c|}{ICLNUIM Dataset} & \\multicolumn{19}{c|}{TUM Dataset}  & \\multicolumn{11}{c|}{EuRoC MAV Dataset}  \\\\\n"
#    latex_str +=  "\\hline\n"
#    latex_str +=  "Scene name & \\multicolumn{4}{c|}{Living Room} & \\multicolumn{7}{c|}{Freiburg 1} & \\multicolumn{12}{c|}{Freiburg 2} & \\multicolumn{5}{c|}{Machine Hall} & \\multicolumn{3}{c|}{Vicon Room 1} & \\multicolumn{3}{c|}{Vicon Room 2}\\\\\n"
    latex_str +=  "Algorithm & " + " & ".join([x.replace("_","\_") for x in dataset_labels]) + "\\\\\n"
    latex_str +=  "\\hline\n"
    for algorithm in data :
        latex_str +=  algorithm.replace("_","\_")
        for dataset in dataset_labels :
            accuracy = None
            
            if not dataset in  data[algorithm].keys() :
                latex_str +=  "& - "
                continue

            if len(data[algorithm][dataset]) == 0  :
                printerr ("Cannot find any iteration in %s data.\n" % (dataset, algorithm))
                exit(1)

            
            for it in data[algorithm][dataset] :
                
                if not STATISTICS_SECTION in it.keys() :
                    printerr ("Invalid data (%s,%s), no STATISTICS_SECTION.\n" % (algorithm,dataset))
                    exit(1)

    
                if not ATE_COLUMN in it[STATISTICS_SECTION].keys() :
                    printerr ("Invalid data(%s,%s), no ATE_COLUMN.\n" % (algorithm,dataset))
                    exit(1)       

            for it in data[algorithm][dataset] :
                if accuracy == None :
                    accuracy = 0.0
                accuracy += float(it[STATISTICS_SECTION][value_name][value_type])
            accuracy = accuracy / len(data[algorithm][dataset])

            
            if accuracy == None :
                latex_str +=  "& - "
            elif high_threshold and accuracy > high_threshold  :
                latex_str +=  "& $>$" + precision %  high_threshold
            elif low_threshold and accuracy < low_threshold :
                latex_str +=  " & \\textbf{" + precision % accuracy + "} " 
            else :
                latex_str +=  " & " + precision % accuracy
        latex_str +=   "\\\\\n" 
    latex_str +=  "\\hline\n"
    latex_str +=  "\\end{tabular}\n"
    latex_str +=  "\n"
            
    return latex_str
    
def generate_latex (data) :

    tables  = ""
    tables += generate_latex_table (data , "ATE Mean" ,    ATE_COLUMN , MEAN_FIELD,  precision = "%.2f" , high_threshold = 1 , low_threshold = 0.1)
    tables += generate_latex_table (data , "ATE Max" ,    ATE_COLUMN ,  MAX_FIELD,  precision = "%.2f"  , high_threshold = 1 , low_threshold = 0.1)
    tables += generate_latex_table (data , "Frame count" , ATE_COLUMN , COUNT_FIELD, precision= "%d")
    
    return generate_latex_doc (tables) 
    

#########################################################################################
#  MAIN
#########################################################################################


def main():       

    # GET ARGUMENTS
    parser = argparse.ArgumentParser()
    #parser.add_argument('-i','--input', action='append', help='<Required> Set flag', required=True)
    parser.add_argument('files', metavar='N', type=str, nargs='+',  help='Files to process')
    parser.add_argument('--latex', action="store_true",  help='Generate the latex file')
    parser.add_argument('--violins', action="store_true",  help='Generate the violins data')
    parser.add_argument('--plot', type=str, help='Generate the violins pdf file')
    
    args = parser.parse_args()
    #print "Input files: %s" % (str(args.input))
    data = load_data_from_files (args.files)
    if args.latex :
        latex_str = generate_latex (data)
        print latex_str
    if args.violins :
        violins = generate_violins (data)
        print violins
    if args.plot :
        violins = generate_violins (data)
        plot_violins (violins, args.plot, default_order)
    
main()
