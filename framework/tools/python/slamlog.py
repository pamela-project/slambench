import os
import time
import re
import math
import numpy as np
import pickle
import deepdish
from .utils import *
import gzip
from progress.bar import Bar


LIBRARY_NAME_PROPERTY = "load-slam-library"

PROPERTIES_SECTION     = "Properties"
STATISTICS_SECTION     = "Statistics"
SUMMARY_SECTION        = "Summary"
FRAME_NUMBER_COLUMN    = "Frame Number"

ATE_COLUMN             = "AbsoluteError"
CPU_MEMORY_COLUMN      = "CPU_Memory"
DURATION_COLUMN        = "Duration_Frame"

FPS_COLUMN             = "FPS"


MAX_FIELD            = "MAX"    
MIN_FIELD            = "MIN"     
MEAN_FIELD            = "MEAN"     
COUNT_FIELD            = "COUNT"  
MEDIAN_FIELD            = "MEDIAN"     

MAX_SUFFIX            = "_MAX"    
MIN_SUFFIX            = "_MIN"     
MEAN_SUFFIX            = "_MEAN"     
COUNT_SUFFIX            = "_COUNT"  
MEDIAN_SUFFIX            = "_MEDIAN"             

#############################################################################################
########     PARSERS
#############################################################################################


def load_inputs (inputs) : 
    printinfo ("Load inputs (%s)" % len(inputs))
    log_files = []
    summary_files = []
    data = {}
    
    for pathname in inputs :
        if os.path.isdir(pathname) :
            dirname = pathname
            try : # Log directory
                log_files += [os.path.join(dirname, f) for f in os.listdir(dirname) if f[-4:] == ".log" and os.path.isfile(os.path.join(dirname, f))]
            except OSError :
                printerr("Working directory %s not found.\n" % dirname )
                return None
        elif pathname[-4:] == ".log" : # Log file        
            log_files += [pathname]
        else : # potential Summary file
            summary_files += [pathname]

    bar = Bar('Log files...',  max=len(log_files))
    for filename in log_files :
        bar.next()
        filedata = load_log_file(filename)
        if filedata :
            data[filename] = load_log_file(filename)
        else :
            printwarning("File is corrupt or incomplete : %s" % filename)

    bar.finish()

    bar = Bar('Summary files...', max=len(summary_files))
    for filename in summary_files :
        bar.next()
        temp = load_summary_file(filename)
        for key,value in temp.items() :
            if key in data :
                printerr("File name appear twice : %s" % key)
            data[key] = value

    bar.finish()
    return data


def save_summary_file (data, filename ):
    printinfo("Saving data in %s ...\n" % filename)
    if filename[-4:] == ".pkl" :
        with open(filename, 'wb') as f:
            pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)
    elif filename[-4:] == ".gkl" :
        with gzip.open(filename, 'wb') as f:
            pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)
    elif filename[-3:] == ".h5" :            
        deepdish.io.save(filename, data, compression=('blosc', 9))
    else :
        printerr ("Unsupported file format: %s" % filename)
        
def load_summary_file ( filename ):
    if filename[-4:] == ".pkl" :
        with open(filename, 'rb') as f:
            return pickle.load(f)
    elif filename[-4:] == ".gkl" :
        with gzip.open(filename, 'rb') as f:
            return pickle.load(f)
    elif filename[-3:] == ".h5" :
        return deepdish.io.load(filename)
    else :
        printerr ("Unsupported file format: %s" % filename)
            
            


def load_data_from_input_dirs ( input_dirs ) :

    filelist = []
    for dirname in input_dirs :
        try :
            filelist += [os.path.join(dirname, f) for f in os.listdir(dirname) if f[-4:] == ".log" and os.path.isfile(os.path.join(dirname, f))]
        except OSError :
            printerr("Working directory %s not found.\n" % dirname )
            return None
    printinfo("%d files to load ...\n" % len(filelist))
    data = load_data_from_files (filelist)
    return data


def load_log_file(filename) :

    
    start_time = time.time()
    
    f = open(filename)
    raw = f.read()
    f.close()

    inside = None
    headers = None

    data = {"date" : None , STATISTICS_SECTION : {} , PROPERTIES_SECTION : {}}
    lines = raw.split("\n")
    load_time = time.time()
        
    for line in lines :
        
        if line == "Process every frame mode enabled" :
            continue
        if line[0:len("SLAMBench Report run started:")] == "SLAMBench Report run started:" :
            matching_header = re.match("SLAMBench Report run started:\s+(.*)",line)
            assert(matching_header)
            data["date"] = str(matching_header.group(1))
            continue

        if re.match(PROPERTIES_SECTION + ":",line) :
            inside = PROPERTIES_SECTION
            continue
        
        if re.match(STATISTICS_SECTION + ":",line) :
            inside = STATISTICS_SECTION
            continue

        if line == "" :
            continue
        
        if re.match("=+",line) :
            if inside != None :             
                continue
            else :
                printerr ("Error unknow section.\n")
                return None

        if inside == PROPERTIES_SECTION :
            matching_arguments = re.match("\s*(.*):\s+(.*)\s*",line)
            if matching_arguments :
                data[PROPERTIES_SECTION][matching_arguments.group(1)] = matching_arguments.group(2)
                continue
            
        if inside == STATISTICS_SECTION :
            matching_fields = line.split("\t")
            if matching_fields :
                if headers and len(headers) == len(matching_fields):
                    for i in range (len(matching_fields)) :
                        current_value = float("NaN")
                        try :
                            current_value = float(matching_fields[i])
                        except ValueError:
                            current_value = float("NaN")
                        data[STATISTICS_SECTION][headers[i]] += [current_value]
                        #if math.isnan(float(matching_fields[i])) :
                        #    printerr ( INVALID + " %s : Error while parsing the file, NaN found.\n" % filename )
                        #    return None
                    continue
                else :
                    if headers :
                        printerr ("New \n")
                    headers = matching_fields[:]
                    for k in headers :
                        if not k in data[STATISTICS_SECTION].keys():
                            data[STATISTICS_SECTION][k] = []
                    continue
                    

                                                
        printerr ("[load_data_from_file('%s')] Error line not parsed inside '%s': '%s'\n" % (filename,inside,line))
        return None
    loop_time = time.time()

    if headers == None :
        return None

    stats = turn_data_to_stats(data)
    if stats == None :
        printerr (" %s : stats == None.\n" % filename )
        return None
    
    if len(stats.keys()) != 1 :
        printerr("This file has more than one algorithm or no algorithm\n")
        return None
    
    data[SUMMARY_SECTION] = stats

    return data

def turn_data_to_stats(data) :
    stats = {}
    if not data or not STATISTICS_SECTION in data.keys() :
        printerr("no data or no STATISTICS_SECTION in data.keys()\n")
        return None
    if not FRAME_NUMBER_COLUMN in data[STATISTICS_SECTION].keys() :
        printerr("no '%s' in data[STATISTICS_SECTION].keys()\n" % FRAME_NUMBER_COLUMN )
        printerr("data[STATISTICS_SECTION].keys() = %s\n" % data[STATISTICS_SECTION].keys() )
        return None
    frame_count = len(data[STATISTICS_SECTION][FRAME_NUMBER_COLUMN])
    last_algorithm_name = None

    ## FIRST PASS TO FIND ALGO SPECIFIC FIELDS 
    for k in data[STATISTICS_SECTION].keys() :
        matching_key = re.match("^((.+)-)?(.+)$",k)

        if not matching_key :
            printerr ("Error with '%s' does not match any known field names.\n" % k)
            printerr ("Statistics header was :\n%s\n" % data[STATISTICS_SECTION].keys() )
            return None
        algorithm_name = matching_key.group(2)
        row_name = matching_key.group(3)
        if not algorithm_name and not row_name in ["Frame Number", "Timestamp"] :
            algorithm_name = "unnamed"
        if  (algorithm_name) :
            if not algorithm_name in stats.keys() :
                stats[algorithm_name] = {}
            if not row_name in stats[algorithm_name].keys() :
                stats[algorithm_name][row_name] = {}
            valid_numbers = [ x for x in data[STATISTICS_SECTION][k] if not math.isnan(float(x)) ]
            if len(valid_numbers) > 0 :                
                stats[algorithm_name][row_name] = {
                    COUNT_FIELD: len(valid_numbers),
                    MIN_FIELD: min(valid_numbers),
                    MAX_FIELD: max(valid_numbers),
                    MEDIAN_FIELD: np.median(valid_numbers),
                    MEAN_FIELD: np.mean(valid_numbers)
                }
            else :
                stats[algorithm_name][row_name] =  {
                    COUNT_FIELD: len(valid_numbers),
                    MIN_FIELD: float("NaN"),
                    MAX_FIELD:  float("NaN"),
                    MEDIAN_FIELD:  float("NaN"),
                    MEAN_FIELD:  float("NaN"),
                }
            if not last_algorithm_name :
                last_algorithm_name = algorithm_name
            else :
                if algorithm_name != last_algorithm_name :
                    printerr ("More than one algo used, current algorithm name is '%s', new algorithm name is '%s' unsupported case." % ( last_algorithm_name, algorithm_name  ) )
                    exit(1)

    return stats

def load_data_from_files (filelist) :
    data = {}
    for filename in filelist :
        
        start_time = time.time()
        temp = load_log_file(filename)
        load_time = (time.time() - start_time) * 1000
        
        if temp and STATISTICS_SECTION in temp.keys()and PROPERTIES_SECTION in temp.keys() :
          stats = turn_data_to_stats(temp)
          
          if not "input" in temp[PROPERTIES_SECTION].keys() :
              printerr (" %s : input argument not found.\n" % filename )
              continue
    
          dataset = temp[PROPERTIES_SECTION]["input"]
          #dataset = dataset.split("/")[-1]

          libraryname = temp[PROPERTIES_SECTION][LIBRARY_NAME_PROPERTY]
          
          if stats == None :
              printerr (" %s : stats == None.\n" % filename )
              continue

          if len(stats.keys()) != 1 :
              printerr("This file has more than one algorithm or no algorithm\n")
              continue

              
          if not libraryname in data.keys() :
              data[libraryname] = {}
              
          if not dataset in data[libraryname].keys() :
              data[libraryname][dataset] = []
                  
          data[libraryname][dataset] += [{PROPERTIES_SECTION :  temp[PROPERTIES_SECTION] , "date" :  temp["date"], STATISTICS_SECTION : stats.values()[0]}]
          printinfo (" %s loaded in %f ms : len of stats = %d\n" % (filename,load_time,len(stats.keys())))
        else :
          printerr (" %s : Error while loading the file. \n" % (filename))
        
    return data


def flat_data (data_array, Xcols, Ycols) :
    
    main = {}
    for run in data_array :
        wee = {}
        for col in run[PROPERTIES_SECTION] :
            if col in Xcols :
                wee[col] = run[PROPERTIES_SECTION][col]
                
        for col in run[STATISTICS_SECTION] :
            for subcol in run[STATISTICS_SECTION][col].keys() :
                if col + "_" + subcol in Ycols :
                    wee[col + "_" + subcol] = run[STATISTICS_SECTION][col][subcol]
        if len(main.keys()) != 0 and len(main.keys()) != len(wee.keys())  :
            printerr("Invalid datapoint.\n")
            exit(1)

        for x in Xcols + Ycols :
            if not x in wee.keys() :
                printerr("%s not found in the log.\n" % x)
                exit(1)
                
        for row in wee.keys() :
            if not row in main :
                main [row] = []
            main [row] += [wee[row]]
 
    return main
