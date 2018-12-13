#!/usr/bin/python
import datetime as dt
import os
import sys
import subprocess
import cgi
import tempfile
import time
import string

################# UTILS #######################

def new_success (classname, name, duration = 0.0, message="") :
    return { "classname" : classname,
             "name"      : name, 
             "command"   : "",
             "env"       : {},
             "duration"  : duration,    
             "result"    : "success",    
             "message"   : message,    
             "stderr"    : "",     
             "stdout"    : "",       }

def new_failure (classname, name, duration = 0.0, message="") :
    return { "classname" : classname,
             "name"      : name, 
             "command"   : "",
             "env"       : {},
             "duration"  : duration,    
             "result"    : "failure",    
             "message"   : message,    
             "stderr"    : "",     
             "stdout"    : "",       }

def run_it(parameters, newenv = None, print_only = False, logfile = None, timeout = None) :
    '''
    This function execute a program.
    :param parameters: list containing both the executable name and it's arguments
    :param newenv: list of envirnement variable to add to the execution context
    :param PrintOnly: if True, the execution don't happend, verbosity only happends.
    '''
    sys.stderr.write("Run new command at " + str(dt.datetime.now()) + "\n")
    res = {}
    res["stdout"] = ""
    res["stderr"] = ""
    res["duration"] = 0.0
    res["returncode"] = None
    sys.stderr.write( "                   ");
    if newenv != None :
        for f in newenv.keys() :
            sys.stderr.write( " %s=\"%s\" " % (f,newenv[f]) )
    strcmd = ""
    for p in parameters :
        if " " in p :
            strcmd += ("\"" + p + "\"")
        else :
            strcmd += (p)
        strcmd += (" ")
    sys.stderr.write("  " + strcmd + "\n")
    
    my_env = os.environ.copy()
    
    if newenv != None :
        for v in newenv :   
            my_env[v] =  str ( newenv[v] )
    if print_only :
        return res

    parameters = ["stdbuf","-o0"] + parameters

    try :

        n1=dt.datetime.now()

        if logfile == None :
            fout = tempfile.NamedTemporaryFile(delete=True)
            ferr = tempfile.NamedTemporaryFile(delete=True)
        else :
            fout = open(logfile + ".out", 'w')
            ferr = open(logfile + ".err", 'w')
        proc = subprocess.Popen(strcmd,shell=True, env=my_env, stdout=fout, stderr=ferr, bufsize=0)

        proc_killed = False

        if timeout :
            d =  dt.datetime.now() - n1  
            elapse_time = ( d.days * 24 * 3600 + d.seconds ) 

            while proc.poll() == None and elapse_time < timeout :
                time.sleep(1)

            if proc.poll() == None :
                killProc(proc.pid)
                proc_killed = True
                
    
        proc.wait()

        res["returncode"] = int(proc.returncode)

        sys.stderr.write( "Return code = %s\n" % (proc.returncode) )

        if proc_killed :
            sys.stderr.write( "Has been stopped.\n")
            res["returncode"] = -1

        
        n2=dt.datetime.now()
        d = n2 - n1
        res["duration"] = ( d.days * 24 * 3600 + d.seconds ) * 1 + (d.microseconds / 1000000.0)
     
        fout.close()
        ferr.close()
        
        printable = set(string.printable)
        if logfile == None :
            res["stdout"] = ""
            res["stderr"] = ""
        else :            
            try :
                fout = open(logfile + ".out", 'r')
                res["stdout"] = fout.read().decode('utf-8', 'ignore').encode('utf-8')
                res["stdout"] = filter(lambda x: x in printable, res["stdout"] )
                fout.close()
                if not fout.closed :
                    sys.stderr.write( "error with stdout file.\n" )
            except IOError :
                sys.stderr.write( "error with stdout reading back.\n" )
                res["stdout"] = ""

            try :
                ferr = open(logfile + ".err", 'r')
                res["stderr"] = ferr.read().decode('utf-8', 'ignore').encode('utf-8')
                res["stderr"]  = filter(lambda x: x in printable, res["stderr"] )
                ferr.close()

                if not ferr.closed :
                    sys.stderr.write( "error with stderr reading back.\n" )
            except IOError :
                sys.stderr.write( "error with stderr file.\n" )
                res["stderr"] = ""

    except OSError :
        sys.stderr.write( "fail to execute this command %s.\n" % (" ".join(parameters)))
        

    return res
        

def run_test (x , logdir = None, print_only = False, timeout = None ) :


    if logdir  != None :
        logfile =  logdir + "/" + x["classname"] + "_" + x["name"]
    else :
        logfile = None
    try :
        run_res = run_it (x["command"], x["env"], logfile = logfile , print_only = print_only, timeout = timeout )
    except TypeError :
        sys.stderr.write( "TypeError return from run_it(%s,%s,%s,%s)\n" % ( x["command"], x["env"],  logfile , print_only ) )
    test_result = { "classname" : x["classname"],
                        "name"      : x["name"], 
                        "command"   : x["command"],
                        "env"       : x["env"],
                        "duration"  : 0.0,    
                        "result"    : "unknown",    
                        "message"   : "",    
                        "stderr"    : "",     
                        "stdout"    : "",       }
    
    if "stdout" in x :
        f = open(x["stdout"], 'w')
        f.write(run_res["stdout"])
        f.close
    if "stderr" in x :
        f = open(x["stderr"], 'w')
        f.write(run_res["stderr"])
        f.close            


    if "duration" in run_res :
        test_result["duration"] = run_res["duration"]
    if "returncode" in run_res :
        if run_res["returncode"] == 0 :
            test_result["result"] = "success"
        elif run_res["returncode"] == None :
            test_result["result"] = "error"
        else :
            test_result["result"] = "failure"

        test_result["message"] = "Return code = %s" % (run_res["returncode"]) 
        if "stderr" in run_res :
            test_result["stderr"] = run_res["stderr"]
        if "stdout" in run_res :
            test_result["stdout"] = run_res["stdout"]
                
    else :
        test_result["result"] = "error"
            

    return test_result

def run_testsuite (testsuite) :
    results = []
    for x in testsuite :
        results +=  run_test (x) 
    return results


def print_testsuite (name,testsuite) :
    errors = len([x for x in testsuite if x["result"] == "error"])
    failures = len([x for x in testsuite if x["result"] == "failure"])
    skips = len([x for x in testsuite if x["result"] == "skip"])
    tests = len([x for x in testsuite])
    time = sum ([x["duration"] for x in testsuite])
    res = ""
    res +="<testsuite errors=\"%d\" failures=\"%d\" name=\"%s\" skips=\"%d\" tests=\"%d\" time=\"%f\">\n" % (errors,failures,name,skips,tests,time)

    for x in testsuite :

        res += "  <testcase classname=\"%s\" name=\"%s\" time=\"%f\">\n" % (cgi.escape(x["classname"],True),cgi.escape(x["name"],True),x["duration"])

        if x["result"] == "success" :
            pass
        elif x["result"] == "failure"   :
            res += "      <failure message=\"%s\"> Failure while running '%s' with env='%s'.</failure>\n" % (cgi.escape(x["message"],True),cgi.escape(" ".join(x["command"]),True),cgi.escape(str(x["env"]),True))
        elif x["result"] == "error"   :
            res += "      <error message=\"%s\">Error while running '%s' with env='%s'.</error>\n" % (cgi.escape(x["message"],True),cgi.escape(" ".join(x["command"]),True),cgi.escape(str(x["env"]),True))
        elif x["result"] == "skip"   :
            res += "      <skipped />\n"

        if len(x["stdout"]) > 0 :
            res += "      <system-out>%s</system-out>\n"% (cgi.escape(x["stdout"],True))
        if len(x["stderr"]) > 0 :
            res += "      <system-err>%s</system-err>\n"% (cgi.escape(x["stderr"],True))

        res += "  </testcase>\n"

    res +="</testsuite>"

    return res



if __name__ == "__main__":
    testsuite = [
        { "classname" : "kfusion-main-1" ,  
          "name" : "test1-execution"  , 
          "command" : ["./repository/build/kfusion/kfusion-main" , "-o" , "log/kfusion-main.log"] ,
          "env" : {"pouet" : "toto"}
      },  
        { "classname" : "kfusion-main-2" ,  
          "name" : "test2-execution"  , 
          "command" : ["./repository/buildd/kfusion/kfusion-main" , "-o" , "log/kfusion-main.log"] ,
          "env" : {"pouet" : "toto"} 
      },
      ]
    res = run_testsuite (testsuite)
    print print_testsuite("yeat",res)
