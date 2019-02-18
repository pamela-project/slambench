
#########################################################################################
# UTILS SYSTEM
#########################################################################################

import sys



class shellcolors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED   = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    
WARNING_CODE = shellcolors.YELLOW + "[ERROR]" +  shellcolors.ENDC
ERROR_CODE = shellcolors.RED + "[ERROR]" +  shellcolors.ENDC
INVALID = shellcolors.RED + "[INVALID]" +  shellcolors.ENDC
VALID   = shellcolors.GREEN + "[VALID]" +  shellcolors.ENDC

verbose_level = False

def setverbose(v) :
    global verbose_level
    verbose_level = v

def printwarning(msg) :
    sys.stderr.write(WARNING_CODE + " " + msg)

    
def printerr(err) :
    sys.stderr.write(ERROR_CODE + " " + err)
    
def printinfo(err) :
    global verbose_level
    if verbose_level :
        sys.stderr.write(shellcolors.GREEN + "[INFO]" +  shellcolors.ENDC + " " + err)

