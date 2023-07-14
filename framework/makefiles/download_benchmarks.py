import sys
import json

with open(sys.argv[1]) as f:
    data = json.load(f)


def echo(str="", echoargs = ""):
    print("\t@echo " + echoargs + " \"" + str + "\"")


keys = ["name", "reference", "orig_repo", "used_repo", "branch", "path"]

print("usecases:")
echo()
echo("=================================================================================================================")
echo("Current list of compatible SLAM systems (alphabetical order). If you are using any of the following SLAM algorithms, ${BoldRed} please refer to their respective publications ${ColorOff}", "-e")
echo()

for algorithm_token, details in data.items():
    name = details["name"]
    reference = details["reference"]
    original_repo = details["original_repo"]
    new_repo = details["new_repo"]
    target_dir = details["path"]
    print("\t@echo -n \"  - %s [%s] : \" ; if [ -d %s ] ; then echo -e \" ${BoldGreen}Found${ColorOff}\"; else echo -e \" ${BoldRed}Not found (make %s) ${ColorOff}\" ; fi" % (name, reference, target_dir, algorithm_token))
    echo("      Original repository: %s" % original_repo)
    echo("      Used repository: %s" % new_repo)
    echo("      available targets are : %s" % name)
    echo()
    print("")

echo("If you want to test SLAMBench with existing SLAM algorithms, once you have download it please run \\\"make slambench APPS=slam1,slam2,...\\\"")
echo("   e.g. make slambench APPS=kfusion,orbslam2")
echo("   You can also use \\\"make slambench APPS=all\\\" to compile them all.")
echo("")
echo("As a next step we suggest you run make datasets.")
echo("")
echo("=================================================================================================================")

previous = None
for algorithm_token, details in data.items():
    name = details["name"]
    reference = details["reference"]
    original_repo = details["original_repo"]
    new_repo = details["new_repo"]
    target_dir = details["path"]
    branch = details["branch"]

    if previous != name:
        print("")
        print("%s:" % algorithm_token)
        echo("=================================================================================================================")
        echo(f"{name} ({reference})")
        echo("Original repository: %s" % original_repo)
        echo("Used repository: %s" % new_repo)
        echo("=================================================================================================================")
        print("")
        print("\t@echo \"Are you sure you want to download this use-case (y/n) ?\" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! \"$$REPLY\" == \"y\" ] ; then echo -e \"\\nExit.\"; false; else echo -e \"\\nDownload starts.\"; fi")
    print("\tmkdir -p %s" % target_dir)
    print("\trm %s -rf" % target_dir)
    print("\tgit clone --recursive --branch %s %s %s" % (branch, new_repo, target_dir))
    print("\t@echo \"cmake_minimum_required(VERSION 3.10)\"   > benchmarks/$@/CMakeLists.txt")
    print("\t@echo \"explore_implementations ( $@ src/* )\"     >> benchmarks/$@/CMakeLists.txt")
    previous = name

list_str = " ".join(set([algoname for algoname in data.keys()]))
print("")
print(".PHONY: %s" % list_str)
print("algorithms :  %s" % list_str)

print("")
print("benchmarks_status:")
for algorithm_token, details in data.items():
    echo("************ Check-in %s in %s" % (algorithm_token, details["path"]))
    print("\t@if [ -d %s ] ; then git -C %s diff; fi" % (details["path"], details["path"]))
