import sys

fd = open(sys.argv[1])
data = fd.read()
targets = []


def echo(str="", echoargs = ""):
    print("\t@echo " + echoargs + " \"" + str + "\"")


for line in data.split("\n"):
    item = line.split(";")
    targets.append({
        "name"          : item[0],
        "citation"      : item[1],
        "original_repo" : item[2],
        "new_branch"    : item[3],
        "new_repo"      : item[4],
        "target_dir"    : item[5],
    })

fd.close()

print("usecases:")
echo()
echo("=================================================================================================================")
echo("Current list of compatible SLAM systems (alphabetical order). If you are using any of the following SLAM algorithms, ${BoldRed} please refer to their respective publications ${ColorOff}", "-e")
echo()

for item in targets:
    name = item["name"]
    citation = item["citation"]
    original_repo = item["original_repo"]
    new_repo = item["new_repo"]
    target_dir = item["target_dir"]
    print("\t@echo -n \"  - %s : \" ; if [ -d %s ] ; then echo -e \" ${BoldGreen}Found${ColorOff}\"; else echo -e \" ${BoldRed}Not found (make %s) ${ColorOff}\" ; fi" % (citation, target_dir, name))
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
for item in targets:
    name = item["name"]
    citation = item["citation"]
    original_repo = item["original_repo"]
    target_dir = item["target_dir"]
    new_branch = item["new_branch"]
    new_repo = item["new_repo"]

    if previous != name:
        print("")
        print("%s:" % name)
        echo("=================================================================================================================")
        echo("%s" % citation)
        echo("Original repository: %s" % original_repo)
        echo("Used repository: %s" % new_repo)
        echo("=================================================================================================================")
        print("")
        print("\t@echo \"Are you sure you want to download this use-case (y/n) ?\" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! \"$$REPLY\" == \"y\" ] ; then echo -e \"\\nExit.\"; false; else echo -e \"\\nDownload starts.\"; fi")
    print("\tmkdir -p %s" % target_dir)
    print("\trm %s -rf" % target_dir)
    print("\tgit clone --recursive --branch %s %s %s" % (new_branch, new_repo, target_dir))
    print("\t@echo \"cmake_minimum_required(VERSION 2.8)\"   > benchmarks/$@/CMakeLists.txt")
    print("\t@echo \"explore_implementations ( $@ src/* )\"     >> benchmarks/$@/CMakeLists.txt")
    previous = name

list_str = " ".join(set([x["name"] for x in targets]))
print("")
print(".PHONY: %s" % list_str)
print("algorithms :  %s" % list_str)

print("")
print("benchmarks_status:")
for item in targets:
    name = item["name"]
    echo("************ Check-in %s in %s" % (name, item["target_dir"]))
    print("\t@if [ -d %s ] ; then git -C %s diff; fi" % (item["target_dir"], item["target_dir"]))
