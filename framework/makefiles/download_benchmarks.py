import sys

fd = open(sys.argv[1])
data =fd.read()

targets = []

for l in data.split("\n") :
    item = l.split(";")
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
print("\t@echo \"\"")
print("\t@echo \"=================================================================================================================\"")
print("\t@echo -e \"Current list of compatible SLAM systems (alphabetical order). If you are using one of those SLAM algorithms, \\033[1;31mplease refer to their respective publications\\033[0m:\"")
print("\t@echo \"\"")

for item in targets :
    name = item["name"]
    citation = item["citation"]
    original_repo = item["original_repo"]
    target_dir = item["target_dir"]
    print("\t@echo -n \"  - %s : \" ; if [ -d %s ] ; then echo -e \"\\033[1;32mFound\\033[0m\" ; else echo -e \"\\033[1;31mNot found (make %s)\\033[0m\" ; fi" % (citation,target_dir,name))
    print("\t@echo    \"    repository: %s\"" % (original_repo))
    print("\t@echo    \"    available targets are : %s\"" % (name))
    print("\t@echo    \"\"")
    print("")

    
print("\t@echo \"If you want to test SLAMBench with existing SLAM algorithms, once you have download it please run \\\"make slambench APPS=slam1,slam2,...\\\"\"")
print("\t@echo \"   e.g. make slambench APPS=kfusion,orbslam2\"")
print("\t@echo \"   You can also use \\\"make slambench APPS=all\\\" to compile them all.\"")
print("\t@echo \"\"")
print("\t@echo \"As a next step we suggest you run make datasets.\"")
print("\t@echo \"\"")
print("\t@echo \"=================================================================================================================\"")




previous = None
for item in targets :

    name = item["name"]
    citation = item["citation"]
    original_repo = item["original_repo"]
    target_dir = item["target_dir"]
    new_branch = item["new_branch"]
    new_repo = item["new_repo"]
    
    if previous != name :
        print("%s:" % name)
        print("\t@echo \"=================================================================================================================\"")
        print("\t@echo    \"  - %s \"" % citation)
        print("\t@echo    \"    Original repository: %s\"" % original_repo)
        print("\t@echo    \"    Used repository: %s\"" % new_repo)
        print("\t@echo \"=================================================================================================================\"")
        print("\t@echo \"\"")
        print("\t@echo \"Are you sure you want to download this use-case (y/n) ?\" && ${GET_REPLY} && echo REPLY=$$REPLY && if [ ! \"$$REPLY\" == \"y\" ] ; then echo -e \"\\nExit.\"; false; else echo -e \"\\nDownload starts.\"; fi")
    print("\tmkdir -p %s" % target_dir)
    print("\trm -rf %s" % target_dir)
    print("\tgit clone --branch %s %s %s"% (new_branch, new_repo,target_dir))
    print("\t@echo \"cmake_minimum_required(VERSION 2.8)\"   > benchmarks/$@/CMakeLists.txt")
    print("\t@echo \"explore_implementations ( $@ src/* )\"     >> benchmarks/$@/CMakeLists.txt")
    previous = name
list_str = " ".join(set([x["name"] for x in targets]))    
print(".PHONY: %s" % list_str )
print("algorithms :  %s" % list_str)



print("benchmarks_status:")

for item in targets :
    name = item["name"]
    print("\t@echo \"************ Check-in %s in %s\"" % (name,item["target_dir"]))
    print("\t@if [ -d %s ] ; then git -C %s diff; fi" % (item["target_dir"],item["target_dir"]))
    
