import sys
DATASET_DIR = "datasets/"

fd = open(sys.argv[1])
data =fd.read()
fd.close()


def echo(str="", echoargs = ""):
    print("\t@echo " + echoargs + " \"" + str + "\"")


print("datasets:")
echo("The following datasets are built on your system and available for use:")
print("\t@for f in `find datasets/ | grep [.]slam` ; do echo \" - $$f\" ; done")
echo()
echo("Here is a list of the datasets available.")
echo("If you are using any of the following datasets, ${BoldRed}please refer to their respective publications${ColorOff}:", "-e")
echo("\t- TUM RGB-D SLAM dataset [Sturm et al, IROS'12]: https://vision.in.tum.de/data/datasets/rgbd-dataset")
echo("\t- ICL-NUIM dataset [Handa et al, ICRA'14]: https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html")
echo("\t- EuRoC MAV Dataset [Burri et al, IJJR'16]: https://projects.asl.ethz.ch/datasets/doku.php")
echo("\t- SVO sample dataset [Forster et al, ICRA 2014]: https://github.com/uzh-rpg/rpg_svo")
echo("\t- Bonn RGB-D Dynamic Dataset [Palazzolo et al, IROS'19]: http://www.ipb.uni-bonn.de/data/rgbd-dynamic-dataset/")
echo("\t- UZH-FPV Drone Racing Dataset [Delmerico et al, ICRA'19]: http://rpg.ifi.uzh.ch/uzh-fpv.html")
echo("\t- OpenLORIS-Scene datasets [Shi et al, ICRA'20]: https://lifelong-robotic-vision.github.io/dataset/scene")
echo("=================================================================================================================")
echo()
echo("=================================================================================================================")
echo("SLAMBench integrates tools to automatically generate files compatible with SLAMBench from existing datasets.")
echo("SLAMBench cannot download the OpenLORIS data for you. Please download the data manually (*-package.tar) to ./datasets/OpenLORIS/")
echo("For details, please visit: https://lifelong-robotic-vision.github.io/dataset/scene")
echo()

targets = []
dataset_name = ""
for line in data.split("\n"):
    if '#' in line or len(line) < 2:
        continue
    items = line.split(":")
    if "Dataset" in items:
        dataset_name = items.pop(1)
        continue
    items = line.split(";")
    dataset_tag = items.pop(0)
    dataset_description = items.pop(0)
    # here need another target
    echo("\t### " + dataset_name + " " + dataset_description + " ###")
    for dataset_file in items:
        echo("\t make ./" + DATASET_DIR + dataset_name + "/" + dataset_file)
    echo()

echo("\tDatasets with a ROS option: TUM")
echo("\t Use the ${BoldGreen}use_rosbag${ColorOff} option to build a dataset from a ROS bag:")
echo("\t make ./datasets/TUM/freiburg1/rgbd_dataset_freiburg1_xyz.slam use_rosbag")
echo()

# at the end need to include dataset-utils.makefile
# make datasettag list
# make datasettag.all
