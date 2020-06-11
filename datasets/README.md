#### Supported Datasets

Here is a list of the datasets available:
   - TUM RGB-D SLAM dataset [Sturm et al., IROS 2012]: https://vision.in.tum.de/data/datasets/rgbd-dataset
   - ICL-NUIM dataset [Handa et al., ICRA 2014]: https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html
   - EuRoC MAV Dataset [Burri et al., IJJR 2016]: https://projects.asl.ethz.ch/datasets/doku.php
   - SVO sample dataset [Forster et al., ICRA 2014]: https://github.com/uzh-rpg/rpg_svo

If you use any of these datasets, please refer to their respective publication.

---------------------------------------

In addition to their _primary_ format, the following datasets
are also supported as __rosbag__ files:
   - TUM RGB-D SLAM dataset [https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats]

Please note that, as rosbags contain unprocessed sensor data,
processing rosbag datasets can take considerably longer
than processing datasets in their primary format.

Rosbag support requires the following ROS packages:
   - cpp_common
   - roscpp_serialization
   - rostime
   - rosbag_storage
   - tf

The following versions of the ROS packages have been tested:
   - ROS kinetic / Linux Ubuntu 16.04 (xenial)
   - ROS melodic / Linux Ubuntu 18.04 (bionic)

If __rosbag__ support is required and ROS is not installed in the standard location (/opt/ros) make sure to setup the ROS environment before building SLAMBench.
