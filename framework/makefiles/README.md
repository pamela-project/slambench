# SLAMBench Dependency System #

## Description ##

The idea is to maximise the chance of a good build, by selection the best cocktail of libraries.

 Name          | repository                                                     | Commit / Tag / Version                   | Date         | Why                  | Notes
-------------  | -------------------------------------------------------------- | ---------------------------------------- | ------------ | -------------------- | ---------
BRISK          | https://www.doc.ic.ac.uk/~sleutene/software/brisk-2.0.3.zip    | 2.0.3                                    | N/A          | ?                    | -
CERES          | https://ceres-solver.googlesource.com/ceres-solver             | 7c57de5080c9f5a4f067e2d20b5f33bad5b1ade6 | ?            | ?                    | -
CVD            | https://github.com/edrosten/libcvd                             | d190474150d4695e4c957863c5121c7eb79615d9 |              | PTAM                 | -
EIGEN          | http://bitbucket.org/eigen/eigen                               | 3.2                                      |              |                      | -
FLANN          | https://github.com/mariusmuja/flann                            | 06a49513138009d19a1f4e0ace67fbff13270c69 |  5 Aug 2016  | PCL                  | -
FREEIMAGE      | https://github.com/mikesart/freeimage.git                      | d49fb3982c1cb7826bc10edaf5c0ac4d9104660f |              |                      | + Patches
FREENECT       | https://github.com/OpenKinect/libfreenect.git                  | 83e57e1318cc64c9aabac481b9e330acc1914a23 |              |                      | -
G2O            | https://github.com/RainerKuemmerle/g2o                         | 1b118ac2ed2055c4016c3b7cbd710225ed1651af | 12 Jan 2017  | LSDSLAM              | -
GCC            | git://gcc.gnu.org/git/gcc.git                                  | gcc-5_3_0-release                        |              |                      | + ec1cc0263f156f70693a62cf17b254a0029f4852
GVARS          | https://github.com/edrosten/gvars                              | fc58c500c9d8f8713fb87a98cf7fb6be1db3295f |              | PTAM                 | -
LIBOPENCL-STUB | https://github.com/krrishnarraj/libopencl-stub.git             | b4f84459e3a3a14d6a18b5dabe0a6ae9cbef709e |              |                      | -
LIBUSB         | https://github.com/libusb/libusb.git                           | 8ddd8d994df6e367603266630bc2fe83b9cad868 |              |                      | -
OPENCV         | https://github.com/Itseez/opencv.git                           | 2c9547e                                  |              | LSDSLAM              | + Patches
OPENGV         | https://github.com/laurentkneip/opengv                         | cc32b16281aa6eab67cb28a61cf87a2a5c2b0961 |              |                      | -
OPENNI 1.5     | https://github.com/OpenNI/OpenNI.git                           | 54e899c492f69aa8fa3e133fdd7d6b468f017b99 |              |                      | + Patches
OPENNI2        | https://github.com/occipital/OpenNI2/                          | 1fce8edffab43c4a4cf201cff86f415b07a2d37f |              |                      | -
OPENTUNER      | https://github.com/jansel/opentuner.git                        | master...                                |              | DSE                  | -
PANGOLIN       | https://github.com/stevenlovegrove/Pangolin.git                | 8b8b7b96adcf58ac2755dedd3f681fc512385af0 |              | GUI, EFUSION         | -
PCL            | https://github.com/PointCloudLibrary/pcl.git                   | 6fb1b65d3099a915255b070269b1ac78ed384921 |              | OR                   | -
SensorKinect   | https://github.com/avin2/SensorKinect.git                      | 15f1975d5e50d84ca06ff784f83f8b7836749a7b |              |                      | -
SUITESPARSE    | https://github.com/jluttine/suitesparse.git                    | v4.3.1                                   |              | EFUSION              | -
TOON           | https://github.com/edrosten/TooN.git                           | 92241416d2a4874fd2334e08a5d417dfea6a1a3f | 21 Sep 2015  | KFUSION, PTAM, CVD   | -
