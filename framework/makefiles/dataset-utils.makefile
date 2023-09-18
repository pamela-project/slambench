####################################
#### DATA SET GENERATION        ####
####################################
check_generator:=if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi

#### OpenLORIS-Scene ####
datasets/OpenLORIS/%.7z :  # Example : $* = office1/office1-3
	# extract 7z from the tar file of the scene, e.g. office1-1_7-package.tar
	for f in $(@D)/*-package.tar; do echo $$f && mkdir -p $(@D) && tar xvf $$f -C $(@D); done
	if [ ! -f $@ ]; then echo "Could not find $(@D)*-package.tar or $@. Please download the data first."; fi

datasets/OpenLORIS/%.dir : ./datasets/OpenLORIS/%.7z
	7z x $< -o$(@D) -aos
	# add the '.dir' suffix
	mv $(subst .7z,,$<) $(subst .7z,.dir,$<)

datasets/OpenLORIS/%.slam : ./datasets/OpenLORIS/%.dir
	${check_generator}
	./build/bin/dataset-generator -d OpenLORIS -i $</ -o $@ $(DATASET_OPTIONS)
	echo "Generated $@"

datasets/OpenLORIS/%.all :
	# if there are any tar, untar them; then build each 7z into a slam file
	scene=datasets/OpenLORIS/$*; \
	if [ -f $$scene*-package.tar ]; then \
		mkdir -p $$scene; \
		for f in $$scene*-package.tar; do \
			echo $$f && tar xvf $$f -C $$scene; \
		done; \
	fi; \
	for f in $$scene/*.7z; do \
		target=`echo $$f | tr .7z .sl`am ; \
		echo =============== $$target =============== ; \
		$(MAKE) $$target; \
	done

.SECONDARY: $(OBJS)


#### EuRoCMAV ####
./datasets/EuRoCMAV/%.zip :  # Example : $* = machine_hall/MH_01_easy/MH_01_easy
	mkdir -p $(@D)
	cd $(@D)  &&  ${WGET} "http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/$*.zip"


./datasets/EuRoCMAV/%.dir : ./datasets/EuRoCMAV/%.zip
	mkdir $@
	unzip $< -d $@

./datasets/EuRoCMAV/%.slam :  ./datasets/EuRoCMAV/%.dir
	${check_generator}
	./build/bin/dataset-generator -d eurocmav -i $</mav0 -o $@ -imu true --stereo-grey true -gt true


#### TUM ####
# check if using tgz file or rosbag
ifeq (TUM, $(findstring TUM, $(MAKECMDGOALS)))
  ifeq (use_rosbag, $(filter use_rosbag, $(MAKECMDGOALS)))
    FILETYPE = bag
    DATASET = tum-rosbag
  else
    FILETYPE = tgz
    DATASET = tum
  endif
endif

use_rosbag:
	@:

./datasets/TUM/%.$(FILETYPE) :  # Example : $* = freiburg2/rgbd_dataset_freiburg2_desk
	mkdir -p $(@D)
	cd $(@D)  &&  ${WGET} "https://vision.in.tum.de/rgbd/dataset/$*.$(FILETYPE)"

./datasets/TUM/%.slam : ./datasets/TUM/%.$(FILETYPE)
	${check_generator}
	mkdir -p $(@D)/$(*F).dir
ifeq (tgz, $(FILETYPE))
	tar xzf $(@D)/$(*F).tgz -C $(@D)/$(*F).dir
else
	mkdir -p $(@D)/$(*F).dir/$(*F)
endif
	./build/bin/dataset-generator -d $(DATASET) -i $(@D)/$(*F).dir/$(*F) -o $@ -grey true -rgb true -gt true -depth true -acc true

#### ICL-NUIM ####
datasets/ICL_NUIM/living-room.ply :  datasets/ICL_NUIM/living-room.ply.tar.gz
	cd datasets/ICL_NUIM  && tar xzf living-room.ply.tar.gz
	touch datasets/ICL_NUIM/living-room.ply # This is a fix to ensure not regenerating the file again because of file create date

datasets/ICL_NUIM/living-room.ply.tar.gz :
	mkdir -p  datasets/ICL_NUIM
	cd datasets/ICL_NUIM  && ${WGET} "http://www.doc.ic.ac.uk/~ahanda/living-room.ply.tar.gz"

datasets/ICL_NUIM/%_loop.tgz :
	mkdir -p  datasets/ICL_NUIM
	cd datasets/ICL_NUIM  && ${WGET} "http://www.doc.ic.ac.uk/~ahanda/$*_loop.tgz"

datasets/ICL_NUIM/%_loop.dir :  datasets/ICL_NUIM/%_loop.tgz
	mkdir -p $@
	tar xzf $< -C $@

datasets/ICL_NUIM/living_room_traj%_loop.slam : datasets/ICL_NUIM/living_room_traj%_loop.dir datasets/ICL_NUIM/living-room.ply
	${check_generator}
	./build/bin/dataset-generator -d iclnuim -i $< -o $@ -ply  datasets/ICL_NUIM/living-room.ply -grey true -rgb true -gt true -depth true -pf true

datasets/ICL_NUIM/office_room_traj%_loop.slam : datasets/ICL_NUIM/office_room_traj%_loop.dir
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d iclnuim -i $< -o $@ -grey true -rgb true -gt true -depth true -pf true

datasets/ICL_NUIM/living_room_traj%_loop_neg.slam : datasets/ICL_NUIM/living_room_traj%_loop.dir datasets/ICL_NUIM/living-room.ply
	${check_generator}
	./build/bin/dataset-generator -d iclnuim -i $< -o $@ -ply  datasets/ICL_NUIM/living-room.ply -grey true -rgb true -gt true -depth true -pf false


datasets/ICL_NUIM/%.gt.freiburg :
	${ECHO}  "Download ground truth trajectory..."
	mkdir -p datasets/ICL_NUIM/
	cd datasets/ICL_NUIM/  &&  if test -x livingRoom$*.gt.freiburg ; then ${ECHO} "Done" ; else ${WGET} "http://www.doc.ic.ac.uk/~ahanda/VaFRIC/$*.gt.freiburg" ; fi

datasets/ICL_KLG/dyson_lab.klg :
	mkdir -p datasets/ICL_KLG/
	cd datasets/ICL_KLG/  &&  ${WGET} http://www.doc.ic.ac.uk/%7Esleutene/datasets/elasticfusion/dyson_lab.klg

#### SVO-artificial ####
datasets/SVO/artificial.tar.gz:
	mkdir -p datasets/SVO/
	cd datasets/SVO && ${WGET} -O artificial.tar.gz "http://rpg.ifi.uzh.ch/datasets/sin2_tex2_h1_v8_d.tar.gz"

datasets/SVO/artificial.dir: ./datasets/SVO/artificial.tar.gz
	mkdir -p $@
	tar -xzf $< -C $@

datasets/SVO/artificial.slam: ./datasets/SVO/artificial.dir
	${check_generator}
	./build/bin/dataset-generator -d svo -i $</sin2_tex2_h1_v8_d -o $@


#### BONN ####
./datasets/BONN/%.ply :  ./datasets/BONN/%.zip
	unzip $< -d datasets/BONN
	touch $@ # This is a fix to ensure not regenerating the file again because of file create date

./datasets/BONN/%.zip :  # Example : $* = rgbd_bonn_balloon
	mkdir -p $(@D)
	cd $(@D)  &&  ${WGET} "http://www.ipb.uni-bonn.de/html/projects/rgbd_dynamic2019/$*.zip"

./datasets/BONN/%.dir : ./datasets/BONN/%.zip
	mkdir $@
	unzip $< -d $@

./datasets/BONN/%.slam : ./datasets/BONN/%.dir ./datasets/BONN/rgbd_bonn_groundtruth_1mm_section.ply
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d bonn -i $</* -o $@ -grey true -rgb true -gt true -depth true -ply datasets/BONN/rgbd_bonn_groundtruth_1mm_section.ply

#./datasets/BONN/%.slam : ./datasets/BONN/%.dir ./datasets/BONN/rgbd_bonn_groundtruth.ply
#	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
#	./build/bin/dataset-generator -d bonn -i $</* -o $@ -grey true -rgb true -gt true -depth true -ply datasets/BONN/rgbd_bonn_groundtruth.ply


#### UZH-FPV Drone ####
 # Example : $* = indoor_foward_3_snapdragon_with_gt
./datasets/UZHFPV/%.zip : ./datasets/UZHFPV/calib/getdatasets
	echo download $*.zip
	mkdir -p $(@D)
	cd $(@D)  &&  ${WGET} "http://rpg.ifi.uzh.ch/datasets/uzh-fpv-newer-versions/v2/$*.zip"

# if contains indoor_45 | outdoor_45 | indoor_forward | outdoor_forward
./datasets/UZHFPV/%.dir : ./datasets/UZHFPV/%.zip
	mkdir -p $@
	unzip $< -d $@
	case "$(@F)" in \
				(*indoor_45*davis*) file=indoor_45_calib_davis.yaml;; \
				(*indoor_45*snap*) file=indoor_45_calib_snapdragon.yaml ;; \
				(*indoor_forward*davis*) file=indoor_forward_calib_davis.yaml ;; \
				(*indoor_forward*snap*) file=indoor_forward_calib_snapdragon.yaml ;; \
				(*outdoor_45*davis*) file=outdoor_45_calib_davis.yaml ;;\
				(*outdoor_45*snap*) file=outdoor_45_calib_snapdragon.yaml ;;\
				(*outdoor_forward*snap*) file=outdoor_forward_calib_davis.yaml ;;\
				(*outdoor_forward*davis*) file=outdoor_forward_calib_snapdragon.yaml ;;\
	esac; \
	cp $(@D)/$$file $@/sensors.yaml; \
	cp $(@D)/"imu_$$file" $@/imu.yaml

./datasets/UZHFPV/%_snapdragon_with_gt.slam:  ./datasets/UZHFPV/%_snapdragon_with_gt.dir
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d uzhfpv -i $< -o $@ -imu true --stereo true --event false -gt true

./datasets/UZHFPV/%_davis_with_gt.slam:  ./datasets/UZHFPV/%_davis_with_gt.dir
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d uzhfpv -i $< -o $@ -imu true --stereo false --event true -gt true

./datasets/UZHFPV/%_snapdragon.slam:  ./datasets/UZHFPV/%_snapdragon.dir
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d uzhfpv -i $< -o $@ -imu true --stereo true --event false -gt false

./datasets/UZHFPV/%_davis.slam:  ./datasets/UZHFPV/%_davis.dir
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d uzhfpv -i $< -o $@ -imu true --stereo false --event true -gt false

./datasets/UZHFPV/calib/getdatasets: ./datasets/UZHFPV/calib/indoor_forward_calib_snapdragon.zip \
                                     ./datasets/UZHFPV/calib/indoor_45_calib_snapdragon.zip \
                                     ./datasets/UZHFPV/calib/outdoor_forward_calib_snapdragon.zip \
                                     ./datasets/UZHFPV/calib/outdoor_45_calib_snapdragon.zip \
             						 ./datasets/UZHFPV/calib/indoor_forward_calib_davis.zip \
 									 ./datasets/UZHFPV/calib/indoor_45_calib_davis.zip \
									 ./datasets/UZHFPV/calib/outdoor_forward_calib_davis.zip \
									 ./datasets/UZHFPV/calib/outdoor_45_calib_davis.zip

./datasets/UZHFPV/calib/%.zip:
	echo download $*.zip
	mkdir -p $(@D)
	cd $(@D)  &&  ${WGET} "http://rpg.ifi.uzh.ch/datasets/uzh-fpv/calib/$*.zip"
	unzip $@ -d $(@D)
	cp $(@D)/$*/camchain-imucam-..$*_imu.yaml datasets/UZHFPV/$*.yaml
	cp $(@D)/$*/imu.yaml datasets/UZHFPV/imu_$*.yaml

#### ETH Illumination ####
./datasets/ETHI/%.zip:
	echo download $*.zip
	mkdir -p $(@D)
	cd $(@D)  &&  ${WGET} "https://cvg.ethz.ch/research/illumination-change-robust-dslam/$*.zip"

./datasets/ETHI/%.dir: ./datasets/ETHI/%.zip
	mkdir $@
	unzip $< -d $@

### TUM-based sequences contain "real", ICLNUIM-based sequences contain "syn"
### Add accelerometer.txt to prevent TUM breaking. Make sure depth.txt and rgb.txt exist in their respective folders.
./datasets/ETHI/%.slam: ./datasets/ETHI/%.dir ./datasets/ICL_NUIM/living-room.ply.tar.gz
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	for d in $</*; do \
  		echo "$$d"; \
		case "$(@F)" in \
			(*real*) touch "$$d"/accelerometer.txt; \
			 		 cp "$$d"/depth.txt "$$d"/depth/; \
			 		 cp "$$d"/rgb.txt "$$d"/rgb/; \
			 		 ./build/bin/dataset-generator -d tum -i "$$d" -o $@ -grey true -rgb true -gt true -depth true -accelerometer false ;; \
			(*syn*) cd "$$d"; \
					for file in depth/*.png; do \
						  base=`basename -- "$$file" .png`;\
						  mv "$$file" "scene_00_$$base.depth.png"; \
					done;\
					for file in rgb/*.png; do \
						  base=`basename -- "$$file"`;\
						  mv "$$file" "scene_00_$$base"; \
					done;\
					cd -; \
					./build/bin/dataset-generator -d iclnuim -i "$$d" -o $@ -ply  datasets/ICL_NUIM/living-room.ply -grey true -rgb true -gt true -depth true -pf false ;;\
		esac \
	done;

./datasets/ETHI/all:./datasets/ICL_NUIM/living-room.ply.tar.gz \
					./datasets/ETHI/ethl_real_flash.slam \
					./datasets/ETHI/ethl_real_local.slam \
					./datasets/ETHI/ethl_real_global.slam \
					./datasets/ETHI/ethl_syn1.slam \
					./datasets/ETHI/ethl_syn1_local.slam \
					./datasets/ETHI/ethl_syn1_global.slam \
					./datasets/ETHI/ethl_syn1_loc_glo.slam \
					./datasets/ETHI/ethl_syn1_flash.slam \
					./datasets/ETHI/ethl_syn2.slam \
					./datasets/ETHI/ethl_syn2_local.slam \
					./datasets/ETHI/ethl_syn2_global.slam \
					./datasets/ETHI/ethl_syn2_loc_glo.slam \
					./datasets/ETHI/ethl_syn2_flash.slam
#if echo $(@F) | grep "syn" ; then make ./datasets/ICL_NUIM/living-room.ply.tar.gz; \
#		./build/bin/dataset-generator -d iclnuim -i $</* -o $@ -ply  datasets/ICL_NUIM/living-room.ply -grey true -rgb true -gt true -depth true -pf true \


./datasets/VolumeDeform/%.dir :
	mkdir $@
	unzip $< -d $@

./datasets/VolumeDeform/%.slam : ./datasets/VolumeDeform/%.dir
	if [ ! -e ./build/bin/dataset-generator ] ; then make slambench ; fi
	./build/bin/dataset-generator -d VolumeDeform -i $< -o $@ -grey true -rgb true -gt true -depth true


#### KITTI ####
./datasets/KITTI/poses :
	mkdir -p $(@D)/poses
	cd $(@D)/poses && ${WGET} "https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_poses.zip" && unzip -j data_odometry_poses.zip -d .

./datasets/KITTI/%.zip :
	mkdir -p $(@D)
	cd $(@D) && ${WGET} "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/$*.zip"

./datasets/KITTI/%.dir : ./datasets/KITTI/%.zip ./datasets/KITTI/poses
	mkdir $@
	cp -r ./datasets/KITTI/poses $@
	unzip $< -d $@
	cd $@ && mv ./*/*/* . && rm -r ./2011_*

./datasets/KITTI/%.slam :  ./datasets/KITTI/%.dir
	${check_generator}
	./build/bin/dataset-generator -d kitti -i $< -o $@


#### NSH ####
./datasets/NSH/nsh_indoor_and_outdoor.zip :
	mkdir -p $(@D)
	cd $(@D) && ${WGET} --load-cookies /tmp/cookies.txt "https://drive.google.com/uc?export=download&confirm=$(${WGET} --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://drive.google.com/uc?export=download&id=1s1MfoRyqfsG7h8TZCAyB42125BGuA6kE' -O- | sed -rn 's/.confirm=([0-9A-Za-z_]+)./\1\n/p')&id=1s1MfoRyqfsG7h8TZCAyB42125BGuA6kE" -O nsh_indoor_and_outdoor.zip && rm -rf /tmp/cookies.txt


./datasets/NSH/nsh_indoor_and_outdoor.dir : ./datasets/NSH/nsh_indoor_and_outdoor.zip
	mkdir $@
	unzip $< -d $@

./datasets/NSH/nsh_indoor_and_outdoor.slam : ./datasets/NSH/nsh_indoor_and_outdoor.dir
	${check_generator}
	./build/bin/dataset-generator -d nsh -i $< -o $@


#### Newer College ####
./datasets/NewerCollege/newer_college_short_quad_mid.zip :
	mkdir -p $(@D)
	cd $(@D) && ${WGET} --load-cookies /tmp/cookies.txt "https://drive.google.com/uc?export=download&confirm=$(${WGET} --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://drive.google.com/uc?export=download&id=12pX0NnDKsEzROycd-XRr4weUtdJ8ql1z' -O- | sed -rn 's/.confirm=([0-9A-Za-z_]+)./\1\n/p')&id=12pX0NnDKsEzROycd-XRr4weUtdJ8ql1z" -O newer_college_short_quad_mid.zip && rm -rf /tmp/cookies.txt

./datasets/NewerCollege/newer_college_short_quad_mid.dir : ./datasets/NewerCollege/newer_college_short_quad_mid.zip
	mkdir $@
	unzip $< -d $@

./datasets/NewerCollege/newer_college_short_quad_mid.slam : ./datasets/NewerCollege/newer_college_short_quad_mid.dir
	${check_generator}
	./build/bin/dataset-generator -d newercollege -i $< -o $@


#### DARPA ####
./datasets/DARPASubt/darpa_subt_anymal_01_sync.zip :
	mkdir -p $(@D)
	cd $(@D) && ${WGET} --load-cookies /tmp/cookies.txt "https://drive.google.com/uc?export=download&confirm=$(${WGET} --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://drive.google.com/uc?export=download&id=17DCNsg8LOH-LNEeNwOV1fExr3xhOYW64' -O- | sed -rn 's/.confirm=([0-9A-Za-z_]+)./\1\n/p')&id=17DCNsg8LOH-LNEeNwOV1fExr3xhOYW64" -O darpa_subt_anymal_01_sync.zip && rm -rf /tmp/cookies.txt

./datasets/DARPASubt/darpa_subt_anymal_01_sync.dir : ./datasets/DARPASubt/darpa_subt_anymal_01_sync.zip
	mkdir $@
	unzip $< -d $@

./datasets/DARPASubt/darpa_subt_anymal_01_sync.slam : ./datasets/DARPASubt/darpa_subt_anymal_01_sync.dir
	${check_generator}
	./build/bin/dataset-generator -d darpasubt -i $< -o $@


.PRECIOUS: \
./datasets/TUM/%.tgz \
./datasets/TUM/%.dir \
./datasets/TUM/%.bag \
./datasets/TUM/%.raw \
./datasets/ICL_NUIM/living_room_traj%_loop.tgz \
./datasets/ICL_NUIM/living_room_traj%_loop.dir \
./datasets/ICL_NUIM/livingRoom%.gt.freiburg \
./datasets/ICL_NUIM/living_room_traj%_loop.raw \
./datasets/BONN/%.zip \
./datasets/BONN/%.ply \
./datasets/BONN/%.dir \
./datasets/UZHFPV/%.dir \
./datasets/UZHFPV/%.zip \
./datasets/ETHI/%.dir \
./datasets/ETHI/%.zip \
./datasets/OpenLORIS/%.dir \
./datasets/OpenLORIS/%-package.tar \
./datasets/VolumeDeform/%.dir \
./datasets/VolumeDeform/%.zip \
./datasets/KITTI/%.dir \
./datasets/KITTI/%.zip \
./datasets/NSH/nsh_indoor_and_outdoor.dir \
./datasets/NSH/nsh_indoor_and_outdoor.zip \
./datasets/NewerCollege/newer_college_short_quad_mid.dir \
./datasets/NewerCollege/newer_college_short_quad_mid.zip \
./datasets/DARPASubt/darpa_subt_anymal_01_sync.dir \
./datasets/DARPASubt/darpa_subt_anymal_01_sync.zip
