
####################################
####       DOCKER PART          ####
####################################

## sudo service docker start

SLAMBENCH2_DIRECTORY=../

.PHONY: ./%-docker-image %-docker-image

./tmp/%-docker-image/DockerFile : ./%.docker
	mkdir -p ./tmp/$*-docker-image/slambench2
	rm -rf ./tmp/$*-docker-image/slambench2/*
	cp -r ${SLAMBENCH2_DIRECTORY}/CMakeLists.txt ${SLAMBENCH2_DIRECTORY}/benchmarks ${SLAMBENCH2_DIRECTORY}/cmake_modules ${SLAMBENCH2_DIRECTORY}/framework ${SLAMBENCH2_DIRECTORY}/Makefile ./tmp/$*-docker-image/slambench2
	cp -rf $< ./tmp/$*-docker-image/DockerFile
	cp -f nvidia-driver.run ./tmp/$*-docker-image/

fedora24 :
	make fedora-24-docker-image

ubuntu14 :
	make ubuntu-14.04-docker-image

ubuntu16 :
	make ubuntu-16.04-docker-image




nvidia_version=$(shell cat /proc/driver/nvidia/version | head -n 1 | awk '{ print $$8 }')

nvidia-driver.run :
	wget -O nvidia-driver.run "http://us.download.nvidia.com/XFree86/Linux-x86_64/${nvidia_version}/NVIDIA-Linux-x86_64-${nvidia_version}.run"

cuda-ubuntu16 : nvidia-driver.run
	make cuda-ubuntu-16.04-docker-image

%-docker-image: ./%.docker ./tmp/%-docker-image/DockerFile
	docker build -f ./tmp/$*-docker-image/DockerFile -t  bbodin/slambench:$*  ./tmp/$*-docker-image
