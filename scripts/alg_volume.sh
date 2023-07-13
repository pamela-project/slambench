#!/bin/bash

PATH_DOCKERFILE=../docker/algorithms

if [ -z "$1" ]; then
    echo "No algorithm provided. Please provide an algorithm as an argument."
    exit 1
fi

echo -n "Do you want to build image of algorithm [y/n]: "
read CHOICE

# Perform actions based on user input
case "$1" in
    "kfusion")
        echo "Select KFusion..."
        
        # Build image for KFusion if choice is y
        if [ "$CHOICE" == "y" ]; then
        	echo "Building the KFusion image..."
        	docker build -t kfusion-deps $PATH_DOCKERFILE/kfusion
    	else
    		echo "No image build requested."
    	fi
        	
        echo "Populate volume for KFusion..."
        docker run -d \
            --name=kfusion \
            --mount source=kfusion-vol,destination=/deps/kfusion \
            kfusion-deps
        ;;
    "lsdslam")
        echo "Select LSD-SLAM..."
        
        # Build image for LSD-SLAM if choice is y
        if [ "$CHOICE" = "y" ]; then
        	echo "Building the LSD-SLAM image..."
    		docker build -t lsdslam-deps $PATH_DOCKERFILE/lsdslam
    	else
    		echo "No image build requested."
    	fi
    	
    	echo "Populate volume for LSD-SLAM..."
    	docker run -d \
    		--name=lsdslam \
    		--mount source=lsdslam-vol,destination=/deps/lsdslam \
    		lsdslam-deps
        ;;
    *)
        echo "Invalid algorithm!"
        ;;
esac


