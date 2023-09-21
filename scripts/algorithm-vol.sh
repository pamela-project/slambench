#!/bin/bash

PATH_DOCKERFILE=./docker/algorithms
WRAPPER_FILE="libslambench-c-wrapper.a"

if [ -z "$1" ]; then
    echo "No algorithm provided. Please provide an algorithm as an argument."
    exit 1
fi

echo "Making sure you have slambench/main image (check it by: docker images)"
echo -n "Do you want to build image of algorithm [y/n]: "
read CHOICE

if [[ -e $WRAPPER_FILE ]]; then
    echo "$WRAPPER_FILE exists."
else
    echo "$WRAPPER_FILE does not exist, copy it from container."
    docker run --name slambench-wrapper slambench/main
    docker cp slambench-wrapper:/slambench/build/lib/libslambench-c-wrapper.a .
fi

# Perform actions based on user input
case "$1" in
    "kfusion")
        echo "Select KFusion..."
        
        # Build image for KFusion if choice is y
        if [ "$CHOICE" = "y" ]; then
        	echo "Building the KFusion image..."
        	docker build -t kfusion-img -f $PATH_DOCKERFILE/kfusion/Dockerfile .
    	else
    		echo "No image build requested."
    	fi
        	
        echo "Populate volume for KFusion..."
        docker run -d \
            --name=kfusion \
            --mount source=kfusion-vol,destination=/deps/kfusion \
            kfusion-img
        ;;
    "lsdslam")
        echo "Select LSD-SLAM..."
        
        # Build image for LSD-SLAM if choice is y
        if [ "$CHOICE" = "y" ]; then
        	echo "Building the LSD-SLAM image..."
    		docker build -t lsdslam-img -f $PATH_DOCKERFILE/lsdslam/Dockerfile .
    	else
    		echo "No image build requested."
    	fi
    	
    	echo "Populate volume for LSD-SLAM..."
    	docker run -d \
    		--name=lsdslam \
    		--mount source=lsdslam-vol,destination=/deps/lsdslam \
    		lsdslam-img
        ;;
    "orbslam2")
        echo "Select ORB-SLAM2..."
        
        # Build image for ORB-SLAM2 if choice is y
        if [ "$CHOICE" = "y" ]; then
            echo "Building the ORB-SLAM2 image..."
            docker build -t orbslam2-img -f $PATH_DOCKERFILE/orbslam2/Dockerfile .
        else
            echo "No image build requested."
        fi
        
        echo "Populate volume for ORB-SLAM2..."
        docker run -d \
            --name=orbslam2 \
            --mount source=orbslam2-vol,destination=/deps/orbslam2 \
            orbslam2-img
        ;;
    "orbslam3")
        echo "Select ORB-SLAM3..."
        
        # Build image for ORB-SLAM3 if choice is y
        if [ "$CHOICE" = "y" ]; then
            echo "Building the ORB-SLAM3 image..."
            docker build -t orbslam3-img -f $PATH_DOCKERFILE/orbslam3/Dockerfile .
        else
            echo "No image build requested."
        fi
        
        echo "Populate volume for ORB-SLAM3..."
        docker run -d \
            --name=orbslam3 \
            --mount source=orbslam3-vol,destination=/deps/orbslam3 \
            orbslam3-img
        ;;
    "openvins")
        echo "Select Open-VINS..."
        
        # Build image for Open-VINS if choice is y
        if [ "$CHOICE" = "y" ]; then
            echo "Building the Open-VINS image..."
            docker build -t openvins-img -f $PATH_DOCKERFILE/openvins/Dockerfile .
        else
            echo "No image build requested."
        fi
        
        echo "Populate volume for Open-VINS.."
        docker run -d \
            --name=openvins \
            --mount source=openvins-vol,destination=/deps/openvins \
            openvins-img
        ;;
    "aloam")
        echo "Select A-LOAM..."
        
        # Build image for A-LOAM if choice is y
        if [ "$CHOICE" = "y" ]; then
            echo "Building the A-LOAM image..."
            docker build -t aloam-img -f $PATH_DOCKERFILE/aloam/Dockerfile .
        else
            echo "No image build requested."
        fi
        
        echo "Populate volume for A-LOAM.."
        docker run -d \
            --name=aloam \
            --mount source=aloam-vol,destination=/deps/aloam \
            aloam-img
        ;;
    "floam")
        echo "Select F-LOAM..."
        
        # Build image for F-LOAM if choice is y
        if [ "$CHOICE" = "y" ]; then
            echo "Building the F-LOAM image..."
            docker build -t floam-img -f $PATH_DOCKERFILE/floam/Dockerfile .
        else
            echo "No image build requested."
        fi
        
        echo "Populate volume for F-LOAM.."
        docker run -d \
            --name=floam \
            --mount source=floam-vol,destination=/deps/floam \
            floam-img
        ;;
    "legoloam")
        echo "Select LeGO-LOAM..."
        
        # Build image for LeGO-LOAM if choice is y
        if [ "$CHOICE" = "y" ]; then
            echo "Building the LeGO-LOAM image..."
            docker build -t legoloam-img -f $PATH_DOCKERFILE/legoloam/Dockerfile .
        else
            echo "No image build requested."
        fi
        
        echo "Populate volume for LeGO-LOAM.."
        docker run -d \
            --name=legoloam \
            --mount source=legoloam-vol,destination=/deps/legoloam \
            legoloam-img
        ;;
    *)
        echo "Invalid algorithm!"
        ;;
esac

