#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Requirements: 
# sudo apt-get install python-argparse

"""
The Kinect provides the color and depth images in an un-synchronized way. This means that the set of time stamps from the color images do not intersect with those of the depth images. Therefore, we need some way of associating color images to depth images.

For this purpose, you can use the ''associate.py'' script. It reads the time stamps from the rgb.txt file and the depth.txt file, and joins them by finding the best matches.
"""

import argparse
import sys
import os
import numpy
import math


def read_file_list(filename):
    """
    Reads a trajectory from a text file. 
    
    File format:
    The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
    and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp. 
    
    Input:
    filename -- File name
    
    Output:
    dict -- dictionary of (stamp,data) tuples
    
    """
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n") 
    list = [[float(v.strip()) for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
    list = [(float(l[0]),l[1:]) for l in list if len(l)>1]
    return dict(list)

def associate(first_list, second_list,offset,max_difference):
    """
    Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we aim 
    to find the closest match for every input tuple.
    
    Input:
    first_list -- first dictionary of (stamp,data) tuples
    second_list -- second dictionary of (stamp,data) tuples
    offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
    max_difference -- search radius for candidate generation

    Output:
    matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))
    
    """
    first_keys = first_list.keys()
    second_keys = second_list.keys()
    potential_matches = [(abs(a - (b + offset)), a, b) 
                         for a in first_keys 
                         for b in second_keys 
                         if abs(a - (b + offset)) < max_difference]
    potential_matches.sort()
    matches = []
    for diff, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))
    
    matches.sort()
    return matches

def slerp(q1, q2, w1, w2):
    q1_dot_q2 = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q2[3]*q2[3]
    if (q1_dot_q2 < 0.0):
        q2 = [-v for v in q2]
        q1_dot_q2 = -q1_dot_q2
    if (q1_dot_q2 > 0.99999):
        k1 = w1
        k2 = w2
    else:
        sin_theta = math.sqrt(1.0 - q1_dot_q2 * q1_dot_q2)
        theta = math.atan2(sin_theta, q1_dot_q2)
        k1 = math.sin(w2 * theta) / sin_theta
        k2 = math.sin(w1 * theta) / sin_theta
    q = [k1 * q1[0] + k2 * q2[0], k1 * q1[1] + k2 * q2[1], k1 * q1[2] + k2 * q2[2], k1 * q1[3] + k2 * q2[3]]
    return q

def associate_with_interpolation(first_list, second_list, offset, max_difference):
    """
    Interpolate first_list according to keys in second_list
    """
    interpolated_first_list = {}
    matches = []
    first_keys = first_list.keys()
    second_keys = second_list.keys()
    first_keys.sort()
    second_keys.sort()
    idx1 = 0
    for i in range(len(second_keys)):
        key = second_keys[i] + offset
        if key < first_keys[0]:
            continue
        elif key > first_keys[-1]:
            break
        idx1 = next(i for i in range(idx1, len(first_keys) - 1) if first_keys[i] <= key and first_keys[i + 1] >= key)
        key1 = first_keys[idx1]
        key2 = first_keys[idx1 + 1]
        if max((key - key1), (key2 - key)) < max_difference:
            w2 = float(key - key1) / (key2 - key1)
            w1 = 1 - w2
            x1 = first_list.get(key1)
            x2 = first_list.get(key2)
            interpolated = [w1 * x1[0] + w2 * x2[0], w1 * x1[1] + w2 * x2[1], w1 * x1[2] + w2 * x2[2]]
            interpolated += slerp(x1[3:], x2[3:], w1, w2)
            interpolated_first_list.update({key: interpolated})
            matches.append((key, second_keys[i]))
    matches.sort()
    return matches, interpolated_first_list


if __name__ == '__main__':
    
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script takes two data files with timestamps and associates them   
    ''')
    parser.add_argument('first_file', help='first text file (format: timestamp data)')
    parser.add_argument('second_file', help='second text file (format: timestamp data)')
    parser.add_argument('--first_only', help='only output associated lines from first file', action='store_true')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02)
    args = parser.parse_args()

    first_list = read_file_list(args.first_file)
    second_list = read_file_list(args.second_file)

    matches = associate(first_list, second_list,float(args.offset),float(args.max_difference))    

    if args.first_only:
        for a,b in matches:
            print("%f %s"%(a," ".join(first_list[a])))
    else:
        for a,b in matches:
            print("%f %s %f %s"%(a," ".join(first_list[a]),b-float(args.offset)," ".join(second_list[b])))
            
        
