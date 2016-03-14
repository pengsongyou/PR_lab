#!/usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
from probabilistic_lib.functions import angle_wrap
import math

#===============================================================================
def splitandmerge(points, split_thres=0.1, inter_thres=0.3, min_points=6, dist_thres=0.12, ang_thres=np.deg2rad(10)):
    # The dataset2.bag parameters areAft as follow
    #def splitandmerge(points, split_thres=0.01, inter_thres=0.3, min_points=6, dist_thres=0.12, ang_thres=np.deg2rad(10)):    '''
    '''
    Takes an array of N points in shape (2, N) being the first row the x
    position and the second row the y position.

    Returns an array of lines of shape (L, 4) being L the number of lines, and
    the columns the initial and final point of each line in the form
    [x1 y1 x2 y2].

    split_thres: distance threshold to provoke a split
    inter_thres: maximum distance between consecutive points in a line
    min_point  : minimum number of points in a line
    dist_thres : maximum distance to merge lines
    ang_thres  : maximum angle to merge lines
    '''
    lines = split(points, split_thres, inter_thres, min_points, 0, points.shape[1]-1)
    return merge(lines, dist_thres, ang_thres)
    
#===============================================================================
def split(points, split_thres, inter_thres, min_points, first_pt, last_pt):
    '''
    Find lines in the points provided.
    first_pt: column position of the first point in the array
    last_pt : column position of the last point in the array
    '''
    assert first_pt >= 0
    assert last_pt <= points.shape[1]

    # Check minimum number of points
    if last_pt < min_points:
        return None
    
    # Line defined as "a*x + b*y + c = 0"
    # Calc (a, b, c) of the line (prelab question)
    x1 = points[0, first_pt]
    y1 = points[1, first_pt]
    x2 = points[0, last_pt]
    y2 = points[1, last_pt]
    
    a = y1 - y2
    b = x2 - x1
    c = x1 * y2 - x2 * y1

    # Distances of points to line (prelab question)
    ps_dis = abs(a * points[0,:] + b * points[1,:] + c) / math.sqrt(math.pow(a,2) + math.pow(b,2))#points set distance
    max_dis = np.amax(ps_dis) # Get the largest distance 
    max_ind = np.argmax(ps_dis) # The coordinate of the point having largest distance
    # Check split threshold
    if max_dis > split_thres:
       
        # Check sublines
        prev = split(points[:,first_pt:max_ind], split_thres, inter_thres, min_points, 0, points[:,first_pt:max_ind].shape[1]-1)
        post = split(points[:,(max_ind + 1) : last_pt], split_thres, inter_thres, min_points, 0, points[:,max_ind + 1 : last_pt].shape[1]-1)
        #prev = split(points[:,first_pt:max_ind-1], split_thres, inter_thres, min_points, 0, points[:,first_pt:max_ind-1].shape[1]-1)
        #post = split(points[:,(max_ind + 1) : last_pt], split_thres, inter_thres, min_points, 0, points[:,max_ind + 1 : last_pt].shape[1]-1)
       
       
        # Return results of sublines
        if prev is not None and post is not None:
            return np.vstack((prev, post))
        elif prev is not None:
            return prev
        elif post is not None:
            return post
        else:
            return None

    # Do not need to split furthermore
    else:
        # Optional check interpoint distance
        for i in range(first_pt, last_pt - 1):
            inter_dis = math.sqrt(math.pow(points[0,i] - points[0,i+1],2) + math.pow(points[1,i]-points[1,i+1],2))
            #print inter_dis
            # Check interpoint distance threshold
            if inter_dis > inter_thres:
                #Split line
                prev = split(points[:,first_pt : i], split_thres, inter_thres, min_points,0, points[:, first_pt:i].shape[1]-1)
                post = split(points[:,(i+1):last_pt], split_thres, inter_thres, min_points,0, points[:, (i+1):last_pt].shape[1]-1)
                #print points[:, (i+1):last_pt].shape[1]-1

                # Return results of sublines
                if prev is not None and post is not None:
                    return np.vstack((prev, post))
                elif prev is not None:
                    return prev
                elif post is not None:
                    return post
                else:
                    return None
        
        # It is a good line
        return np.array([[x1, y1, x2, y2]])
        
#===============================================================================
def merge(lines, dist_thres, ang_thres):
    '''
    Merge similar lines according to the given thresholds.
    '''
    # No data received
    if lines is None:
        return np.array([])
        
    # Check and merge similar consecutive lines
    i = 0
    while i in range(lines.shape[0]-1):
        
        # Line angles
        ang1 = math.atan2(lines[i,3] - lines[i,1], lines[i,2] - lines[i,0]) 
        ang2 = math.atan2(lines[i+1,3] - lines[i+1,1], lines[i+1,2] - lines[i+1,0])  
        
        # Below thresholds?
        angdiff = abs(angle_wrap(ang1-ang2))
        disdiff = math.sqrt(math.pow(lines[i,2]-lines[i+1,0],2) + math.pow(lines[i,3]-lines[i+1,1],2))
        if angdiff < ang_thres and disdiff < dist_thres:
            
            # Joined line
            lines[i,:] = np.array([lines[i,0], lines[i,1], lines[i+1,2], lines[i+1,3]])
            
            # Delete unnecessary line
            lines = np.delete(lines, i+1,0)
            
        # Nothing to merge
        else:
            i += 1
            
    return lines
