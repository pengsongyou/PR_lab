#!/usr/bin/python
# -*- coding: utf-8 -*-

from math import *
import numpy as np
import math
from scipy.linalg import block_diag
from probabilistic_lib.functions import angle_wrap, comp, state_inv, state_inv_jacobian
import rospy

#============================================================================
class EKF_SLAM(object):
    '''
    Class to hold the whole EKF-SLAM.
    '''
    
    #========================================================================
    def __init__(self, x0,y0,theta0, odom_lin_sigma, 
                 odom_ang_sigma, meas_rng_noise, meas_ang_noise):
        '''
        Initializes the ekf filter
        room_map : an array of lines in the form [x1 y1 x2 y2]
        num      : number of particles to use
        odom_lin_sigma: odometry linear noise
        odom_ang_sigma: odometry angular noise
        meas_rng_noise: measurement linear noise
        meas_ang_noise: measurement angular noise
        '''
        
        # Copy parameters
        self.odom_lin_sigma = odom_lin_sigma
        self.odom_ang_sigma = odom_ang_sigma
        self.meas_rng_noise = meas_rng_noise
        self.meas_ang_noise = meas_ang_noise
        self.chi_thres = 0.7 # TODO chose your own value
       
        # Odometry uncertainty 
        self.Qk = np.array([[ self.odom_lin_sigma**2, 0, 0],\
                            [ 0, self.odom_lin_sigma**2, 0 ],\
                            [ 0, 0, self.odom_ang_sigma**2]])
        
        # Measurements uncertainty
        self.Rk=np.eye(2)
        self.Rk[0,0] = self.meas_rng_noise
        self.Rk[1,1] = self.meas_ang_noise
        
        # State vector initialization
        self.xk = np.array([x0,y0,theta0]) # Position
        self.Pk = np.zeros((3,3)) # Uncertainty
        
        # Initialize buffer for forcing observing n times a feature before 
        # adding it to the map
        self.featureObservedN = np.array([])
        self.min_observations = 5
    
    #========================================================================
    def get_number_of_features_in_map(self):
        '''
        returns the number of features in the map
        '''
        return (self.xk.size-3)/2
    
    #========================================================================
    def get_polar_line(self, line, odom):
        '''
        Transforms a line from [x1 y1 x2 y2] from the world frame to the
        vehicle frame using odometry [x y ang].
        Returns [range theta]
        '''
        # Line points
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        
        # Compute line (a, b, c) and range
        line = np.array([y1-y2, x2-x1, x1*y2-x2*y1])
        pt = np.array([odom[0], odom[1], 1])
        dist = np.dot(pt, line) / np.linalg.norm(line[:2])
        
        # Compute angle
        if dist < 0:
            ang = np.arctan2(line[1], line[0])
        else:
            ang = np.arctan2(-line[1], -line[0])
        
        # Return in the vehicle frame
        return np.array([np.abs(dist), angle_wrap(ang - odom[2])])
        
    #========================================================================
    def predict(self, uk):
        
        '''
        Predicts the position of the robot according to the previous position and the odometry measurements. It also updates the uncertainty of the position
        '''
        # - Update self.xk and self.Pk using uk and self.Qk
            
        # Compound robot with odometry
        a = comp(self.xk[0:3],uk)
        self.xk = np.append(a, self.xk[3:])

        # Compute jacobians of the composition with respect to robot (A_k) 
        # and odometry (W_k)
        A_k = np.array([[1,0, - math.sin(self.xk[2]) * uk[0] - math.cos(self.xk[2]) * uk[1]],[0,1,math.cos(self.xk[2]) * uk[0] - math.sin(self.xk[2]) * uk[1]],[0,0,1]])
        W_k = np.array([[math.cos(self.xk[2]), - math.sin(self.xk[2]),0],[math.sin(self.xk[2]),math.cos(self.xk[2]),0],[0,0,1]])

        # Prepare the F_k and G_k matrix for the new uncertainty computation
        n = self.get_number_of_features_in_map() # number of features in the map or call it in the state vector

        F_k = np.eye(3 + 2*n, 3 + 2*n)
        G_k = np.zeros((3 + 2*n, 3))

        F_k[0:3, 0:3] = A_k
        G_k[0:3, 0:3] = W_k
        
        tmp = self.Pk
        Pk = np.zeros((3 + 2*n, 3 + 2*n))
        Pk[0:self.Pk.shape[0], 0:self.Pk.shape[1]] = tmp
        self.Pk = Pk

        # Compute uncertainty
        P1 = np.dot(F_k,self.Pk)
        P1 = np.dot(P1,np.transpose(F_k)) 
        P2 = np.dot(G_k,self.Qk)
        P2 = np.dot(P2,np.transpose(G_k))

        self.Pk = P1 + P2

    #========================================================================
        
    def data_association(self, lines):
        '''
        Implements ICCN for each feature of the scan.
        '''
    
        # fore each sensed line do:
        #   1- Transform the sened line to polar
        #   2- for each feature of the map (in the state vector) compute the 
        #      mahalanobis distance
        #   3- Data association
        
        # Init variable
        
        N = self.xk.shape[0]
        Innovk_List   = np.zeros((0,0))
        H_k_List      = np.zeros((0,N))
        Rk_List       = np.zeros((0,0))
        #idx_not_associated = np.array(range(lines.shape[0]))
        idx_not_associated = np.zeros((0,1))

        
        for i in range(0, lines.shape[0]):
            # Transfer the sensend lines to robot polar line system
            Pr = self.get_polar_line(lines[i],[0,0,0])
            minD = 1e9 
            minj = -1

            for j in range(3,self.xk.shape[0],2):
                z = Pr 
                D,v,h,H = self.lineDist(z,j)
                if np.sqrt(D) < minD:
                    minj = j
                    minz = z
                    minh = h
                    minH = H
                    minv = v
                    minD = D
                
            if minD < self.chi_thres: # current sensored line has already been associated 

                # H_k_List = np.vstack((H_k_List, minH)) # Finally should be 2m * (3 + 2n)
                # Innovk_List = np.append(Innovk_List, minv) # Finally should be 2m * 1
                # Rk_List = block_diag(Rk_List, self.Rk) 

                # The optional part
                if self.featureObservedN[(minj - 3)/2 -1] >= self.min_observations:
                    H_k_List = np.vstack((H_k_List, minH)) # Finally should be 2m * (3 + 2n)
                    Innovk_List = np.append(Innovk_List, minv) # Finally should be 2m * 1
                    Rk_List = block_diag(Rk_List, self.Rk) 
                else: 
                    self.featureObservedN[(minj - 3)/2 -1] += 1

            else: 
                idx_not_associated = np.append(idx_not_associated,i)
                   
        print idx_not_associated
        return Innovk_List, H_k_List, Rk_List, idx_not_associated
        
    #========================================================================
    def update_position(self, Innovk_List, H_k_List, Rk_List) :
        '''
        Updates the position of the robot according to the given the position
        and the data association parameters.
        Returns state vector and uncertainty.
        
        '''
        # Program this function
        if Innovk_List.shape[0]<1:
            return
            
        # Kalman Gain
        S = np.dot(np.dot(H_k_List,self.Pk),np.transpose(H_k_List)) + Rk_List
        K = np.dot(np.dot(self.Pk, np.transpose(H_k_List)), np.linalg.inv(S))# Key of Kalman filter
        # Update Position
        self.xk += np.dot(K,Innovk_List)
        # Update Uncertainty
        I = np.eye(self.xk.shape[0])
        u1 = I - np.dot(K,H_k_List)
        self.Pk = np.dot(np.dot(u1, self.Pk),np.transpose(u1)) + np.dot(np.dot(K,Rk_List),np.transpose(K))

    #========================================================================
    def state_augmentation(self, lines, idx):
        '''
        given the whole set of lines read by the kineckt sensor and the
        indexes that have not been associated augment the state vector to 
        introduce the new features
        '''
        # If no features to add to the map exit function
        if idx.size<1:
            return

        # TODO Program this function
        for i in idx:
            # Transfer the lines to robot polar line system
            self.featureObservedN = np.append(self.featureObservedN,1) 
            
            Pr = self.get_polar_line(lines[i],[0,0,0])
            
            # Transfer the robot polar line system to the world

            # [self.xk[0], self.xk[1], self.xk[2]] is the robot position in the world frame
            Pw, H_position, H_line = self.tfPolarLine([self.xk[0],self.xk[1],self.xk[2]], Pr)

            # Add new line Jacobian into the Uncertainty
            N = len(self.xk) # N = 3 + 2n

            # F should be (3 + (2n + 2)) * (3 + 2n)

            F_k_ = np.eye(N) # First (3 * 2n) * (3 * 2n) matrix should be identity matrix 
            if N < 4 :
                F_k = np.vstack((F_k_, H_position))
            else:
                new = np.zeros((2, N))
                new[0:2, 0:3] = H_position                
                F_k = np.vstack((F_k_, new)) # (3 + 2n + 2) * (3 + 2n)

            G_k = np.zeros((N + 2, 2))# (3 + 2n + 2) * 2
            G_k[N : N + 2, 0 : 2] = H_line  
            
            # Add new lines into the state vector
            self.xk = np.append(self.xk, Pw)            
            
            self.Pk = np.dot(np.dot(F_k, self.Pk),np.transpose(F_k)) + np.dot(np.dot(G_k, self.Rk), np.transpose(G_k)) 
    #========================================================================
    def tfPolarLine(self,tf,line):
        '''
        Transforms a polar line in the robot frame to a polar line in the
        world frame
        '''
        # Decompose transformation
        # Decompose transformation
        x_x = tf[0]
        x_y = tf[1]
        x_ang = tf[2]  
        
        # Compute the new phi
        phi = angle_wrap(line[1] + x_ang)
        
        rho_ = line[0] + x_x * np.cos(phi) + x_y * np.sin(phi)
        sign = 1
        if rho_ <0:
            rho_ = -rho_
            phi = angle_wrap(phi+pi)   
            sign = -1
        
        # Allocate jacobians
        H_tf = np.zeros((2,3))
        H_line = np.eye(2)

        # Evaluate jacobian respect to transformation
        F1 = sign*np.cos(phi)
        F2 = sign*np.sin(phi)
        F3 = -sign*x_x * np.sin(phi) + sign*x_y * np.cos(phi)
        H_tf = np.array([[F1,F2,F3],[0,0,1]])

        # Evaluate jacobian respect to line
        H_line = np.array([[sign, F3],[0, 1]])        
        return np.array([rho_,phi]), H_tf, H_line
                
    #========================================================================
    def lineDist(self,z,idx):
        '''
        Given a line and an index of the state vector it computes the
        distance between both lines
        '''        
                
        # Transform the map line into robot frame and compute jacobians
        x_w_r = state_inv(self.xk[0:3]) #world origin in robot frame
        J_inv = state_inv_jacobian(self.xk[0:3])
        h, H_position, H_line = self.tfPolarLine(x_w_r, [self.xk[idx],self.xk[idx+1]]) # w.r.t. robot frame
        
        # Allocate overall jacobian

        N = len(self.xk) # N = 3 + 2n
        H = np.zeros((2, N))

        # Concatenate position jacobians and place them into the position
        H[:, 0:3] = H_position 
        
        # Place the position of the jacobina with respec to the line in its
        # position  
        H[:, idx : idx + 2] = H_line 

        # Calculate innovation
        v = z - h

        # Calculate innovation uncertainty
        G = np.zeros((N,N))
        G[0:3,0:3] = J_inv
        G[3:N, 3:N] = np.eye(N-3, N-3)
        Pk = np.dot(np.dot(G,self.Pk),np.transpose(G))
        S = np.dot(np.dot(H, Pk),np.transpose(H)) + self.Rk
        

        # Calculate mahalanobis distance
        D = np.dot(np.dot(np.transpose(v), np.linalg.inv(S)), v)
        
        H = np.dot(H,G)
        return D,v,h,H
