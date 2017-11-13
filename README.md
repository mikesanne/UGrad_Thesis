# UGrad_Thesis

The main functionality of this program is in this root directory

main - computes a viewset containing all of the poses of a UAV
Plot3D - Allows you to plot 4 views and their 2 sets of corresponding matched points in 3D
Plot - Plots all of the camera poses from a viewset v

Folder GroundTruth

This folder provides scripts for comparing ground truth data with estimates in terms of
roll, pitch, yaw, x, y, and z values
RPYXYZ_Cam = Generates the r,p,y,x,y,z vectors corresponding to the estimated poses
RPYXYZ_Vicon = Generates the r,p,y,x,y,z vectors corresponding to the Vicon Ground Truth Poses
LinearTime = Linear read in order to linearise the ground truth samples for image time matching
euler_to_R = function to convert euler angles into rotation matrices
Odometry = Function producing the visual odometry from the estimated RPYXYZ values s
PlotRPYXYZ = runs RPYXYZ_CAM and RPYXYZ_Vicon and plots the comparison

Folder STEWE
Contains the implementation produced by Henrik Stewenius for an improved implementation of
the five point pose estimation proposed by Nister

Folder 3rd party
PPfrome, normalisepoints, triangulat_hom = scripts used to estimate correct camera from 4 
possible P matrices
ransac5pE - An implementation of Stewe 5-point algorithm with RANSAC to improve accuracy

Folders Other Attempts
Undocumented alternative approaches to solving the same problem, including - use of SIFT features, 
use of openCV libraries, etc.