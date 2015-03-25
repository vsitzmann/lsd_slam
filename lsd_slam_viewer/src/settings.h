/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with dvo. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once


extern float pointTesselation;
extern float lineTesselation;

extern bool keepInMemory;
extern bool showKFCameras;
extern bool showKFPointclouds;
extern bool showConstraints;
extern bool showCurrentCamera;
extern bool showCurrentPointcloud;

extern float scaledDepthVarTH;
extern float absDepthVarTH;
extern int minNearSupport;
extern int cutFirstNKf;
extern int sparsifyFactor;
extern bool saveAllVideo;

extern int numRefreshedAlready;

extern double lastFrameTime;

extern unsigned int ransacIterations;
extern float inlierThreshold;
extern float octreeLeafSidelengthFactor;
extern bool useOcransac;
extern bool followCamera;

extern bool drawCollisionMap;
extern bool drawInlier;
extern bool drawPlane;
extern bool drawARObject;
extern bool drawOctree;
extern bool drawOctreeCell;
extern bool drawARPointcloud;
extern bool egoPerspective;

extern bool collisionChecking;
extern bool drawCollisionPoints;
extern bool arDemo;

extern int benchmarkingScene;

