/*
 * PlaneFitting.cpp
 *
 *  Created on: Oct 18, 2014
 *      Author: vincent
 */

#include "PlaneFitting.h"

#include "KeyFrameDisplay.h"
#include "settings.h"

#include <iostream>


//takes vector of KeyFrameDisplay's and returns a matrix of inliers on the dominant plane.

std::vector<Eigen::Vector3f> PlaneFitting::ransac(std::vector<KeyFrameDisplay*> &keyframeDisplays, int iterations, float inlierDistance){

	if(debugMode)
	{
		std::cerr<<__PRETTY_FUNCTION__<<std::endl;
		printf("keyframe Diplay size: %lu\n", keyframeDisplays.size());
	}


	int absolutePointNumber = 0;


	for(unsigned int i=0;i<keyframeDisplays.size();i++)
	{
		absolutePointNumber+=keyframeDisplays[i]->keyframePointcloud.size();
	}

	if(debugMode)
		printf("Ransac will run on a total number of %d points\n", absolutePointNumber);

	std::vector<Eigen::Vector3f> globalPointCloud;
	std::vector<Eigen::Vector3f> inliers;
	globalPointCloud.reserve(absolutePointNumber);

	if(debugMode)
		printf("joining keyframe pointclouds to one pointcloud...\n");

	for(unsigned int i =0; i<keyframeDisplays.size(); i++)
	{
		globalPointCloud.insert(globalPointCloud.end(), keyframeDisplays[i]->keyframePointcloud.begin(), keyframeDisplays[i]->keyframePointcloud.end());
	}

	int maxInliers = 0, inlier = 0;
	int pos1, pos2, pos3;
	Eigen::Vector3f P1, P2, P3, P;
	double dis;
	double bestPoints[3] = {0};
	Eigen::Vector4f modelParameters;

	for (int i=0; i < iterations; i++)
	{
		if(debugMode)
			std::cerr<<__PRETTY_FUNCTION__<<" Ransac iteration "<< i << " out of " << iterations <<std::endl;

		// get Random Points
		pos1 = rand() % absolutePointNumber;
		pos2 = rand() % absolutePointNumber;
		pos3 = rand() % absolutePointNumber;

		// ensure all points are different
		while (pos1 == pos2 || pos2 == pos3 || pos1 == pos3) {
			pos1 = rand() % absolutePointNumber;
			pos2 = rand() % absolutePointNumber;
			pos3 = rand() % absolutePointNumber;
		}

		P1 = globalPointCloud[pos1];
		P2 = globalPointCloud[pos2];
		P3 = globalPointCloud[pos3];

		modelParameters = calcPlaneParams(P1, P2, P3);

		if(debugMode)
					std::cerr<<__PRETTY_FUNCTION__<<" Identify inliers..." << std::endl;
		inlier = 0;

		int j = 0;

		while(j < absolutePointNumber){
			// calculate distance between point and plane
			P = globalPointCloud[j];
			dis = calcPlanePointDis(modelParameters, P);

			if (dis < inlierDistance){
				inlier++;
			}

			j+=downsampleFactor;
		}

		if (inlier > maxInliers) {
			maxInliers = inlier;
			bestPoints[0] = pos1;
			bestPoints[1] = pos2;
			bestPoints[2] = pos3;
		}
	}

	if(debugMode)
			std::cerr<<__PRETTY_FUNCTION__<<" Best model identified. Calculating best model parameters... " << std::endl;

	P1 = globalPointCloud[bestPoints[0]];
	P2 = globalPointCloud[bestPoints[1]];
	P3 = globalPointCloud[bestPoints[2]];

	modelParameters = calcPlaneParams(P1, P2, P3);

	if(debugMode)
		std::cerr<<__PRETTY_FUNCTION__<<" Identifying final inliers. " << std::endl;

	for (int j = 0; j < absolutePointNumber; j++) {
		// calculate distance between point and plane
		P = globalPointCloud[j];

		double dis = calcPlanePointDis(modelParameters, P);

		if (dis < inlierDistance){
			inliers.push_back(globalPointCloud[j]);
		}
	}

	return inliers;
}

double PlaneFitting::calcPlanePointDis(Eigen::Vector4f modelParams, Eigen::Vector3f P){
	return std::abs((double)(modelParams[0]*P[0] + modelParams[1]*P[1] + modelParams[2]*P[2] + modelParams[3])) /
			sqrt((double)(modelParams[0]*modelParams[0] + modelParams[1]*modelParams[1] + modelParams[2]*modelParams[2]));

}

Eigen::Vector4f PlaneFitting::calcPlaneParams(Eigen::Vector3f P1, Eigen::Vector3f P2, Eigen::Vector3f P3){
	Eigen::Vector4f buffer;

	buffer[0] = P1[1] * (P2[2] - P3[2]) + P2[1] * (P3[2] - P1[2]) + P3[1] * (P1[2] - P2[2]);
	buffer[1] = P1[2] * (P2[0] - P3[0]) + P2[2] * (P3[0] - P1[0]) + P3[2] * (P1[0] - P2[0]);
	buffer[2] = P1[0] * (P2[1] - P3[1]) + P2[0] * (P3[1] - P1[1]) + P3[0] * (P1[1] - P2[1]);
	buffer[3] = -(P1[0] * (P2[1] * P3[2] - P3[1] * P2[2]) + P2[0] * (P3[1] * P1[2] - P1[1] * P3[2]) + P3[0] * (P1[1] * P2[2] - P2[1] * P1[2]));

	return buffer;
}

void PlaneFitting::makeInlierPC(std::vector<Eigen::Vector3f> inliers, MyVertex * vertexBuffer, float color[3]){
	for(unsigned int i = 0; i<inliers.size(); i++)
	{
		vertexBuffer[i].point[0] = inliers[i][0];
		vertexBuffer[i].point[1] = inliers[i][1];
		vertexBuffer[i].point[2] = inliers[i][2];

		vertexBuffer[i].color[3] = 100;
		vertexBuffer[i].color[2] = color[0];
		vertexBuffer[i].color[1] = color[1];
		vertexBuffer[i].color[0] = color[2];

		//	printf("Pointnumber: %d, x: %f, y: %f, z: %f\n", pointsToDraw[i].point[0],pointsToDraw[i].point[1], pointsToDraw[i].point[2]);
	}
}


