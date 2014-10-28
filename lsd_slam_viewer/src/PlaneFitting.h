/*
 * PlaneFitting.h
 *
 *  Created on: Oct 18, 2014
 *      Author: vincent
 */

#ifndef SRC_PLANEFITTING_H_
#define SRC_PLANEFITTING_H_

#include <Eigen/Core>

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <iostream>
#include <ctime>
#include <vector>
#include "KeyFrameDisplay.h"

class KeyFrameDisplay;

class PlaneFitting {

public:
	static std::vector<Eigen::Vector3f> ransac(std::vector<KeyFrameDisplay*> &keyframeDisplays, int iterations, float inlierDistance);
	static void makeInlierPC(std::vector<Eigen::Vector3f> inliers, MyVertex * vertexBuffer, float color[3]);
	static Eigen::Matrix3f pcaPlaneFitting(Eigen::MatrixXf &mat, Eigen::Vector3f &tangent, Eigen::Vector3f &bitangent, Eigen::Vector3f &center);
	static Eigen::Vector4f calcPlaneParams(Eigen::Vector3f P1, Eigen::Vector3f P2, Eigen::Vector3f P3);
	static double calcPlanePointDis(Eigen::Vector4f modelParams, Eigen::Vector3f P);


private:

};

#endif /* SRC_PLANEFITTING_H_ */
