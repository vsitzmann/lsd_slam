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
	template <typename Derived> static Eigen::Matrix3f pcaPlaneFitting(Eigen::MatrixBase<Derived> &mat, Eigen::Vector3f &tangent, Eigen::Vector3f &bitangent, Eigen::Vector3f &center){
		center = mat.colwise().mean().transpose();

		Eigen::MatrixXf centered = mat.rowwise() - center.transpose();
		Eigen::MatrixXf cov = (centered.adjoint() * centered) / float(mat.rows());

		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov);

		tangent =  eig.eigenvectors().col(2);
		bitangent =  eig.eigenvectors().col(1);

		return cov;
	};
	static Eigen::Vector4f calcPlaneParams(Eigen::Vector3f P1, Eigen::Vector3f P2, Eigen::Vector3f P3);
	static double calcPlanePointDis(Eigen::Vector4f modelParams, Eigen::Vector3f P);


private:

};

#endif /* SRC_PLANEFITTING_H_ */
