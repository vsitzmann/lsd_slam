/*
 * PlaneFittingTools.cpp
 *
 *  Created on: Feb 26, 2015
 *      Author: vincent
 */

#include "PlaneFittingTools.h"
#include "Octree.h"
#include <ctime>
#include <Eigen/Core>
#include <iostream>
#include <cmath>
#include <random>

typedef Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic, Eigen::RowMajor> MatrixXfrm;

void PlaneFittingTools::ransac(const std::vector<Eigen::Vector3f> &pointcloud, std::vector<Eigen::Vector3f> * inliers, unsigned int iterations,float inlierTolerance, unsigned int scoreDownsampling){
	unsigned int bestPoints[3] = {0};
	unsigned int maxInliers = 0;
	unsigned int pcSize = pointcloud.size();

	for (unsigned int c = 0; c<iterations; c++){
		int pos1, pos2, pos3;
		pos1 = rand() % pcSize;
		pos2 = rand() % pcSize;
		pos3 = rand() % pcSize;

		// ensure all points are different
		while (pos1 == pos2 || pos2 == pos3 || pos1 == pos3) {
			pos1 = rand() % pcSize;
			pos2 = rand() % pcSize;
			pos3 = rand() % pcSize;
		}

		Eigen::Vector3f P1, P2, P3;
		P1 = (pointcloud)[pos1];
		P2 = (pointcloud)[pos2];
		P3 = (pointcloud)[pos3];

		HessianNormalForm plane;
		calcHessianParameters(P1, P2, P3, &plane);

		unsigned int inlier = 0;
		for(unsigned int j = 0; j<pointcloud.size(); j+=scoreDownsampling) {
			double dis = calcPlanePointDis(plane, pointcloud[j]);

			if (dis < inlierTolerance){
				inlier++;
			}
		}

		if (inlier > maxInliers) {
			maxInliers = inlier;
			bestPoints[0] = pos1;
			bestPoints[1] = pos2;
			bestPoints[2] = pos3;
		}
	}

	Eigen::Vector3f P1, P2, P3;
	P1 = (pointcloud)[bestPoints[0]];
	P2 = (pointcloud)[bestPoints[1]];
	P3 = (pointcloud)[bestPoints[2]];

	HessianNormalForm plane;
	calcHessianParameters(P1, P2, P3, &plane);

	inliers->clear();
	inliers->reserve(maxInliers);
	findInliers(pointcloud, inliers, inlierTolerance, plane);
}

void PlaneFittingTools::octreeRansac(const std::vector<Eigen::Vector3f> &pointcloud, Octree &octree, std::vector<Eigen::Vector3f> * inliers, unsigned int iterations, float inlierTolerance, unsigned int scoreDownsampling){
	unsigned int maxInliers = 0;
	Eigen::Vector3f bP1 (0,0,0);
	Eigen::Vector3f bP2 (0,0,0);
	Eigen::Vector3f bP3 (0,0,0);

	for (unsigned  int c = 0; c<iterations; c++){
		/** Select a random tree leaf by selecting a random point from the PC and finding its containing leaf **/
		unsigned int temp = rand() % pointcloud.size();
		Eigen::Vector3f P1 = pointcloud[temp];
		Octree * currentLeaf = octree.getLeafContainingPoint(P1);

		unsigned int inlier = 0;
		unsigned int leafPCSize = currentLeaf->data.size();
		if(leafPCSize<3) { continue;}

		// get two more random points from the current octree leaf.
		unsigned int pos2, pos3;
		pos2 = rand()%leafPCSize;
		pos3 = rand()%leafPCSize;

		Eigen::Vector3f P2, P3;
		P2 = currentLeaf->data[pos2];
		P3 = currentLeaf->data[pos3];

		// ensure all points are different
		while (P1 == P2 || P2 == P3 || P1 == P3) {
			P2 = currentLeaf->data[rand()%leafPCSize];
			P3 = currentLeaf->data[rand()%leafPCSize];
		}

		HessianNormalForm plane;
		calcHessianParameters(P1, P2, P3, &plane);

		for(unsigned int j = 0; j<pointcloud.size(); j+=scoreDownsampling){
			float dis = calcPlanePointDis(plane, pointcloud[j]);

			if (dis < inlierTolerance){
				inlier++;
			}
		}

		if (inlier > maxInliers) {
			maxInliers = inlier;
			bP1 = P1;
			bP2 = P2;
			bP3 = P3;
		}
	}

	HessianNormalForm plane;
	calcHessianParameters(bP1, bP2, bP3, &plane);

	inliers->clear();
	inliers->reserve(maxInliers);
	findInliers(pointcloud, inliers, inlierTolerance, plane);
}

void PlaneFittingTools::findInliers(const std::vector<Eigen::Vector3f> &pointCloud, std::vector<Eigen::Vector3f> * inliers, float inlierTolerance, HessianNormalForm plane){
	for ( auto &i : pointCloud ) {
		double dis = calcPlanePointDis(plane, i);

		if (dis < inlierTolerance){
			inliers->push_back(i);
		}
	}
}

Eigen::Vector3f PlaneFittingTools::findOutermostPoint(const std::vector<Eigen::Vector3f> & pointcloud){
	float maxValue = 0;

	for (unsigned int i =0; i<pointcloud.size(); i++) {
		Eigen::Vector3f point = pointcloud[i];

		float currentMax = std::max(std::abs(point.x()), std::abs(point.y()));
		currentMax = std::max(currentMax, std::abs( point.z() ) );

		if(currentMax > maxValue){
			maxValue = currentMax;
		}
	}

	Eigen::Vector3f result (maxValue, maxValue, maxValue);

	return result;
}


void PlaneFittingTools::calcHessianParameters(const Eigen::Vector3f & P1, const Eigen::Vector3f &P2, const Eigen::Vector3f &P3, HessianNormalForm * plane){
	plane->normal = (P2 - P1).cross(P3 - P1).normalized();

	if( plane->normal.dot(P1) < 0 ) plane->normal *= -1;

	plane->originDis = P1.dot( plane->normal );
}

float PlaneFittingTools::calcPlanePointDis(const HessianNormalForm &plane, const Eigen::Vector3f &P){
	return std::abs(plane.normal.dot(P) - plane.originDis);
}

float PlaneFittingTools::calcSignedPlanePointDis(const HessianNormalForm &plane, const Eigen::Vector3f &P){
	return plane.normal.dot(P) - plane.originDis;
}

float PlaneFittingTools::sumOfSquaredErrors(const std::vector<Eigen::Vector3f> & pointcloud, const HessianNormalForm &plane){
	float sum = 0;

	for ( auto &i : pointcloud ) {
		sum += std::pow(calcPlanePointDis(plane, i), 2);
	}

	return sum;
}

Eigen::Matrix3f PlaneFittingTools::pcaPlaneFitting(const std::vector<Eigen::Vector3f> & dataSamples, Eigen::Vector3f * tangent, Eigen::Vector3f * bitangent, Eigen::Vector3f * center){
	MatrixXfrm mat ((int)dataSamples.size(), 3);
	mat = Eigen::Map<MatrixXfrm> ((float*)dataSamples.data(), (int)dataSamples.size(), 3);

	*center = mat.colwise().mean().transpose();

	Eigen::MatrixXf centered = mat.rowwise() - center->transpose();
	Eigen::MatrixXf cov = (centered.adjoint() * centered) / float(mat.rows());

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov);

	*tangent =  eig.eigenvectors().col(2);
	*bitangent =  eig.eigenvectors().col(1);

	return cov;
}

Eigen::Vector3f PlaneFittingTools::projectPoint(const Eigen::Vector3f &point, const HessianNormalForm & plane){
	return point - calcSignedPlanePointDis(plane, point)*plane.normal;
}

Eigen::Vector4f PlaneFittingTools::projectPoint(const Eigen::Vector4f &point, const HessianNormalForm & plane){
	Eigen::Vector4f homogenNormal = Eigen::Vector4f::Zero();
	homogenNormal.topRows(3) = plane.normal;

	return point - calcSignedPlanePointDis(plane, (Eigen::Vector3f)point.topRows(3))*homogenNormal;
}
