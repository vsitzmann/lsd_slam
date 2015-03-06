/*
 * PlaneFittingTools.h
 *
 *  Created on: Feb 26, 2015
 *      Author: vincent
 */

#ifndef SRC_PLANEFITTINGTOOLS_H_
#define SRC_PLANEFITTINGTOOLS_H_

#include <Eigen/Core>
#include <Eigen/Dense>

#include "Octree.h"

struct HessianNormalForm {
	Eigen::Vector3f normal;
	float originDis;
};

typedef Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic, Eigen::RowMajor> MatrixXfrm;

class PlaneFittingTools {
public:
	static void ransac(const std::vector<Eigen::Vector3f> &pointcloud, std::vector<Eigen::Vector3f> * inliers, unsigned int iterations, float inlierTolerance, unsigned int scoreDownsampling);
	static void octreeRansac(const std::vector<Eigen::Vector3f> &pointcloud, Octree &octree, Eigen::Vector3f * cellIdentifier, std::vector<Eigen::Vector3f> * inliers, unsigned  int iterations, float inlierTolerance, unsigned int scoreDownsampling);

	static float calcSignedPlanePointDis(const HessianNormalForm &plane, const Eigen::Vector3f &P);
	static Eigen::Vector3f findOutermostPoint(const std::vector<Eigen::Vector3f> & pointcloud);
	static void calcHessianParameters(const Eigen::Vector3f & P1, const Eigen::Vector3f & P2, const Eigen::Vector3f & P3, HessianNormalForm * plane);
	static float calcPlanePointDis(const HessianNormalForm & plane, const Eigen::Vector3f & P);
	static void findInliers(const std::vector<Eigen::Vector3f> &pointCloud, std::vector<Eigen::Vector3f> * inliers, float inlierTolerance, HessianNormalForm plane);
	static Eigen::Matrix3f pcaPlaneFitting(const std::vector<Eigen::Vector3f> & dataSamples, Eigen::Vector3f * tangent, Eigen::Vector3f * bitangent, Eigen::Vector3f * center);
	static void buildOctree(const std::vector<Eigen::Vector3f> &pointcloud, Octree * octree, int downsampleFactor, int maxCellPoints);
	static float sumOfSquaredErrors(const std::vector<Eigen::Vector3f> & pointcloud, const HessianNormalForm &plane);

};

#endif /* SRC_PLANEFITTINGTOOLS_H_ */
