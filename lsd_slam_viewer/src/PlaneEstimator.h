/*
 * PlaneEstimator.h
 *
 *  Created on: Feb 10, 2015
 *      Author: vincent
 */

#ifndef SRC_PLANEESTIMATOR_H_
#define SRC_PLANEESTIMATOR_H_

#include <Eigen/Core>

#include "KeyFrameDisplay.h"
#include "KeyFrameGraphDisplay.h"
#include "PointCloudViewer.h"

class PointCloudViewer;
class KeyFrameGraphDisplay;
class KeyFrameDisplay;

class PlaneEstimator {
public:
	PlaneEstimator(PointCloudViewer  * viewer);
	virtual ~PlaneEstimator();

	void draw();

	void addMsg(lsd_slam_viewer::keyframeMsgConstPtr msg);
	void addImgMsg();

	void beginPlaneTracking();
	Eigen::Matrix4f getPlaneParameters();
	void createCollisionMap();

private:
	/**** Functions ****/
	void refreshPlane();
	void calcKeyframeCovMatrix(std::vector<Eigen::Vector3f> * keyframePointcloud, Eigen::Matrix3f &covarianceMatrix, Eigen::Vector3f & keyframeCenter);
	void initARDemo();
	void generatePlaneVBOs();

	/**** Variables ****/
	PointCloudViewer * viewer;
	std::vector<Eigen::Vector3f> inliers;

	Eigen::Matrix3f covarianceMatrix;

	bool planeTracking;

	unsigned int totalInlierNumber;
	unsigned int keyframeUpdateTracker;

	unsigned int planeBufferId;
	unsigned int inlierBufferId;
	unsigned int lastUpdateFrame;
	Eigen::Vector4f cameraViewDirection;

	std::vector<Eigen::Vector3f> plane_vertices;

	Eigen::Vector3f initialCenter;
	Eigen::Vector3f initialTangent;
	Eigen::Vector3f initialBitangent;

	Eigen::Matrix <float, 4, 4, Eigen::ColMajor> planeMatrix;



private:
	std::vector<Eigen::Vector3f> ransac(const std::vector<KeyFrameDisplay*> &keyframeDisplays, const int iterations, const float inlierDistance);
	Eigen::Vector4f calcPlaneParams(Eigen::Vector3f P1, Eigen::Vector3f P2, Eigen::Vector3f P3);
	void calcHessianParameters(Eigen::Vector3f P1, Eigen::Vector3f P2, Eigen::Vector3f P3, Eigen::Vector3f &normal, float &planeOriginDis);
	double calcPlanePointDis(Eigen::Vector3f planeNormal, float planeOriginDis, Eigen::Vector3f P);
	void selfOrganizingMap(const std::vector<KeyFrameDisplay*> &keyframeDisplays);
	Eigen::Vector2f getClosestIndex(const std::vector< std::vector <Eigen::Vector3f> > pointCloud, Eigen::Vector3f point);

	template <typename Derived> static Eigen::Matrix3f pcaPlaneFitting(Eigen::MatrixBase<Derived> &mat, Eigen::Vector3f &tangent, Eigen::Vector3f &bitangent, Eigen::Vector3f &center){
		center = mat.colwise().mean().transpose();

		Eigen::MatrixXf centered = mat.rowwise() - center.transpose();
		Eigen::MatrixXf cov = (centered.adjoint() * centered) / float(mat.rows());

		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov);

		tangent =  eig.eigenvectors().col(2);
		bitangent =  eig.eigenvectors().col(1);

		return cov;
	};

};

#endif /* SRC_PLANEESTIMATOR_H_ */
