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
#include "Octree.h"
#include "PlaneFittingTools.h"

class PointCloudViewer;
class KeyFrameGraphDisplay;
class KeyFrameDisplay;
class Octree;

class PlaneEstimator {
public:
	PlaneEstimator(PointCloudViewer  * viewer);
	virtual ~PlaneEstimator();

	void flipPlane();
	void draw();
	void beginPlaneTracking(const Eigen::Vector3f & cameraCoordinates);
	Eigen::Matrix4f getPlaneMatrix();
	HessianNormalForm getPlane();

	/*** Collision Map ****/
	bool checkCollision(const Eigen::Vector4f &position);

	/*** Toggle & get/set functions ***/
	void toggleConsensusSetDraw();
	void toggleCollisionDraw();
	void togglePlaneDraw();
	void togglePlaneUpdate();
	float getSceneScale();

/*** Private variables ***/
private:
	PointCloudViewer * viewer;
	std::vector<Eigen::Vector3f> consensusSet;

	Eigen::Matrix3f covarianceMatrix;
	Eigen::Vector3f initialCenter;
	Eigen::Vector3f initialTangent;
	Eigen::Vector3f initialBitangent;

	bool planeUpdating;
	bool planeTracking;

	float sceneScale;

	unsigned int planeBufferId;
	unsigned int inlierBufferId;

	unsigned int keyframeUpdateTracker;
	unsigned int lastUpdateFrame;

	bool planeValid;
	bool planeBuffersValid;

	std::vector<Eigen::Vector3f> plane_vertices;

	HessianNormalForm currentPlane;
	Eigen::Matrix <float, 4, 4, Eigen::ColMajor> planeMatrix;

	/*** Collision Map ****/
	std::vector<Eigen::Vector3f> collisionInliers;
	unsigned int collisionMapSize;
	std::vector< std::vector<int> > collisionMap;

	/*** Octree ***/
	Octree * octree;
	std::vector<Eigen::Vector3f> octreeCell;

/*** Private functions ****/
private:
	void refreshPlane();
	void generatePlaneVBOs();
	void calcSceneScale();

	/** Consensus set identification **/
	void ransacConsensusSetID(const std::vector<Eigen::Vector3f> &pointcloud);
	void ocRansacConsensusSetID(const std::vector<Eigen::Vector3f> &pointcloud);

	/*** Collision Map ***/
	void initCollisionMap();
	void createCollisionMap(const Eigen::Vector3f & cameraCoordinates);
};

#endif /* SRC_PLANEESTIMATOR_H_ */
