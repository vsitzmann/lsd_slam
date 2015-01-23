/*
 * PlaneEstimator.h
 *
 *  Created on: Dec 19, 2014
 *      Author: vincent
 */

#ifndef SRC_PLANEESTIMATOR_H_
#define SRC_PLANEESTIMATOR_H_

#include "sophus/sim3.hpp"
#include "Car.h"

#include "KeyFrameDisplay.h"
#include "KeyFrameGraphDisplay.h"

#include "PointCloudViewer.h"

class KeyFrameGraphDisplay;
class KeyFrameDisplay;

class PlaneEstimator {
public:
	PlaneEstimator(PointCloudViewer * viewer);
	virtual ~PlaneEstimator();

	void draw();

	void addMsg(lsd_slam_viewer::keyframeMsgConstPtr msg);
	void addImgMsg();

	void beginPlaneTracking();

	Eigen::Vector3f initialCenter;
	Eigen::Vector3f initialTangent;
	Eigen::Vector3f initialBitangent;

	Eigen::Vector3f currentTangent;
	Eigen::Vector3f currentBitangent;

	Car * car;

private:
	/***** Functions ******/
	void refreshPlane();
	void calcKeyframeCovMatrix(std::vector<Eigen::Vector3f> * keyframePointcloud, Eigen::Matrix3f &covarianceMatrix, Eigen::Vector3f & keyframeCenter);
	void initARDemo();

	/**** Variables ****/
	Eigen::Matrix3f covarianceMatrix;

	bool planeTracking;

	int totalInlierNumber;
	int keyframeUpdateTracker;

	unsigned int planeBufferId;
	int planeBufferNumPoints;
	int lastUpdateFrame;

	PointCloudViewer * viewer;
};

#endif /* SRC_PLANEESTIMATOR_H_ */
