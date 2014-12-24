/*
 * PlaneEstimator.h
 *
 *  Created on: Dec 19, 2014
 *      Author: vincent
 */

#ifndef SRC_PLANEESTIMATOR_H_
#define SRC_PLANEESTIMATOR_H_

#include "sophus/sim3.hpp"

#include "KeyFrameDisplay.h"
#include "KeyFrameGraphDisplay.h"

class KeyFrameGraphDisplay;
class KeyFrameDisplay;

class PlaneEstimator {
public:
	PlaneEstimator(KeyFrameGraphDisplay * graphDisplay);
	virtual ~PlaneEstimator();

	void draw();

	void addMsg(lsd_slam_viewer::keyframeMsgConstPtr msg);
	void addImgMsg();

	void beginPlaneTracking();

private:
	/***** Functions ******/
	void refreshPlane();
	void calcKeyframeCovMatrix(std::vector<Eigen::Vector3f> * keyframePointcloud, Eigen::Matrix3f &covarianceMatrix, Eigen::Vector3f & keyframeCenter);

	/**** Variables ****/
	Eigen::Matrix3f covarianceMatrix;
	Eigen::Vector3f center;
	Eigen::Vector3f tangent;
	Eigen::Vector3f bitangent;

	bool planeTracking;

	int totalInlierNumber;
	int keyframeUpdateTracker;

	unsigned int planeBufferId;
	int planeBufferNumPoints;
	int lastUpdateFrame;

	int height;
	int width;

	KeyFrameGraphDisplay *graphDisplay;
	KeyFrameDisplay * keyFrameDisplay;
};

#endif /* SRC_PLANEESTIMATOR_H_ */
