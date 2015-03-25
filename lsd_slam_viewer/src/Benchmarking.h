/*
 * Benchmarking.h
 *
 *  Created on: Feb 26, 2015
 *      Author: vincent
 */

#ifndef SRC_BENCHMARKING_H_
#define SRC_BENCHMARKING_H_

#include <Eigen/Core>
#include "PlaneEstimator.h"
#include "ARViewer.h"
#include "PlaneFittingTools.h"

class PlaneEstimator;
class ARViewer;

class MeanVarianceEstimator{
public:
	MeanVarianceEstimator();
	virtual ~MeanVarianceEstimator();
	void addValue(float value);
	float getVariance();
	float getMean();
	void reset();

	float mean;
private:
	int n;
	float M2;
};

class Benchmarking {
public:
	Benchmarking(	ARViewer *arViewer);
	virtual ~Benchmarking();

	ARViewer *arViewer;

	Eigen::Vector3f groundTruthNormal;

	unsigned int inlierNumber;
	unsigned int outlierNumber;
	unsigned int pointcloudSize;
	float sceneScale;

	unsigned int benchmarkIterations;
	unsigned int benchDownsampleFactor;

	void ransacToleranceSensitivity();
	void ransacIterationSensitivity();
	void ocRansacLeafSizeSensitivity();
	void ocRansacIterationSensitivity();
	void ocRansacLeafSizeSensitivitySidelengthBased();
	void ocRansacIterationSensitivitySidelengthBased();
	void ocRansacToleranceSensitivitySidelengthBased();
	void allOctreeBenchmarks();

private:
	void calcIterationNo();
	void assemblePointcloud(std::vector<Eigen::Vector3f> *pointcloud);
	void writeBufferToFile(std::string filename, std::string content);
	void readGroundTruthFile();
	void readConfigFile();
	float calcErrorFromInliers(const std::vector<Eigen::Vector3f> & inliers);
	std::string dateTime();


};

#endif /* SRC_BENCHMARKING_H_ */
