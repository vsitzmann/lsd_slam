/*
 * Benchmarking.cpp
 *
 *  Created on: Feb 26, 2015
 *      Author: vincent
 */

#include "Benchmarking.h"
#include "PlaneFittingTools.h"
#include "ARViewer.h"

#include <random>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <string>

enum scenes {desktop, room, foodcourt, laboratory};
float optimTols [] = {0.0691, 0.048, 0.065, 0.098};
float optimLeafSideLengths [] = {39, 84, 183, 111};
float octreeTestingTols[] = {0.025, 0.05, 0.075, 0.1};
float optimLeafSidelengths[] = {20, 50, 19, 50};

scenes currentScene = desktop;

Benchmarking::Benchmarking(ARViewer * arViewer) {
	this->arViewer = arViewer;

	benchmarkIterations = 100;
	benchDownsampleFactor = 1000;

	srand(time(0));

	readGroundTruthFile();
}

Benchmarking::~Benchmarking() {
}

void Benchmarking::allOctreeBenchmarks(){

}

void Benchmarking::ransacToleranceSensitivity(){
	std::vector<Eigen::Vector3f> pointcloud;
	assemblePointcloud(&pointcloud);

	float minTolerance = 0.0000;
	float maxTolerance = 0.5;
	float tolStepLength = 0.005;
	int benchRansacIterations = 290;

	std::stringstream stream;

	std::cout<<"Ransac Tolerance Sensitivity Test"<<std::endl;
	stream<<"Ransac Tolerance Sensitivity Test"<<std::endl;

	stream<<"Date and time:,"<<dateTime().c_str()<<std::endl;
	stream<<"Min Tolerance:,"<<minTolerance<<std::endl;
	stream<<"Max Tolerance:,"<<maxTolerance<<std::endl;
	stream<<"Step length:,"<<tolStepLength<<std::endl;
	stream<<"Benchmarking iterations:,"<<benchmarkIterations<<std::endl;
	stream<<"Ransac iterations:,"<<ransacIterations<<std::endl;
	stream<<"Downsample factor:,"<<benchDownsampleFactor<<std::endl;
	stream<<"Scene scale:,"<<sceneScale<<std::endl;
	stream<<std::endl;

	stream<<"Tolerance,avgPlaneNormalError,error variance"<<std::endl;
	for(float tol = minTolerance; tol<=maxTolerance; tol+=tolStepLength){
		unsigned int littleSupport = 0;
		long double avgPlaneNormalError = 0;
		long double M2 = 0;
		int n = 0;

		for(unsigned int c = 0; c<benchmarkIterations; c++){
			std::vector<Eigen::Vector3f> inliers;
			float absTolerance = tol * sceneScale;
			PlaneFittingTools::ransac(pointcloud, &inliers, benchRansacIterations, absTolerance, benchDownsampleFactor);

			if(inliers.empty()) continue;
			if(inliers.size()<0.05*pointcloud.size()) {
				littleSupport++;
			}

			Eigen::Vector3f planeVec1, planeVec2, inlierMean;
			PlaneFittingTools::pcaPlaneFitting(inliers, &planeVec1, &planeVec2, &inlierMean);

			Eigen::Vector3f normal = planeVec1.cross(planeVec2).normalized();

			if(normal.dot(inlierMean) < 0 ) normal *= -1;

			long double planeNormalError = normal.dot(groundTruthNormal);
			if(c>benchmarkIterations-4) std::cout<<planeNormalError<<std::endl;

			//Variance estimator as described by Donald E. Knuth (1998). The Art of Computer Programming.
			//Iteratively calculates an estimate of the variance as well as the exact mean of a data series.
			//Since mean is calculated iteratively, a "continue" because of too few inliers does not impact the mean.
			++n;
			long double delta = planeNormalError - avgPlaneNormalError;
			avgPlaneNormalError += delta/n;
			M2 +=delta*(planeNormalError- avgPlaneNormalError);
		}

		long double variance = M2/(n-1);

		stream<<tol<<","<<avgPlaneNormalError<<","<<variance;

		if(littleSupport==benchmarkIterations){
			stream<<",<5% inliers support this plane";
		}
		stream<<std::endl;

		std::cout<<tol<<","<<avgPlaneNormalError<<","<<variance<<std::endl<<std::endl;
	}

	writeBufferToFile("RANSAC_tol", stream.str());
}

void Benchmarking::ransacIterationSensitivity(){
	std::vector<Eigen::Vector3f> pointcloud;
	assemblePointcloud(&pointcloud);

	unsigned int itStepLength = 1;
	unsigned int minIterations = 1;
	unsigned int maxIterations = 500;
	float relTolerance = 0.1;	//Tolerance relative to scene scale! Will be factored with scene scale automatically.

	std::stringstream stream;

	std::cout<<"Ransac Iterations Sensitivity Test"<<std::endl;
	stream<<"Ransac Iterations Sensitivity Test"<<std::endl;

	stream<<"Date and time:,"<<dateTime().c_str()<<std::endl;
	stream<<"Min Iterations:,"<<minIterations<<std::endl;
	stream<<"Max Iterations:,"<<maxIterations<<std::endl;
	stream<<"Step length:,"<<itStepLength<<std::endl;
	stream<<"Downsample factor:,"<<benchDownsampleFactor<<std::endl;
	stream<<"Scene scale:,"<<sceneScale<<std::endl;
	stream<<"Ransac tolerance:,"<<relTolerance<<std::endl;
	stream<<std::endl;

	stream<<"Iteration,avgPlaneNormalError,errorVariance"<<std::endl;
	for(unsigned int iter = minIterations; iter<=maxIterations; iter+=itStepLength){
		unsigned int littleSupport = 0;

		long double avgPlaneNormalError = 0;
		long double n = 0;
		long double M2 = 0;

		if(iter==100) itStepLength = 10;

		for(unsigned int c = 0; c<benchmarkIterations; c++){
			std::vector<Eigen::Vector3f> inliers;
			long double absTolerance = relTolerance * sceneScale;
			PlaneFittingTools::ransac(pointcloud, &inliers, iter, absTolerance, benchDownsampleFactor);

			if(inliers.size()<0.05*pointcloud.size()) {
				littleSupport++;
				continue;
			}

			Eigen::Vector3f planeVec1, planeVec2, inlierMean;
			PlaneFittingTools::pcaPlaneFitting(inliers, &planeVec1, &planeVec2, &inlierMean);

			HessianNormalForm plane;
			plane.normal = planeVec1.cross(planeVec2).normalized();
			plane.originDis = plane.normal.dot(inlierMean);

			if( (plane).normal.dot(inlierMean) < 0 ) (plane).normal *= -1;

			long double planeNormalError = plane.normal.dot(groundTruthNormal);
			if(c>benchmarkIterations-4) std::cout<<planeNormalError<<std::endl;

			//Variance estimator as described by Donald E. Knuth (1998). The Art of Computer Programming, volume 2: Seminumerical Algorithms, 3rd edn., p. 232. Boston: Addison-Wesley.
			++n;
			long double delta = planeNormalError - avgPlaneNormalError;
			avgPlaneNormalError += delta/n;
			M2 +=delta*(planeNormalError- avgPlaneNormalError);
		}

		long double variance = M2/(n-1);

		stream<<iter<<","<<avgPlaneNormalError<<","<<variance;

		if(littleSupport==benchmarkIterations){
			stream<<",<5% inliers support this plane";
		}
		stream<<std::endl;

		std::cout<<iter<<","<<avgPlaneNormalError<<","<<variance<<std::endl;

	}

	writeBufferToFile("RANSAC_iter", stream.str());
}

void Benchmarking::ocRansacLeafSizeSensitivity(){
	std::vector<Eigen::Vector3f> pointcloud;
	assemblePointcloud(&pointcloud);

	Octree * octree = 0;

	float factorStepLength = 0.1;
	float minFactor = 0.1;
	float maxFactor = 100;
	float relTolerance = optimTols[currentScene];	//Tolerance relative to scene scale! Will be factored with scene scale automatically.
	int benchRansacIterations = 300;

	std::stringstream stream;

	std::cout<<"Octree Leafsize Sensitivity Test"<<std::endl;
	stream<<"Octree Leafsize Sensitivity Test"<<std::endl;

	stream<<"Date and time:,"<<dateTime().c_str()<<std::endl;
	stream<<"Min Divisor:,"<<minFactor<<std::endl;
	stream<<"Max Divisor:,"<<maxFactor<<std::endl;
	stream<<"Step length:,"<<factorStepLength<<std::endl;
	stream<<"Relative tolerance:,"<<relTolerance<<std::endl;
	stream<<"Benchmarking iterations:,"<<benchmarkIterations<<std::endl;
	stream<<"Downsample factor:,"<<benchDownsampleFactor<<std::endl;
	stream<<"Scene scale:,"<<sceneScale<<std::endl;
	stream<<std::endl;

	stream<<"Iteration,avgPlaneNormalError,errorVariance"<<std::endl;
	for(float fac = minFactor; fac<=maxFactor; fac+=factorStepLength){
		if(fac>=1) factorStepLength = 1;

		unsigned int littleSupport = 0;

		std::clock_t test = std::clock();

		delete octree;
		octree = new Octree( Eigen::Vector3f(0,0,0), PlaneFittingTools::findOutermostPoint(pointcloud), (unsigned int)(pointcloud.size()*(float)fac/100.0f));

		for ( auto &i : pointcloud ) {
			octree->insert(i);
		}

		std::cout<<"Built Octree, took "<<(std::clock()-test)/(float)CLOCKS_PER_SEC<<std::endl;

		long double avgPlaneNormalError = 0;
		int n = 0;
		long double M2 = 0;

		for(unsigned int c = 0; c<benchmarkIterations; c++){
			std::vector<Eigen::Vector3f> inliers;
			float absTolerance = relTolerance * sceneScale;
			PlaneFittingTools::octreeRansac(pointcloud, *octree, &inliers, benchRansacIterations, absTolerance, benchDownsampleFactor);

			if(inliers.empty()) continue;
			if(inliers.size()<0.05*pointcloud.size()) {
				littleSupport++;
			}

			Eigen::Vector3f planeVec1, planeVec2, inlierMean;
			PlaneFittingTools::pcaPlaneFitting(inliers, &planeVec1, &planeVec2, &inlierMean);
			Eigen::Vector3f normal = planeVec1.cross(planeVec2).normalized();
			if( normal.dot(inlierMean) < 0 ) normal *= -1;

			long double planeNormalError = normal.dot(groundTruthNormal);
			if(c>benchmarkIterations-4) std::cout<<planeNormalError<<std::endl;

			//Variance estimator as described by Donald E. Knuth (1998). The Art of Computer Programming, volume 2: Seminumerical Algorithms, 3rd edn., p. 232. Boston: Addison-Wesley.
			++n;
			float delta = planeNormalError - avgPlaneNormalError;
			avgPlaneNormalError += delta/n;
			M2 +=delta*(planeNormalError- avgPlaneNormalError);
		}

		long double variance = M2/(n-1);

		stream<<fac<<","<<avgPlaneNormalError<<","<<variance;

		if(littleSupport==benchmarkIterations){
			stream<<",<5% inliers support this plane";
		}
		stream<<std::endl;

		std::cout<<fac<<","<<avgPlaneNormalError<<","<<variance<<std::endl;

	}

	delete octree;

	writeBufferToFile("OCRANSAC_leafsize", stream.str());
}

void Benchmarking::ocRansacLeafSizeSensitivitySidelengthBased(){
	std::vector<Eigen::Vector3f> pointcloud;
	assemblePointcloud(&pointcloud);

	Octree * octree = 0;

	for(int d = 0; d<4; d++){
		float factorStepLength = 0.1;
		float minFactor = 0.1;
		float relTolerance = octreeTestingTols[d];	//Tolerance relative to scene scale! Will be factored with scene scale automatically.
		int benchRansacIterations = 20;

		Eigen::Vector3f outermostPoint = PlaneFittingTools::findOutermostPoint(pointcloud);
		float outermostPointNorm = outermostPoint.norm();
		float maxFactor = outermostPointNorm / (sceneScale * relTolerance);

		std::stringstream stream;

		std::cout<<"Octree Leafsize Sensitivity Test - Sidelength-based"<<std::endl;
		stream<<"Octree Leafsize Sensitivity Test - Sidelength-based"<<std::endl;

		stream<<"Date and time:,"<<dateTime().c_str()<<std::endl;
		stream<<"Min factor:,"<<minFactor<<std::endl;
		stream<<"Max factor:,"<<maxFactor<<std::endl;
		stream<<"Step length:,"<<factorStepLength<<std::endl;
		stream<<"Relative tolerance:,"<<relTolerance<<std::endl;
		stream<<"Ransac iterations:,"<<benchRansacIterations<<std::endl;
		stream<<"Downsample factor:,"<<benchDownsampleFactor<<std::endl;
		stream<<"Scene scale:,"<<sceneScale<<std::endl;
		stream<<std::endl;

		stream<<"Iteration,avgPlaneNormalError,errorVariance"<<std::endl;

		for(float fac = minFactor; fac<=maxFactor; fac+=factorStepLength){
			if(fac>=1) factorStepLength = 1;

			unsigned int littleSupport = 0;

			std::clock_t test = std::clock();

			delete octree;
			octree = new Octree( Eigen::Vector3f(0,0,0), outermostPoint, (float)(relTolerance*fac*sceneScale));

			for ( auto &i : pointcloud ) {
				octree->insertSidelenghtBased(i);
			}

			std::cout<<"Built Octree, took "<<(std::clock()-test)/(float)CLOCKS_PER_SEC<<std::endl;

			long double avgPlaneNormalError = 0;
			int n = 0;
			long double M2 = 0;

			for(unsigned int c = 0; c<benchmarkIterations; c++){
				std::vector<Eigen::Vector3f> inliers;
				float absTolerance = relTolerance * sceneScale;
				PlaneFittingTools::octreeRansac(pointcloud, *octree, &inliers, benchRansacIterations, absTolerance, benchDownsampleFactor);

				if(inliers.empty()) continue;
				if(inliers.size()<0.05*pointcloud.size()) {
					littleSupport++;
				}

				Eigen::Vector3f planeVec1, planeVec2, inlierMean;
				PlaneFittingTools::pcaPlaneFitting(inliers, &planeVec1, &planeVec2, &inlierMean);
				Eigen::Vector3f normal = planeVec1.cross(planeVec2).normalized();
				if( normal.dot(inlierMean) < 0 ) normal *= -1;

				long double planeNormalError = normal.dot(groundTruthNormal);
				if(c>benchmarkIterations-4) std::cout<<planeNormalError<<std::endl;

				//Variance estimator as described by Donald E. Knuth (1998). The Art of Computer Programming, volume 2: Seminumerical Algorithms, 3rd edn., p. 232. Boston: Addison-Wesley.
				++n;
				float delta = planeNormalError - avgPlaneNormalError;
				avgPlaneNormalError += delta/n;
				M2 +=delta*(planeNormalError- avgPlaneNormalError);
			}

			long double variance = M2/(n-1);

			stream<<fac<<","<<avgPlaneNormalError<<","<<variance;

			if(littleSupport==benchmarkIterations){
				stream<<",<5% inliers support this plane";
			}
			stream<<std::endl;

			std::cout<<fac<<","<<avgPlaneNormalError<<","<<variance<<std::endl;

		}

		delete octree;

		writeBufferToFile("OCRANSAC_leafsize_sidelength", stream.str());
	}
}

void Benchmarking::ocRansacIterationSensitivitySidelengthBased(){
	std::vector<Eigen::Vector3f> pointcloud;
	assemblePointcloud(&pointcloud);

	unsigned int itStepLength = 1;
	unsigned int minIterations = 1;
	unsigned int maxIterations = 500;
	float relTolerance = optimTols[currentScene];	//Tolerance relative to scene scale! Will be factored with scene scale automatically.
	float cellSizeFac = optimLeafSideLengths[currentScene];

	Octree * octree = new Octree( Eigen::Vector3f(0,0,0), PlaneFittingTools::findOutermostPoint(pointcloud), (float)(cellSizeFac*sceneScale*relTolerance));
	for ( auto &i : pointcloud ) {
		octree->insertSidelenghtBased(i);
	}

	std::stringstream stream;

	/*** Print the header lines ***/
	std::cout<<"Octree Iterations Sensitivity Test - Sidelength-Based"<<std::endl;
	stream<<"Octree Iterations Sensitivity Test- Sidelength-Based"<<std::endl;

	stream<<"Date and time:,"<<dateTime().c_str()<<std::endl;
	stream<<"Min Iterations:,"<<minIterations<<std::endl;
	stream<<"Max Iterations:,"<<maxIterations<<std::endl;
	stream<<"Step length:,"<<itStepLength<<std::endl;
	stream<<"Benchmarking iterations:,"<<benchmarkIterations<<std::endl;
	stream<<"Cell size factor:,"<<cellSizeFac<<std::endl;
	stream<<"Downsample factor:,"<<benchDownsampleFactor<<std::endl;
	stream<<"Scene scale:,"<<sceneScale<<std::endl;
	stream<<std::endl;
	stream<<"Iteration,avgPlaneNormalError,errorVariance"<<std::endl;

	/*** Benchmarking process. ***/
	for(unsigned int iter = minIterations; iter<=maxIterations; iter+=itStepLength){
		float avgPlaneNormalError = 0;
		int n = 0;
		float M2 = 0;

		unsigned int littleSupport = 0;

		if(iter==100) itStepLength = 10;

		for(unsigned int c = 0; c<benchmarkIterations; c++){
			std::vector<Eigen::Vector3f> inliers;
			float absTolerance = relTolerance * sceneScale;
			PlaneFittingTools::octreeRansac(pointcloud, *octree, &inliers, iter, absTolerance, benchDownsampleFactor);


			Eigen::Vector3f planeVec1, planeVec2, inlierMean;
			PlaneFittingTools::pcaPlaneFitting(inliers, &planeVec1, &planeVec2, &inlierMean);

			HessianNormalForm plane;
			plane.normal = planeVec1.cross(planeVec2).normalized();
			plane.originDis = plane.normal.dot(inlierMean);

			if( (plane).normal.dot(inlierMean) < 0 ) (plane).normal *= -1;

			float planeNormalError = plane.normal.dot(groundTruthNormal);
			if(c>benchmarkIterations-4) std::cout<<planeNormalError<<std::endl;

			//Variance estimator as described by Donald E. Knuth (1998). The Art of Computer Programming, volume 2: Seminumerical Algorithms, 3rd edn., p. 232. Boston: Addison-Wesley.
			++n;
			float delta = planeNormalError - avgPlaneNormalError;
			avgPlaneNormalError += delta/n;
			M2 +=delta*(planeNormalError- avgPlaneNormalError);
		}

		float variance = M2/(n-1);

		stream<<iter<<","<<avgPlaneNormalError<<","<<variance;

		if(littleSupport==benchmarkIterations){
			stream<<",<5% inliers support this plane";
		}
		stream<<std::endl;

		std::cout<<iter<<","<<avgPlaneNormalError<<","<<variance<<std::endl;

	}

	delete octree;

	writeBufferToFile("OCRANSAC_iter", stream.str());
}


void Benchmarking::ocRansacIterationSensitivity(){
	std::vector<Eigen::Vector3f> pointcloud;
	assemblePointcloud(&pointcloud);

	unsigned int itStepLength = 1;
	unsigned int minIterations = 1;
	unsigned int maxIterations = 500;
	float relTolerance = optimTols[currentScene];	//Tolerance relative to scene scale! Will be factored with scene scale automatically.
	float cellSizeFac = 0.32;

	Octree * octree = new Octree( Eigen::Vector3f(0,0,0), PlaneFittingTools::findOutermostPoint(pointcloud), (unsigned int)(pointcloud.size()*cellSizeFac));
	for ( auto &i : pointcloud ) {
		octree->insert(i);
	}

	std::stringstream stream;

	/*** Print the header lines ***/
	std::cout<<"Octree Iterations Sensitivity Test"<<std::endl;
	stream<<"Octree Iterations Sensitivity Test"<<std::endl;

	stream<<"Date and time:,"<<dateTime().c_str()<<std::endl;
	stream<<"Min Iterations:,"<<minIterations<<std::endl;
	stream<<"Max Iterations:,"<<maxIterations<<std::endl;
	stream<<"Step length:,"<<itStepLength<<std::endl;
	stream<<"Benchmarking iterations:,"<<benchmarkIterations<<std::endl;
	stream<<"Cell size factor:,"<<cellSizeFac<<std::endl;
	stream<<"Downsample factor:,"<<benchDownsampleFactor<<std::endl;
	stream<<"Scene scale:,"<<sceneScale<<std::endl;
	stream<<std::endl;
	stream<<"Iteration,avgPlaneNormalError,errorVariance"<<std::endl;

	/*** Benchmarking process. ***/
	for(unsigned int iter = minIterations; iter<=maxIterations; iter+=itStepLength){
		float avgPlaneNormalError = 0;
		int n = 0;
		float M2 = 0;

		unsigned int littleSupport = 0;

		if(iter==100) itStepLength = 10;

		for(unsigned int c = 0; c<benchmarkIterations; c++){
			std::vector<Eigen::Vector3f> inliers;
			float absTolerance = relTolerance * sceneScale;
			PlaneFittingTools::octreeRansac(pointcloud, *octree, &inliers, iter, absTolerance, benchDownsampleFactor);


			Eigen::Vector3f planeVec1, planeVec2, inlierMean;
			PlaneFittingTools::pcaPlaneFitting(inliers, &planeVec1, &planeVec2, &inlierMean);

			HessianNormalForm plane;
			plane.normal = planeVec1.cross(planeVec2).normalized();
			plane.originDis = plane.normal.dot(inlierMean);

			if( (plane).normal.dot(inlierMean) < 0 ) (plane).normal *= -1;

			float planeNormalError = plane.normal.dot(groundTruthNormal);
			if(c>benchmarkIterations-4) std::cout<<planeNormalError<<std::endl;

			//Variance estimator as described by Donald E. Knuth (1998). The Art of Computer Programming, volume 2: Seminumerical Algorithms, 3rd edn., p. 232. Boston: Addison-Wesley.
			++n;
			float delta = planeNormalError - avgPlaneNormalError;
			avgPlaneNormalError += delta/n;
			M2 +=delta*(planeNormalError- avgPlaneNormalError);
		}

		float variance = M2/(n-1);

		stream<<iter<<","<<avgPlaneNormalError<<","<<variance;

		if(littleSupport==benchmarkIterations){
			stream<<",<5% inliers support this plane";
		}
		stream<<std::endl;

		std::cout<<iter<<","<<avgPlaneNormalError<<","<<variance<<std::endl;

	}

	delete octree;
	writeBufferToFile("OCRANSAC_iter", stream.str());
}

void Benchmarking::ocRansacToleranceSensitivitySidelengthBased(){
	std::vector<Eigen::Vector3f> pointcloud;
	assemblePointcloud(&pointcloud);

	float minTolerance = 0.0000;
	float maxTolerance = 0.26;
	float tolStepLength = 0.001;
	int benchRansacIterations = 300;
	float absCellSizeFac = optimLeafSidelengths[currentScene] * optimTols[currentScene];

	Octree * octree = new Octree( Eigen::Vector3f(0,0,0), PlaneFittingTools::findOutermostPoint(pointcloud), (float)(absCellSizeFac*sceneScale));
	for ( auto &i : pointcloud ) {
		octree->insertSidelenghtBased(i);
	}

	std::stringstream stream;

	/*** Print the header lines ***/
	std::cout<<"ORANSAC Tolerance Sensitivity Test - Sidelength-based"<<std::endl;
	stream<<"ORANSAC Tolerance Sensitivity Test - Sidelength-based"<<std::endl;

	stream<<"Date and time:,"<<dateTime().c_str()<<std::endl;
	stream<<"Min tolerance:,"<<minTolerance<<std::endl;
	stream<<"Max minTolerance:,"<<minTolerance<<std::endl;
	stream<<"Step length:,"<<minTolerance<<std::endl;
	stream<<"Benchmarking iterations:,"<<benchmarkIterations<<std::endl;
	stream<<"Absolute cell size factor:,"<<absCellSizeFac<<std::endl;
	stream<<"Relative cell size factor:,"<<optimLeafSidelengths[currentScene]<<std::endl;
	stream<<"Downsample factor:,"<<benchDownsampleFactor<<std::endl;
	stream<<"Scene scale:,"<<sceneScale<<std::endl;
	stream<<std::endl;
	stream<<"Tolerance,avgPlaneNormalError,errorVariance"<<std::endl;

	/*** Benchmarking process. ***/
	for(float tol = minTolerance; tol<=maxTolerance; tol+=tolStepLength){
		unsigned int littleSupport = 0;
		long double avgPlaneNormalError = 0;
		long double M2 = 0;
		int n = 0;

		if(tol>0.005) tolStepLength = 0.005;
		if(tol>0.1) tolStepLength = 0.01;

		for(unsigned int c = 0; c<benchmarkIterations; c++){
			std::vector<Eigen::Vector3f> inliers;
			float absTolerance = tol * sceneScale;
			PlaneFittingTools::octreeRansac(pointcloud, *octree, &inliers, benchRansacIterations, absTolerance, benchDownsampleFactor);

			if(inliers.empty()) continue;
			if(inliers.size()<0.05*pointcloud.size()) {
				littleSupport++;
			}

			Eigen::Vector3f planeVec1, planeVec2, inlierMean;
			PlaneFittingTools::pcaPlaneFitting(inliers, &planeVec1, &planeVec2, &inlierMean);

			Eigen::Vector3f normal = planeVec1.cross(planeVec2).normalized();

			if(normal.dot(inlierMean) < 0 ) normal *= -1;

			long double planeNormalError = normal.dot(groundTruthNormal);
			if(c>benchmarkIterations-4) std::cout<<planeNormalError<<std::endl;

			//Variance estimator as described by Donald E. Knuth (1998). The Art of Computer Programming.
			//Iteratively calculates an estimate of the variance as well as the exact mean of a data series.
			//Since mean is calculated iteratively, a "continue" because of too few inliers does not impact the mean.
			++n;
			long double delta = planeNormalError - avgPlaneNormalError;
			avgPlaneNormalError += delta/n;
			M2 +=delta*(planeNormalError- avgPlaneNormalError);
		}

		long double variance = M2/(n-1);

		stream<<tol<<","<<avgPlaneNormalError<<","<<variance;

		if(littleSupport==benchmarkIterations){
			stream<<",<5% inliers support this plane";
		}
		stream<<std::endl;

		std::cout<<tol<<","<<avgPlaneNormalError<<","<<variance<<std::endl<<std::endl;
	}

	delete octree;
	writeBufferToFile("RANSAC_tol", stream.str());
}

//Assembles the global pointcloud from all keyframe pointclouds *WITH RESPECT TO THE CHOSEN DOWNSAMPLE VALUE*
void Benchmarking::assemblePointcloud(std::vector<Eigen::Vector3f> *pointcloud){
	std::vector< KeyFrameDisplay *> keyframes = this->arViewer->viewer->getGraphDisplay()->getKeyframes();

	for(unsigned int i =0; i<keyframes.size(); i++)
	{
		pointcloud->insert(pointcloud->end(), keyframes[i]->getKeyframePointcloud()->begin(), keyframes[i]->getKeyframePointcloud()->end());
	}

	std::cout<<"Pointcloud size: "<<pointcloud->size()<<std::endl;

	//Calculate the scene scale
	float maxDistance = 0;
	for(unsigned int i =1; i<keyframes.size(); i+=2)
	{
		float distance = (keyframes[i]->camToWorld.translation() - keyframes[i-1]->camToWorld.translation()).norm();

		if(distance>maxDistance)
			maxDistance = distance;
	}

	sceneScale = maxDistance;
}



void Benchmarking::writeBufferToFile(std::string filename, std::string content){
	filename = (dateTime()).append(filename).append(".csv");

	  ofstream buffer;
	  buffer.open (filename.c_str());

	  buffer<<content.c_str()<<std::endl;

	  buffer.close();
}

void Benchmarking::readGroundTruthFile() {
	std::string line;
	ifstream buffer ("groundTruth.txt");

	  if (buffer.is_open()) {
		for(int i =0; i<3; i++) {
			getline (buffer,line);
			groundTruthNormal[i] =atof(line.c_str());
		}

		buffer.close();

		std::cout<<"Ground truth normal: "<<groundTruthNormal<<std::endl;
	  } else std::cout << "Unable to open ground truth file";
}

void Benchmarking::readConfigFile() {
	std::string line;
	ifstream buffer ("config.txt");

	  if (buffer.is_open()) {
		getline (buffer,line);
		benchmarkIterations = atoi(line.c_str());
		buffer.close();
	  } else std::cout << "Unable to open config file";
}

std::string Benchmarking::dateTime(){
   // current date/time based on current system
   time_t now = time(0);

   // convert now to tm struct for UTC
   tm *gmtm = gmtime(&now);
   char * dt = asctime(gmtm);

   std::string str(dt);

   return str;
}
