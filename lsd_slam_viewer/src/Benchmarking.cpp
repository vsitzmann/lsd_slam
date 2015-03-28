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
#include "settings.h"

float optimTols [] = {0.0691, 0.048, 0.065, 0.098};
float optimLeafSideLengths [] = {39, 84, 183, 111};
float octreeTestingTols[] = {0.025, 0.05, 0.075, 0.1};
float optimLeafSidelengths[] = {20, 50, 19, 50};
Eigen::Vector3f groundTruthNormals [4];

Benchmarking::Benchmarking(ARViewer * arViewer) {
	this->arViewer = arViewer;

	benchmarkIterations = 100;

	benchDownsampleFactor = 1000;

	srand(time(0));

	readGroundTruthFile();

	groundTruthNormals[0] = Eigen::Vector3f(0.0215894f, 0.794667f, 0.606662f); //Desktop
	groundTruthNormals[1] = Eigen::Vector3f(-0.125544f, 0.888503f, 0.441363f); //Room
	groundTruthNormals[2] = Eigen::Vector3f(-0.162747f, 0.657752f, 0.735443f); //Foodcourt
	groundTruthNormals[3] = Eigen::Vector3f(-0.109438f, 0.929572f, 0.352021f); //Laboratory
}

Benchmarking::~Benchmarking() {
}

void Benchmarking::allOctreeBenchmarks(){

}

void Benchmarking::ransacToleranceSensitivity(){
	std::vector<Eigen::Vector3f> pointcloud;
	assemblePointcloud(&pointcloud);

	float minTolerance = 0.0000;
	float maxTolerance = 0.26;
	float tolStepLength = 0.001;
	int benchRansacIterations = 300;
    std::unique_ptr<MeanVarianceEstimator> normalErrorStatistics(new MeanVarianceEstimator());

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
		normalErrorStatistics->reset();

		if(tol>0.005) tolStepLength = 0.005;
		if(tol>0.1) tolStepLength = 0.01;

		for(unsigned int c = 0; c<benchmarkIterations; c++){
			std::vector<Eigen::Vector3f> inliers;
			float absTolerance = tol * sceneScale;
			PlaneFittingTools::ransac(pointcloud, &inliers, benchRansacIterations, absTolerance, benchDownsampleFactor);

			if(inliers.empty()) continue;
			if(inliers.size()<0.05*pointcloud.size()) {
				littleSupport++;
			}

			float planeNormalError = calcErrorFromInliers(inliers);
			if(c>benchmarkIterations-4) std::cout<<planeNormalError<<std::endl;

			normalErrorStatistics->addValue(planeNormalError);
		}

		float variance, mean;
		variance = normalErrorStatistics->getVariance();
		mean = normalErrorStatistics->getMean();

		stream<<tol<<","<<mean<<","<<variance;

		if(littleSupport==benchmarkIterations){
			stream<<",<5% inliers support this plane";
		}
		stream<<std::endl;

		std::cout<<tol<<","<<mean<<","<<variance<<std::endl<<std::endl;
	}

	writeBufferToFile("RANSAC_tol", stream.str());
}

void Benchmarking::ransacIterationSensitivity(){
	std::vector<Eigen::Vector3f> pointcloud;
	assemblePointcloud(&pointcloud);

	unsigned int itStepLength = 1;
	unsigned int minIterations = 1;
	unsigned int maxIterations = 500;
	float relTolerance = optimTols[benchmarkingScene];	//Tolerance relative to scene scale! Will be factored with scene scale automatically.
	long double absTolerance = relTolerance * sceneScale;
	std::unique_ptr<MeanVarianceEstimator> normalErrorStatistics(new MeanVarianceEstimator());

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
		normalErrorStatistics->reset();

		if(iter==100) itStepLength = 10;

		for(unsigned int c = 0; c<benchmarkIterations; c++){
			std::vector<Eigen::Vector3f> inliers;
			PlaneFittingTools::ransac(pointcloud, &inliers, iter, absTolerance, benchDownsampleFactor);

			if(inliers.size()<0.05*pointcloud.size()) {
				littleSupport++;
				continue;
			}

			float planeNormalError = calcErrorFromInliers(inliers);
			if(c>benchmarkIterations-4) std::cout<<planeNormalError<<std::endl;

			normalErrorStatistics->addValue(planeNormalError);
		}

		float variance, mean;
		variance = normalErrorStatistics->getVariance();
		mean = normalErrorStatistics->getMean();

		stream<<iter<<","<<mean<<","<<variance;

		if(littleSupport==benchmarkIterations){
			stream<<",<5% inliers support this plane";
		}
		stream<<std::endl;

		std::cout<<iter<<","<<mean<<","<<variance<<std::endl;

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
	float relTolerance = optimTols[benchmarkingScene];	//Tolerance relative to scene scale! Will be factored with scene scale automatically.
	int benchRansacIterations = 300;
    std::unique_ptr<MeanVarianceEstimator> normalErrorStatistics(new MeanVarianceEstimator());

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
		normalErrorStatistics->reset();

		std::clock_t test = std::clock();

		delete octree;
		octree = new Octree( Eigen::Vector3f(0,0,0), PlaneFittingTools::findOutermostPoint(pointcloud), (unsigned int)(pointcloud.size()*(float)fac/100.0f));

		for ( auto &i : pointcloud ) {
			octree->insert(i);
		}

		std::cout<<"Built Octree, took "<<(std::clock()-test)/(float)CLOCKS_PER_SEC<<std::endl;

		for(unsigned int c = 0; c<benchmarkIterations; c++){
			std::vector<Eigen::Vector3f> inliers;
			float absTolerance = relTolerance * sceneScale;
			PlaneFittingTools::octreeRansac(pointcloud, *octree, &inliers, benchRansacIterations, absTolerance, benchDownsampleFactor);

			if(inliers.empty()) continue;
			if(inliers.size()<0.05*pointcloud.size()) {
				littleSupport++;
			}

			float planeNormalError = calcErrorFromInliers(inliers);
			if(c>benchmarkIterations-4) std::cout<<planeNormalError<<std::endl;

			normalErrorStatistics->addValue(planeNormalError);
		}

		float variance, mean;
		variance = normalErrorStatistics->getVariance();
		mean = normalErrorStatistics->getMean();

		stream<<fac<<","<<mean<<","<<variance;

		if(littleSupport==benchmarkIterations){
			stream<<",<5% inliers support this plane";
		}
		stream<<std::endl;

		std::cout<<fac<<","<<mean<<","<<variance<<std::endl;

	}

	delete octree;

	writeBufferToFile("OCRANSAC_leafsize", stream.str());
}

void Benchmarking::ocRansacLeafSizeSensitivitySidelengthBased(){
	std::vector<Eigen::Vector3f> pointcloud;
	assemblePointcloud(&pointcloud);

	Octree * octree = 0;

	float factorStepLength = 0.1;
	float minFactor = 0.1;
	float relTolerance = optimTols[benchmarkingScene];	//Tolerance relative to scene scale! Will be factored with scene scale automatically.
	int benchRansacIterations = 100;
	std::unique_ptr<MeanVarianceEstimator> normalErrorStatistics(new MeanVarianceEstimator());

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
		normalErrorStatistics->reset();

		std::clock_t test = std::clock();

		delete octree;
		octree = new Octree( Eigen::Vector3f(0,0,0), outermostPoint, (float)(relTolerance*fac*sceneScale));

		for ( auto &i : pointcloud ) {
			octree->insertSidelenghtBased(i);
		}

		std::cout<<"Built Octree, took "<<(std::clock()-test)/(float)CLOCKS_PER_SEC<<std::endl;

		for(unsigned int c = 0; c<benchmarkIterations; c++){
			std::vector<Eigen::Vector3f> inliers;
			float absTolerance = relTolerance * sceneScale;
			PlaneFittingTools::octreeRansac(pointcloud, *octree, &inliers, benchRansacIterations, absTolerance, benchDownsampleFactor);

			if(inliers.empty()) continue;
			if(inliers.size()<0.05*pointcloud.size()) {
				littleSupport++;
			}

			float planeNormalError = calcErrorFromInliers(inliers);
			if(c>benchmarkIterations-4) std::cout<<planeNormalError<<std::endl;

			normalErrorStatistics->addValue(planeNormalError);
		}

		float variance, mean;
		variance = normalErrorStatistics->getVariance();
		mean = normalErrorStatistics->getMean();

		stream<<fac<<","<<mean<<","<<variance;

		if(littleSupport==benchmarkIterations){
			stream<<",<5% inliers support this plane";
		}
		stream<<std::endl;

		std::cout<<fac<<","<<mean<<","<<variance<<std::endl;

	}

	delete octree;

	writeBufferToFile("OCRANSAC_leafsize_sidelength", stream.str());
}

void Benchmarking::ocRansacIterationSensitivitySidelengthBased(){
	std::vector<Eigen::Vector3f> pointcloud;
	assemblePointcloud(&pointcloud);

	unsigned int itStepLength = 1;
	unsigned int minIterations = 1;
	unsigned int maxIterations = 510;
	float relTolerance = optimTols[benchmarkingScene];	//Tolerance relative to scene scale! Will be factored with scene scale automatically.
	float cellSizeFac = optimLeafSideLengths[benchmarkingScene];
    std::unique_ptr<MeanVarianceEstimator> normalErrorStatistics(new MeanVarianceEstimator());

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
		unsigned int littleSupport = 0;
		normalErrorStatistics->reset();


		if(iter==100) itStepLength = 10;

		for(unsigned int c = 0; c<benchmarkIterations; c++){
			std::vector<Eigen::Vector3f> inliers;
			float absTolerance = relTolerance * sceneScale;
			PlaneFittingTools::octreeRansac(pointcloud, *octree, &inliers, iter, absTolerance, benchDownsampleFactor);

			float planeNormalError = calcErrorFromInliers(inliers);
			if(c>benchmarkIterations-4) std::cout<<planeNormalError<<std::endl;

			normalErrorStatistics->addValue(planeNormalError);
		}

		float variance, mean;
		variance = normalErrorStatistics->getVariance();
		mean = normalErrorStatistics->getMean();

		stream<<iter<<","<<mean<<","<<variance;

		if(littleSupport==benchmarkIterations){
			stream<<",<5% inliers support this plane";
		}
		stream<<std::endl;

		std::cout<<iter<<","<<mean<<","<<variance;
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
	float relTolerance = optimTols[benchmarkingScene];	//Tolerance relative to scene scale! Will be factored with scene scale automatically.
	float cellSizeFac = 0.32;
    std::unique_ptr<MeanVarianceEstimator> normalErrorStatistics(new MeanVarianceEstimator());

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

		normalErrorStatistics->reset();
		unsigned int littleSupport = 0;

		if(iter==100) itStepLength = 10;

		for(unsigned int c = 0; c<benchmarkIterations; c++){
			std::vector<Eigen::Vector3f> inliers;
			float absTolerance = relTolerance * sceneScale;
			PlaneFittingTools::octreeRansac(pointcloud, *octree, &inliers, iter, absTolerance, benchDownsampleFactor);

			float planeNormalError = calcErrorFromInliers(inliers);
			if(c>benchmarkIterations-4) std::cout<<planeNormalError<<std::endl;

			normalErrorStatistics->addValue(planeNormalError);
		}

		float variance, mean;
		variance = normalErrorStatistics->getVariance();
		mean = normalErrorStatistics->getMean();

		stream<<iter<<","<<mean<<","<<variance;

		if(littleSupport==benchmarkIterations){
			stream<<",<5% inliers support this plane";
		}
		stream<<std::endl;

		std::cout<<iter<<","<<mean<<","<<variance<<std::endl;

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
	float absCellSizeFac = optimLeafSidelengths[benchmarkingScene] * optimTols[benchmarkingScene];
    std::unique_ptr<MeanVarianceEstimator> normalErrorStatistics(new MeanVarianceEstimator());

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
	stream<<"Relative cell size factor:,"<<optimLeafSidelengths[benchmarkingScene]<<std::endl;
	stream<<"Downsample factor:,"<<benchDownsampleFactor<<std::endl;
	stream<<"Scene scale:,"<<sceneScale<<std::endl;
	stream<<std::endl;
	stream<<"Tolerance,avgPlaneNormalError,errorVariance"<<std::endl;

	/*** Benchmarking process. ***/
	for(float tol = minTolerance; tol<=maxTolerance; tol+=tolStepLength){
		unsigned int littleSupport = 0;
		normalErrorStatistics->reset();

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

			float planeNormalError = calcErrorFromInliers(inliers);
			if(c>benchmarkIterations-4) std::cout<<planeNormalError<<std::endl;

			normalErrorStatistics->addValue(planeNormalError);
		}

		float variance, mean;
		variance = normalErrorStatistics->getVariance();
		mean = normalErrorStatistics->getMean();

		stream<<tol<<","<<mean<<","<<variance;

		if(littleSupport==benchmarkIterations){
			stream<<",<5% inliers support this plane";
		}
		stream<<std::endl;

		std::cout<<tol<<","<<mean<<","<<variance<<std::endl<<std::endl;
	}

	delete octree;
	writeBufferToFile("OCRANSAC_tol", stream.str());
}

//Assembles the global pointcloud from all keyframe pointclouds *WITH RESPECT TO THE CHOSEN DOWNSAMPLE VALUE*
void Benchmarking::assemblePointcloud(std::vector<Eigen::Vector3f> *pointcloud){
	std::vector< KeyFrameDisplay *> keyframes = this->arViewer->pcViewer->getGraphDisplay()->getKeyframes();

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

float Benchmarking::calcErrorFromInliers(const std::vector<Eigen::Vector3f> & inliers){
	Eigen::Vector3f planeVec1, planeVec2, inlierMean;
	PlaneFittingTools::pcaPlaneFitting(inliers, &planeVec1, &planeVec2, &inlierMean);

	Eigen::Vector3f normal = planeVec1.cross(planeVec2).normalized();

	if(normal.dot(inlierMean) < 0 ) normal *= -1;
	float dotProduct = normal.dot(groundTruthNormals[benchmarkingScene]);
	if(dotProduct>1) return 0;

	float planeNormalError = acos(dotProduct)*180.0f/M_PI;

	return planeNormalError;
}


//Variance estimator as described by Donald E. Knuth (1998). The Art of Computer Programming, volume 2: Seminumerical Algorithms, 3rd edn., p. 232. Boston: Addison-Wesley.
MeanVarianceEstimator::MeanVarianceEstimator(){
	reset();
}

MeanVarianceEstimator::~MeanVarianceEstimator(){

}

float MeanVarianceEstimator::getVariance(){
	return M2/(n-1);
}

float MeanVarianceEstimator::getMean(){
	return mean;
}

void MeanVarianceEstimator::reset(){
	n = 0;
	M2 = 0.0;
	mean = 0.0;
}


void MeanVarianceEstimator::addValue(float value){

	++n;
	long double delta = value - mean;
	mean += delta/n;
	M2 += delta*(value - mean);
}





