/*
 * PlaneEstimator.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: vincent
 */

#define GL_GLEXT_PROTOTYPES 1

#include "PlaneEstimator.h"
#include "PointCloudViewer.h"
#include "PlaneFittingTools.h"

#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <ctime>

#include "settings.h"
#include "Octree.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

typedef Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic, Eigen::RowMajor> MatrixXfrm;

bool isRunning = false;

int counter = 0;

PlaneEstimator::PlaneEstimator(PointCloudViewer * viewer) {
	// TODO Auto-generated constructor stub
	generatePlaneVBOs();
	collisionMapSize = 100;

	planeInlierBufferValid = false;
	octreeBufferValid = false;
	collisionBufferValid = false;

	octree = 0;
	sceneScale = 0;

	this->viewer = viewer;
}

PlaneEstimator::~PlaneEstimator() {
	delete octree;
}

void PlaneEstimator::flipPlane() {
	planeMatrix.col(2) = -1 * planeMatrix.col(2);
}

void PlaneEstimator::generatePlaneVBOs(){
	float triangleWidth = 0.1;
	float triangleHeight = 0.1;
	Eigen::Vector3f virtualCenter = -25*triangleWidth*Eigen::Vector3f(0,1,0)-25*triangleHeight*Eigen::Vector3f(1,0,0);

	while(plane_vertices.size()<5000){
		virtualCenter += triangleHeight * Eigen::Vector3f(1, 0, 0);

		for(int i = 0; i<50; i++)
		{
			for(int j = 1; j>=0; j--)
			{
				Eigen::Vector3f buffer;

				buffer=virtualCenter + triangleWidth * i * Eigen::Vector3f(0,1,0) + triangleHeight * j * Eigen::Vector3f(1,0,0);

				plane_vertices.push_back(buffer);
			}
		}
	}
}

void PlaneEstimator::draw() {

	if(planeTracking){
		refreshPlane();

		//Since the inliers always change together with the plane, only one bool variable is
		//required.
		if(!planeInlierBufferValid){
			//Since glDeleteBuffers ignores a bufferID of zero, it is no problem to delete
			//a buffer that has not yet been loaded with data yet.
			glDeleteBuffers(1, &planeBufferId);
			planeBufferId = 0;
			glGenBuffers(1, &planeBufferId);
			glBindBuffer(GL_ARRAY_BUFFER, planeBufferId);
			glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f) * plane_vertices.size(), plane_vertices.data(), GL_STATIC_DRAW);

			glDeleteBuffers(1, &inlierBufferId);
			inlierBufferId = 0;
			glGenBuffers(1, &inlierBufferId);
			glBindBuffer(GL_ARRAY_BUFFER, inlierBufferId);
			glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f) * consensusSet.size(), consensusSet.data(), GL_STATIC_DRAW);

			planeInlierBufferValid = true;
		}

		if(!collisionBufferValid){
			glDeleteBuffers(1, &collisionBufferId);
			collisionBufferId = 0;
			glGenBuffers(1, &collisionBufferId);
			glBindBuffer(GL_ARRAY_BUFFER, collisionBufferId);         // for vertex coordinates
			glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f) * collisionInliers.size(), collisionInliers.data(), GL_STATIC_DRAW);
			collisionBufferValid = true;
		}

		if(!octreeBufferValid){
			glDeleteBuffers(1, &octreeBufferId);
			octreeBufferId = 0;
			glGenBuffers(1, &octreeBufferId);
			glBindBuffer(GL_ARRAY_BUFFER, octreeBufferId);         // for vertex coordinates
			glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f) * octreeCell.size(), octreeCell.data(), GL_STATIC_DRAW);
			octreeBufferValid = true;
		}

		if(drawPlane){
			glMatrixMode(GL_MODELVIEW);

			glPushMatrix();
				glMultMatrixf(planeMatrix.data());

				glEnableClientState(GL_VERTEX_ARRAY);

				  glBindBuffer(GL_ARRAY_BUFFER, planeBufferId);
					glVertexPointer(3, GL_FLOAT, sizeof(Eigen::Vector3f), 0);
					glColor3f(1, 1, 1);
					glDrawArrays(GL_LINE_STRIP, 0, plane_vertices.size());

				  glDisableClientState(GL_VERTEX_ARRAY);

			glPopMatrix();
		}

		glEnableClientState(GL_VERTEX_ARRAY);

			//For better visibility, all special points are drawn in a distinctive color and
			//a larger size.
			glPointSize(pointTesselation * 2);

			if(drawInlier){
				glBindBuffer(GL_ARRAY_BUFFER, inlierBufferId);
				glVertexPointer(3, GL_FLOAT, sizeof(Eigen::Vector3f), 0);
				glColor3f(0, 0, 1);
				glDrawArrays(GL_POINTS, 0, consensusSet.size());
			}

			if(drawCollisionMap){
				glBindBuffer(GL_ARRAY_BUFFER, collisionBufferId);
				glVertexPointer(3, GL_FLOAT, sizeof(Eigen::Vector3f), 0);
				glColor3f(1, 0, 0);
				glDrawArrays(GL_POINTS, 0, collisionInliers.size());
			}

			if(drawOctreeCell){
				glBindBuffer(GL_ARRAY_BUFFER, octreeBufferId);
				glVertexPointer(3, GL_FLOAT, sizeof(Eigen::Vector3f), 0);
				glColor3f(0, 1, 0);
				glDrawArrays(GL_POINTS, 0, octreeCell.size());
			}

			glPointSize(pointTesselation);

		glDisableClientState(GL_VERTEX_ARRAY);

		glColor3f(1, 1, 1);

		if(drawOctree && octree!=0){
			octree->drawOctree();
		}

	}
}

void PlaneEstimator::beginPlaneTracking(const Eigen::Vector3f & cameraCoordinates){
	std::vector< KeyFrameDisplay *> keyframes = viewer->getGraphDisplay()->getKeyframes();

	calcSceneScale();

	if(keyframes.empty()) return;
	std::vector<Eigen::Vector3f> globalPointCloud;
	for(unsigned int i =0; i<keyframes.size(); i++)
	{
		globalPointCloud.insert(globalPointCloud.end(), keyframes[i]->getKeyframePointcloud()->begin(), keyframes[i]->getKeyframePointcloud()->end());
	}

	std::cout<<"Total number of points: "<<globalPointCloud.size()<<std::endl;

	std::clock_t timer = clock();
	consensusSet.clear();

	if(useOcransac)ocRansacConsensusSetID(globalPointCloud);
	else ransacConsensusSetID(globalPointCloud);

	if(consensusSet.empty()) return;
	std::cout<<"RANSAC runtime: "<<(clock()-timer)/(float)CLOCKS_PER_SEC<<std::endl;

	std::cout<<"Size of consensus set: "<<consensusSet.size()<<std::endl<<std::endl;

	covarianceMatrix = PlaneFittingTools::pcaPlaneFitting(consensusSet, &initialTangent, &initialBitangent, &initialCenter);

	planeMatrix = Eigen::Matrix4f::Identity();
	planeMatrix.col(3).topRows(3) = initialCenter;

	Eigen::Matrix3f rot;

	rot.col(0) = initialTangent.normalized();
	rot.col(1) = initialBitangent.normalized();
	rot.col(2) = initialTangent.cross(initialBitangent).normalized();

	currentPlane.normal = rot.col(2);
	currentPlane.originDis = currentPlane.normal.dot(initialCenter);

	planeMatrix.topLeftCorner(3,3) = rot;

	createCollisionMap(cameraCoordinates);

	planeTracking = true;
	planeInlierBufferValid = false;
}

void PlaneEstimator::ransacConsensusSetID(const std::vector<Eigen::Vector3f> &pointcloud){
	unsigned int downsampleFactor = pointcloud.size()/10000;

	PlaneFittingTools::ransac(pointcloud, &consensusSet, ransacIterations, inlierThreshold*sceneScale, downsampleFactor);
}

void PlaneEstimator::ocRansacConsensusSetID(const std::vector<Eigen::Vector3f> &pointcloud){
	unsigned int downsampleFactor = pointcloud.size()/10000;

	delete octree;
	octree = new Octree( Eigen::Vector3f(0,0,0), PlaneFittingTools::findOutermostPoint(pointcloud), (float)(octreeLeafSidelengthFactor*inlierThreshold*sceneScale));

	for ( auto &i : pointcloud ) {
		octree->insertSidelenghtBased(i);
	}

	PlaneFittingTools::octreeRansac(pointcloud, *octree, &consensusSet, ransacIterations, inlierThreshold*sceneScale, downsampleFactor);

	Octree * winnerCell = octree->getLeafContainingPoint(consensusSet[0]);
	octreeCell = winnerCell->data;

	octreeBufferValid = false;
}


/*** update the plane with the information of the latest frame ***/
void PlaneEstimator::refreshPlane(){
	//If there are no keyframes yet, return
	if(viewer->getGraphDisplay()->getKeyframes().empty()) return;

	//If the number of keyframes didn't change since the last call, return
	if(lastUpdateFrame == viewer->getGraphDisplay()->getKeyframes().size()) return;
	lastUpdateFrame = viewer->getGraphDisplay()->getKeyframes().size();

	std::cout<<"New keyframe pointcloud added. Refreshing plane."<<std::endl;

	KeyFrameDisplay * mostRecentFrame = viewer->getGraphDisplay()->getKeyframes().back();
	std::vector<Eigen::Vector3f> * keyframePointcloud = mostRecentFrame->getKeyframePointcloud();

	//Calculate Hessian Representation of plane
	HessianNormalForm plane;
	plane.normal = planeMatrix.col(2).topRows(3).normalized();
	plane.originDis = plane.normal.dot(planeMatrix.col(3).topRows(3));

	//Identify inliers in new pointcloud
	std::vector<Eigen::Vector3f> keyframeInliers;
	PlaneFittingTools::findInliers((*keyframePointcloud), &keyframeInliers, inlierThreshold, plane);
	if(keyframeInliers.empty()) return;

	//Add the new inliers to the inlier vector so that they are drawn, too
	consensusSet.insert(consensusSet.end(), keyframeInliers.begin(), keyframeInliers.end());

	//Calculate covariance matrix of new inliers
	int keyframeInlierNo = keyframeInliers.size();

	//Map the memory layout of the std::vector to the memory layout of a Eigen::Matrix3f
	MatrixXfrm mat (keyframeInlierNo, 3);
	mat = Eigen::Map<MatrixXfrm> ((float*)consensusSet.data(), keyframeInlierNo, 3);

	//Calculate the new keyframe's covariance matrix
	Eigen::Vector3f keyframePointcloudMean = mat.colwise().mean().transpose();
	Eigen::Matrix3f centered = mat.rowwise() - keyframePointcloudMean.transpose();
	Eigen::Matrix3f keyframeCovMatrix = (centered.adjoint() * centered) / (float)mat.rows();

	//Add "old" covariance Matrix and new one
	covarianceMatrix = (covarianceMatrix + keyframeCovMatrix);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(covarianceMatrix);

	//The plane normal is the cross product of the two largest principal components,
	//i.e. the eigenvectors with the largest respective eigenvalues. These are contained in columns 2 and 3 of the
	//SelfAdjointEigenSolver eigenvectors matrix (see Eigen Documentation)
	Eigen::Vector3f tangent, bitangent, normal;
	tangent = eig.eigenvectors().col(2);
	bitangent = eig.eigenvectors().col(1);
	normal = tangent.cross(bitangent).normalized();

	//In order to make the plane keep its original orientation, the initial plane x and y axis are projected
	//onto the new plane.
	Eigen::Vector3f x, y, z;
	x = initialTangent - (initialTangent.dot(normal)) * normal;
	y = initialBitangent - (initialBitangent.dot(normal)) * normal;

	x.normalize();
	y.normalize();
	z = x.cross(y).normalized();

	if(z.dot(planeMatrix.col(2).topRows(3))<0 ) z = z*-1;

	//x, y and z make a orthonormal coordinate system that can be used as a transformation matrix for the
	//plane vertices.
	Eigen::Matrix3f rot;
	rot.col(0) = x;
	rot.col(1) = y;
	rot.col(2) = z;

	currentPlane.normal = rot.col(2);
	currentPlane.originDis = currentPlane.normal.dot(initialCenter);

	planeMatrix.topLeftCorner(3,3) = rot;

	planeInlierBufferValid = false;
}

// Get/Set functions
Eigen::Matrix4f PlaneEstimator::getPlaneMatrix(){
	return planeMatrix;
}

HessianNormalForm PlaneEstimator::getPlane(){
	return currentPlane;
}

void PlaneEstimator::initCollisionMap(){
	collisionMap = std::vector<std::vector<int> > (collisionMapSize);

	for(unsigned int i = 0; i<collisionMapSize; i++){
		collisionMap[i] = std::vector<int> (collisionMapSize, 0);
	}
}

void PlaneEstimator::createCollisionMap(const Eigen::Vector3f & cameraCoordinates){
	std::vector<KeyFrameDisplay *> keyframes = viewer->getGraphDisplay()->getKeyframes();

	initCollisionMap();
	collisionInliers.clear();

	float normalSign;

	if(cameraCoordinates.dot(currentPlane.normal)>0) normalSign = 1;
	else normalSign = -1;

	for(unsigned int i =0; i<keyframes.size(); i++)
	{
		std::vector<Eigen::Vector3f> * pointcloud =keyframes[i]->getKeyframePointcloud();

		for(unsigned int j = 0; j<pointcloud->size(); j++){
			Eigen::Vector3f currentPoint = (*pointcloud)[j];

			float distance = PlaneFittingTools::calcSignedPlanePointDis(currentPlane, currentPoint) * normalSign;

			if( (distance > 0) && (distance< 7*inlierThreshold*sceneScale) ){
				Eigen::Vector3f projectedPoint = PlaneFittingTools::projectPoint(currentPoint, currentPlane);
				Eigen::Vector4f homogenPoint = Eigen::Vector4f::Ones();
				homogenPoint.topRows(3) = projectedPoint;
				Eigen::Vector4f projectedPoint_planeCos = planeMatrix.inverse() * homogenPoint;

				if(std::abs( (float) projectedPoint_planeCos[0] )<2.5 && std::abs( (float) projectedPoint_planeCos[1] ) <2.5){
					collisionInliers.push_back(currentPoint);

					unsigned int xIndex = ((float)collisionMapSize/5.0f)*(float)(projectedPoint_planeCos(0) + 2.5);
					unsigned int yIndex = ((float)collisionMapSize/5.0f)*(float)(projectedPoint_planeCos(1) + 2.5);
					collisionMap[xIndex][yIndex]+=1;
				}
			}
		}
	}


  for (unsigned int i = 0; i<collisionMapSize; i++) {
	  for (unsigned int j = 0; j<collisionMapSize; j++)  {
		  	  if(collisionMap[i][j]>30) collisionMap[i][j] = 1;
		  	  else collisionMap[i][j] = 0;
	  }

	  std::cout<<std::endl;
  }

  collisionBufferValid = false;
}

bool PlaneEstimator::checkCollision(const Eigen::Vector4f &position){

	if(std::abs( (float) position[0] )<2.5 && std::abs( (float) position[1] ) <2.5){
		unsigned int xIndex = ((float)collisionMapSize/5.0f)*(float)(position(0) + 2.5);
		unsigned int yIndex = ((float)collisionMapSize/5.0f)*(float)(position(1) + 2.5);

		if(collisionMap[xIndex][yIndex] == 1) return true;
	}

	return false;
}

void PlaneEstimator::calcSceneScale(){
	std::vector< KeyFrameDisplay *> keyframes = viewer->getGraphDisplay()->getKeyframes();

	if(keyframes.size()<2) sceneScale = 1;

	float maxDistance = 0;

	for(unsigned int i =1; i<keyframes.size(); i+=2)
	{
		float distance = (keyframes[i]->camToWorld.translation() - keyframes[i-1]->camToWorld.translation()).norm();

		if(distance>maxDistance)
			maxDistance = distance;
	}

	sceneScale = maxDistance;
}

float PlaneEstimator::getSceneScale(){
	calcSceneScale();

	std::cout<<"Scene Scale: "<<sceneScale<<std::endl;

	return sceneScale;
}


/*
 * void PlaneEstimator::toggleConsensusSetDraw(){
	drawConsensusSet = !drawConsensusSet;
}

void PlaneEstimator::toggleCollisionDraw(){
	drawCollisionMap = !drawCollisionMap;
}

void PlaneEstimator::togglePlaneDraw(){
	drawPlane = !drawPlane;
}

void PlaneEstimator::togglePlaneUpdate(){
	planeUpdating = !planeUpdating;
}

 *
 */


