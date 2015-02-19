/*
 * PlaneEstimator.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: vincent
 */

#define GL_GLEXT_PROTOTYPES 1

#include "PlaneEstimator.h"
#include "PointCloudViewer.h"

#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "settings.h"

typedef Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic, Eigen::RowMajor> MatrixXfrm;

PlaneEstimator::PlaneEstimator(PointCloudViewer * viewer) {
	// TODO Auto-generated constructor stub
	generatePlaneVBOs();

	this->viewer = viewer;
}

PlaneEstimator::~PlaneEstimator() {
	// TODO Auto-generated destructor stub
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

		planeBufferId = 0;
		glGenBuffers(1, &planeBufferId);
		glBindBuffer(GL_ARRAY_BUFFER, planeBufferId);         // for vertex coordinates
		glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f) * plane_vertices.size(), plane_vertices.data(), GL_STATIC_DRAW);

		inlierBufferId = 0;
		glGenBuffers(1, &inlierBufferId);
		glBindBuffer(GL_ARRAY_BUFFER, inlierBufferId);         // for vertex coordinates
		glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f) * inliers.size(), inliers.data(), GL_STATIC_DRAW);


		glMatrixMode(GL_MODELVIEW);

		glPushMatrix();
			glMultMatrixf(planeMatrix.data());

//			  glBindBuffer(GL_ARRAY_BUFFER, planeBufferId);
//			  glVertexAttribPointer(
//				0,  // attribute
//				3,                  // number of elements per vertex, here (x,y,z,w)
//				GL_FLOAT,           // the type of each element
//				GL_FALSE,           // take our values as-is
//				0,                  // no extra data between each position
//				0                   // offset of first element
//			  );
//
//			  glBindBuffer(GL_ARRAY_BUFFER, inlierBufferId);
//			  glVertexAttribPointer(
//				0,  // attribute
//				3,                  // number of elements per vertex, here (x,y,z,w)
//				GL_FLOAT,           // the type of each element
//				GL_FALSE,           // take our values as-is
//				0,                  // no extra data between each position
//				0                   // offset of first element
//			  );


			glEnableClientState(GL_VERTEX_ARRAY);

			  glBindBuffer(GL_ARRAY_BUFFER, planeBufferId);
				glVertexPointer(3, GL_FLOAT, sizeof(Eigen::Vector3f), 0);
				glColor3f(1, 1, 1);
				glDrawArrays(GL_LINE_LOOP, 0, plane_vertices.size());

			  glDisableClientState(GL_VERTEX_ARRAY);

		glPopMatrix();

		if(drawInlier){
			glEnableClientState(GL_VERTEX_ARRAY);

				glPointSize(1);
				glBindBuffer(GL_ARRAY_BUFFER, inlierBufferId);
				glVertexPointer(3, GL_FLOAT, sizeof(Eigen::Vector3f), 0);
				glColor3f(0, 0, 1);
				glDrawArrays(GL_POINTS, 0, inliers.size());

				glDisableClientState(GL_VERTEX_ARRAY);

				glColor3f(1, 1, 1);
		}


		glDeleteBuffers(1, &planeBufferId);
		glDeleteBuffers(1, &inlierBufferId);

	}
}

void PlaneEstimator::beginPlaneTracking(){
	if(viewer->getGraphDisplay()->getKeyframes().empty()) return;

	inliers = ransac(viewer->getGraphDisplay()->getKeyframes(), 300, ransacTolerance);

	MatrixXfrm mat ((int)inliers.size(), 3);
	mat = Eigen::Map<MatrixXfrm> ((float*)inliers.data(), (int)inliers.size(), 3);

	totalInlierNumber = inliers.size();

	covarianceMatrix = pcaPlaneFitting(mat, initialTangent, initialBitangent, initialCenter);


	planeMatrix = Eigen::Matrix4f::Identity();
	planeMatrix.col(3).topRows(3) = initialCenter;

	Eigen::Vector3f x, y, z;

	x = initialTangent.normalized();
	y = initialBitangent.normalized();
	z = -x.cross(y).normalized();

	Eigen::Matrix3f rot;

	rot.col(0) = x;
	rot.col(1) = y;
	rot.col(2) = z;

	planeMatrix.topLeftCorner(3,3) = rot;

	planeTracking = true;
}

std::vector<Eigen::Vector3f> PlaneEstimator::ransac(const std::vector<KeyFrameDisplay*> &keyframeDisplays, const int iterations, const float inlierDistance){
	int absolutePointNumber = 0;

	std::vector<Eigen::Vector3f> inliers;

	printf("Ransac will work on %lu keyframes.\n", keyframeDisplays.size());

	for(unsigned int i=0;i<keyframeDisplays.size();i++)
	{
		absolutePointNumber+=keyframeDisplays[i]->getKeyframePointcloud()->size();
	}

	printf("Ransac will run on a total number of %d points\n", absolutePointNumber);

	std::vector<Eigen::Vector3f> globalPointCloud;
	globalPointCloud.reserve(absolutePointNumber);

	for(unsigned int i =0; i<keyframeDisplays.size(); i++)
	{
		globalPointCloud.insert(globalPointCloud.end(), keyframeDisplays[i]->getKeyframePointcloud()->begin(), keyframeDisplays[i]->getKeyframePointcloud()->end());
	}

	int maxInliers = 0, inlier = 0;
	int pos1, pos2, pos3;
	Eigen::Vector3f P1, P2, P3, P;
	double dis;
	double bestPoints[3] = {0};
	Eigen::Vector3f planeNormal;
	float planeOriginDis;
	Eigen::Vector4f planeParams;

	for (int i=0; i < iterations; i++)
	{

		std::cerr<<__PRETTY_FUNCTION__<<" Ransac iteration "<< i << " out of " << iterations <<std::endl;

		// get Random Points
		pos1 = rand() % absolutePointNumber;
		pos2 = rand() % absolutePointNumber;
		pos3 = rand() % absolutePointNumber;

		// ensure all points are different
		while (pos1 == pos2 || pos2 == pos3 || pos1 == pos3) {
			pos1 = rand() % absolutePointNumber;
			pos2 = rand() % absolutePointNumber;
			pos3 = rand() % absolutePointNumber;
		}

		P1 = globalPointCloud[pos1];
		P2 = globalPointCloud[pos2];
		P3 = globalPointCloud[pos3];

		calcHessianParameters(P1, P2, P3, planeNormal, planeOriginDis);

		inlier = 0;
		int j = 0;

		while(j < absolutePointNumber){
			// calculate distance between point and plane
			P = globalPointCloud[j];
			dis = calcPlanePointDis(planeNormal, planeOriginDis, P);

			if (dis < inlierDistance){
				inlier++;
			}

			j+=10;
		}

		if (inlier > maxInliers) {
			maxInliers = inlier;
			bestPoints[0] = pos1;
			bestPoints[1] = pos2;
			bestPoints[2] = pos3;
		}
	}

	P1 = globalPointCloud[bestPoints[0]];
	P2 = globalPointCloud[bestPoints[1]];
	P3 = globalPointCloud[bestPoints[2]];

	calcHessianParameters(P1, P2, P3, planeNormal, planeOriginDis);

	for (int j = 0; j < absolutePointNumber; j++) {
		// calculate distance between point and plane
		P = globalPointCloud[j];
		double dis = calcPlanePointDis(planeNormal, planeOriginDis, P);

		if (dis < inlierDistance){
			inliers.push_back(globalPointCloud[j]);
		}
	}

	printf("%u inliers out of a total %u points\n", inliers.size());
	return inliers;
}

void PlaneEstimator::selfOrganizingMap(const std::vector<KeyFrameDisplay*> &keyframeDisplays){
	int absolutePointNumber = 0;

	std::vector<Eigen::Vector3f> inliers;

	for(unsigned int i=0;i<keyframeDisplays.size();i++)
	{
		absolutePointNumber+=keyframeDisplays[i]->getKeyframePointcloud()->size();
	}

	std::vector<Eigen::Vector3f> globalPointCloud;
	globalPointCloud.reserve(absolutePointNumber);

	for(unsigned int i =0; i<keyframeDisplays.size(); i++)
	{
		globalPointCloud.insert(globalPointCloud.end(), keyframeDisplays[i]->getKeyframePointcloud()->begin(), keyframeDisplays[i]->getKeyframePointcloud()->end());
	}

	std::vector< std::vector<Eigen::Vector3f> > kohonenNet;
	kohonenNet.reserve(2000);

	for(int i = 0; i<2000; i++){

	}

	int trainIndex = 0;
	int cooperationRadius = kohonenNet.size()/10;
	Eigen::Vector2f winnerIndices (0,0);
	float distance = 0;

	for(int i = 0; i<10000; i++){
		//Choose point from pointcloud at random.
		trainIndex = rand()%(absolutePointNumber);

		//Get the index of the closest neuron.
		winnerIndices = getClosestIndex(kohonenNet, globalPointCloud[trainIndex]);

		for(int i = 0; i<cooperationRadius; i++){
			for(int j = 0; j<cooperationRadius; j++){


			}
		}
	}


}

Eigen::Vector2f PlaneEstimator::getClosestIndex(const std::vector< std::vector <Eigen::Vector3f> > pointCloud, Eigen::Vector3f point){
	float minDistance = 10000;
	Eigen::Vector2f minCoordinates (0,0);
	float distance = 0;

	for(unsigned int i = 0; i<pointCloud.size(); i++){
		for(unsigned int j = 0; j<pointCloud[i].size(); j++){
			distance = (point-pointCloud[i][j]).norm();

			if(distance<minDistance){
				minCoordinates(0) = i;
				minCoordinates(1) = j;

				minDistance = distance;
			}
		}
	}

	return minCoordinates;
}

void PlaneEstimator::calcHessianParameters(Eigen::Vector3f P1, Eigen::Vector3f P2, Eigen::Vector3f P3, Eigen::Vector3f &normal, float &planeOriginDis){
	normal = (P2 - P1).cross(P3 - P1).normalized();

	if(normal.dot(P1)<0) normal *= -1;

	planeOriginDis = P1.dot(normal);
}

void PlaneEstimator::refreshPlane(){
	if(viewer->getGraphDisplay()->getKeyframes().empty()) return;
//	if(viewer->getGraphDisplay()->getKeyframes().size() == lastUpdateFrame) return;

	lastUpdateFrame = viewer->getGraphDisplay()->getKeyframes().size();

	Eigen::Matrix3f keyframeCovMatrix;
	Eigen::Vector3f keyframePointcloudMean;
	KeyFrameDisplay * mostRecentFrame = viewer->getGraphDisplay()->getKeyframes().back();

	/*** update the plane estimation with the information of the latest frame ***/
	std::vector<Eigen::Vector3f> * keyframePointcloud = mostRecentFrame->getKeyframePointcloud();

	//Calculate Hessian Representation of plane
	Eigen::Vector3f planeSupportPoint = planeMatrix.col(3).topRows(3);
	Eigen::Vector3f planeNormal = planeMatrix.col(2).topRows(3).normalized();
	float planeOriginDis = planeNormal.dot(planeSupportPoint);

	//Identify inliers in new pointcloud
	double dis = 0;
	std::vector<Eigen::Vector3f> inliers;

	for(unsigned int i = 0; i<keyframePointcloud->size(); i++){
		dis = calcPlanePointDis(planeNormal, planeOriginDis, (*keyframePointcloud)[i]);

		if(dis < ransacTolerance){
			inliers.push_back((*keyframePointcloud)[i]);
		}
	}

	if(inliers.empty()) return;

	//Calculate covariance matrix of new inliers
	int keyframeInlierNo = inliers.size();

	MatrixXfrm mat (keyframeInlierNo, 3);
	mat = Eigen::Map<MatrixXfrm> ((float*)inliers.data(), keyframeInlierNo, 3);

	keyframePointcloudMean = mat.colwise().mean().transpose();

	Eigen::MatrixXf centered = mat.rowwise() - 	keyframePointcloudMean.transpose();
	keyframeCovMatrix = (centered.adjoint() * centered) / float(mat.rows());

	//Weighted average of "old" covariance Matrix and new one
	covarianceMatrix = (covarianceMatrix*totalInlierNumber + keyframeCovMatrix*keyframeInlierNo) / float(keyframeInlierNo+totalInlierNumber);
	totalInlierNumber += keyframeInlierNo;

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(covarianceMatrix);
	Eigen::Vector3f x, y, z;

	//The plane normal is the cross product of the two principal components.
	Eigen::Vector3f tangent = eig.eigenvectors().col(2);
	Eigen::Vector3f bitangent = eig.eigenvectors().col(1);
	Eigen::Vector3f normal = tangent.cross(bitangent).normalized();

	//In order to make the plane keep its original orientation, the original plane coordinate system is
	//projected onto the new plane
	x = initialTangent - (initialTangent.dot(normal)) * normal;
	y = initialBitangent - (initialBitangent.dot(normal)) * normal;

	x.normalize();
	y.normalize();
	z = -x.cross(y).normalized();

	Eigen::Matrix3f rot;

	rot.col(0) = x;
	rot.col(1) = y;
	rot.col(2) = z;

	planeMatrix.topLeftCorner(3,3) = rot;
}

// Get/Set functions
Eigen::Matrix4f PlaneEstimator::getPlaneParameters(){
	return planeMatrix;
}

double PlaneEstimator::calcPlanePointDis(Eigen::Vector3f planeNormal, float planeOriginDis, Eigen::Vector3f P){
	return std::abs(planeNormal.dot(P) - planeOriginDis);
}

Eigen::Vector4f PlaneEstimator::calcPlaneParams(Eigen::Vector3f P1, Eigen::Vector3f P2, Eigen::Vector3f P3){
	Eigen::Vector4f buffer1;

	buffer1[0] = P1[1] * (P2[2] - P3[2]) + P2[1] * (P3[2] - P1[2]) + P3[1] * (P1[2] - P2[2]);
	buffer1[1] = P1[2] * (P2[0] - P3[0]) + P2[2] * (P3[0] - P1[0]) + P3[2] * (P1[0] - P2[0]);
	buffer1[2] = P1[0] * (P2[1] - P3[1]) + P2[0] * (P3[1] - P1[1]) + P3[0] * (P1[1] - P2[1]);
	buffer1[3] = -(P1[0] * (P2[1] * P3[2] - P3[1] * P2[2]) + P2[0] * (P3[1] * P1[2] - P1[1] * P3[2]) + P3[0] * (P1[1] * P2[2] - P2[1] * P1[2]));

	return buffer1;
}

void PlaneEstimator::createCollisionMap(){


}
