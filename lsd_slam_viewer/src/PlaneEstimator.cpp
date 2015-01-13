/*
 * PlaneEstimator.cpp
 *
 *  Created on: Dec 19, 2014
 *      Author: vincent
 */

#define GL_GLEXT_PROTOTYPES 1

#include "PlaneEstimator.h"

#include "PlaneFitting.h"
#include "settings.h"

#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

typedef Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic, Eigen::RowMajor> MatrixXfrm;

PlaneEstimator::PlaneEstimator(KeyFrameGraphDisplay * graphDisplay) {
	this->graphDisplay = graphDisplay;

	lastUpdateFrame = 0;
}

PlaneEstimator::~PlaneEstimator() {
	// TODO Auto-generated destructor stub
}

void PlaneEstimator::draw() {

	if(planeTracking){
		refreshPlane();

		car->draw();

		glDisable(GL_LIGHTING);

		glLineWidth(1.0);

		glBindBuffer(GL_ARRAY_BUFFER, planeBufferId);

		glVertexPointer(3, GL_FLOAT, sizeof(MyVertex), 0);
		glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(MyVertex), (const void*) (3*sizeof(float)));

		glEnableClientState(GL_VERTEX_ARRAY);
		glEnableClientState(GL_COLOR_ARRAY);

		glDrawArrays(GL_LINE_LOOP, 0, planeBufferNumPoints);

		glDisableClientState(GL_COLOR_ARRAY);
		glDisableClientState(GL_VERTEX_ARRAY);

	}
}

void PlaneEstimator::beginPlaneTracking(){
	if(debugMode)
		std::cerr<<__PRETTY_FUNCTION__<<std::endl;

	std::vector<Eigen::Vector3f> inliers = PlaneFitting::ransac(graphDisplay->keyframes, 25, inlierTolerance);

	MatrixXfrm mat ((int)inliers.size(), 3);
	mat = Eigen::Map<MatrixXfrm> ((float*)inliers.data(), (int)inliers.size(), 3);

	totalInlierNumber = inliers.size();

	covarianceMatrix = PlaneFitting::pcaPlaneFitting(mat, tangent, bitangent, center);

	initARDemo();

	planeTracking = true;
}

void PlaneEstimator::refreshPlane(){
	if(lastUpdateFrame == graphDisplay->keyframes.size()) return;
	lastUpdateFrame = graphDisplay->keyframes.size();

	Eigen::Matrix3f keyframeCovMatrix;
	Eigen::Vector3f keyframeCenter;
	Eigen::Vector3f normal;
	std::vector<Eigen::Vector3f> inliers;
 	Eigen::Vector4f planeParams;
	Eigen::Vector3f P1, P2, P3;
	Eigen::Vector3f keyframeTangent;
	Eigen::Vector3f keyframeBitangent;
	int keyframeInlierNo = 0;
	KeyFrameDisplay * mostRecentFrame = graphDisplay->keyframes.back();

	/*** update the plane estimation with the information of the latest frame ***/
	std::vector<Eigen::Vector3f> * keyframePointcloud = &(mostRecentFrame->keyframePointcloud);

	P1 = center;
	P2 = center + tangent;
	P3 = center + bitangent;

	planeParams = PlaneFitting::calcPlaneParams(P1, P2, P3);

	double dis = 0;

	for(unsigned int i = 0; i<keyframePointcloud->size(); i++){
		dis = PlaneFitting::calcPlanePointDis(planeParams, (*keyframePointcloud)[i]);

		if(dis < inlierTolerance){
			inliers.push_back((*keyframePointcloud)[i]);
		}
	}

	if(inliers.empty()) return;

	keyframeInlierNo = inliers.size();

	MatrixXfrm mat (keyframeInlierNo, 3);
	mat = Eigen::Map<MatrixXfrm> ((float*)inliers.data(), keyframeInlierNo, 3);

	keyframeCenter = mat.colwise().mean().transpose();

	Eigen::MatrixXf centered = mat.rowwise() - 	keyframeCenter.transpose();
	keyframeCovMatrix = (centered.adjoint() * centered) / float(mat.rows());

	covarianceMatrix = (covarianceMatrix*totalInlierNumber + keyframeCovMatrix*keyframeInlierNo) / float(keyframeInlierNo+totalInlierNumber);
	totalInlierNumber += keyframeInlierNo;

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(covarianceMatrix);

	keyframeTangent =  eig.eigenvectors().col(2);
	keyframeBitangent =  eig.eigenvectors().col(1);
	normal = keyframeTangent.cross(keyframeBitangent).normalized();

	keyframeTangent = tangent - (tangent.dot(normal)) * normal;
	keyframeBitangent = bitangent - (bitangent.dot(normal)) * normal;

//	center = keyframeCenter;

//	Eigen::Vector3f normal = tangent.cross(bitangent).normalized();
//
//	center = mostRecentFrame->camToWorld.translation() - (mostRecentFrame->camToWorld.translation().dot(normal)-inliers[0].dot(normal)) * normal;

	Eigen::Vector3f triangleWidth = keyframeTangent.normalized()/10;
	Eigen::Vector3f triangleHeight = keyframeBitangent.normalized()/10;
	Eigen::Vector3f virtualCenter = center-2.5*keyframeTangent.normalized()-2.5*keyframeBitangent.normalized();

	std::vector<MyVertex> planeVertices;

	while(planeVertices.size()<5000){
		virtualCenter +=triangleHeight;

		for(int i = 0; i<50; i++)
		{
			for(int j = 1; j>=0; j--)
			{
				MyVertex buffer;

				buffer.point[0] = virtualCenter[0] + triangleWidth[0]*i+j*triangleHeight[0];
				buffer.point[1] = virtualCenter[1] + triangleWidth[1]*i+j*triangleHeight[1];
				buffer.point[2] = virtualCenter[2] + triangleWidth[2]*i+j*triangleHeight[2];

				buffer.color[0] = 100;
				buffer.color[1] = planeColor[0];
				buffer.color[2] = planeColor[1];
				buffer.color[3] = planeColor[2];

				planeVertices.push_back(buffer);
			}
		}
	}

	planeBufferNumPoints = planeVertices.size();

	planeBufferId = 0;
	glGenBuffers(1, &planeBufferId);
	glBindBuffer(GL_ARRAY_BUFFER, planeBufferId);         // for vertex coordinates
	glBufferData(GL_ARRAY_BUFFER, sizeof(MyVertex) * planeBufferNumPoints, planeVertices.data(), GL_STATIC_DRAW);
}

void PlaneEstimator::initARDemo(){
	if(debugMode)
		std::cerr<<__PRETTY_FUNCTION__<<std::endl;

	Eigen::Matrix4f initialCarPose = Eigen::Matrix4f::Identity();
	initialCarPose.rightCols(1).topRows(3) = center;

	Eigen::Vector3f x, y, z;

	x = tangent.normalized();
	y = bitangent.normalized();
	z = x.cross(y).normalized();

	Eigen::Matrix3f rot;

	rot.col(1) = x;
	rot.col(2) = y;
	rot.col(3) = z;

	initialCarPose.topLeftCorner(3,3) = rot;

	Eigen::Vector4f upVector = Eigen::Vector4f::Zero();
	upVector.topRows(3) = z;

	this->car = new Car(initialCarPose, upVector,  tangent.norm()/10);
}

