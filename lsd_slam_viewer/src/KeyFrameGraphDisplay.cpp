/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with dvo. If not, see <http://www.gnu.org/licenses/>.
*/
#define GL_GLEXT_PROTOTYPES 1

#include "QGLImageWindow.h"
#include "PlaneFitting.h"
#include "KeyFrameGraphDisplay.h"
#include "KeyFrameDisplay.h"
#include "settings.h"

#include <sstream>
#include <fstream>

#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "ros/package.h"

KeyFrameGraphDisplay::KeyFrameGraphDisplay()
{
	flushPointcloud = false;
	printNumbers = false;
	planeTracking = false;

	planeBufferIdValid = false;

	totalInlierNumber = 0;
	keyframeUpdateTracker = 0;

	height = 0;
	width = 0;
}

KeyFrameGraphDisplay::~KeyFrameGraphDisplay()
{
	for(unsigned int i=0;i<keyframes.size();i++)
		delete keyframes[i];
}


void KeyFrameGraphDisplay::draw()
{
	dataMutex.lock();
	numRefreshedAlready = 0;

	// draw keyframes
	float color[3] = {0,0,1};
	for(unsigned int i=0;i<keyframes.size();i++)
	{
		if(showKFCameras)
			keyframes[i]->drawCam(lineTesselation, color);

		if((showKFPointclouds && (int)i > cutFirstNKf) || i == keyframes.size()-1)
			keyframes[i]->drawPC(pointTesselation, 1);
	}

	if(flushPointcloud)
	{

		printf("Flushing Pointcloud to %s!\n", (ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
		std::ofstream f((ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
		int numpts = 0;
		for(unsigned int i=0;i<keyframes.size();i++)
		{
			if((int)i > cutFirstNKf)
				numpts += keyframes[i]->flushPC(&f);
		}
		f.flush();
		f.close();

		std::ofstream f2((ros::package::getPath("lsd_slam_viewer")+"/pc.ply").c_str());
		f2 << std::string("ply\n");
		f2 << std::string("format binary_little_endian 1.0\n");
		f2 << std::string("element vertex ") << numpts << std::string("\n");
		f2 << std::string("property float x\n");
		f2 << std::string("property float y\n");
		f2 << std::string("property float z\n");
		f2 << std::string("property float intensity\n");
		f2 << std::string("end_header\n");

		std::ifstream f3((ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
		while(!f3.eof()) f2.put(f3.get());

		f2.close();
		f3.close();

		system(("rm "+ros::package::getPath("lsd_slam_viewer")+"/pc_tmp.ply").c_str());
		flushPointcloud = false;
		printf("Done Flushing Pointcloud with %d points!\n", numpts);

	}


	if(printNumbers)
	{
		int totalPoint = 0;
		int visPoints = 0;
		for(unsigned int i=0;i<keyframes.size();i++)
		{
			totalPoint += keyframes[i]->totalPoints;
			visPoints += keyframes[i]->displayedPoints;
		}

		printf("Have %d points, %d keyframes, %d constraints. Displaying %d points.\n",
				totalPoint, (int)keyframes.size(), (int)constraints.size(), visPoints);
		printNumbers = false;
	}




	if(showConstraints)
	{
		// draw constraints
		glLineWidth(lineTesselation);
		glBegin(GL_LINES);
		for(unsigned int i=0;i<constraints.size();i++)
		{
			if(constraints[i].from == 0 || constraints[i].to == 0)
				continue;

			double colorScalar = std::max(0.0, std::min(1.0, constraints[i].err / 0.05));
			glColor3f(colorScalar, 1 - colorScalar, 0);


			Sophus::Vector3f t = constraints[i].from->camToWorld.translation();
			glVertex3f((GLfloat) t[0],(GLfloat) t[1], (GLfloat) t[2]);

			t = constraints[i].to->camToWorld.translation();
			glVertex3f((GLfloat) t[0],(GLfloat) t[1], (GLfloat) t[2]);

		}
		glEnd();
	}

	/**** For plane estimation *****/

	if(planeTracking){
		refreshPlane();

		/*cv::Mat indicators = cv::Mat(height,width,CV_8UC3, cv::Scalar::all(0));

		displayImage("Estimated plane", indicators, false);
		waitKey(0);*/

		glDisable(GL_LIGHTING);

		glPushMatrix();

				glLineWidth(1.0);

				glBindBuffer(GL_ARRAY_BUFFER, planeBufferId);

				glVertexPointer(3, GL_FLOAT, sizeof(MyVertex), 0);
				glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(MyVertex), (const void*) (3*sizeof(float)));

				glEnableClientState(GL_VERTEX_ARRAY);
				glEnableClientState(GL_COLOR_ARRAY);

				glDrawArrays(GL_LINE_LOOP, 0, planeBufferNumPoints);

				glDisableClientState(GL_COLOR_ARRAY);
				glDisableClientState(GL_VERTEX_ARRAY);

			glPopMatrix();
	}

	dataMutex.unlock();
}

void KeyFrameGraphDisplay::addMsg(lsd_slam_viewer::keyframeMsgConstPtr msg)
{
	dataMutex.lock();
	if(keyframesByID.count(msg->id) == 0)
	{
		KeyFrameDisplay* disp = new KeyFrameDisplay();
		keyframesByID[msg->id] = disp;
		keyframes.push_back(disp);

	//	printf("added new KF, now there are %d!\n", (int)keyframes.size());
	}

	width = msg->width;
	height = msg->height;

	if(keyframes.size()==beginPlaneTrackingIndex) beginPlaneTracking();

	keyframesByID[msg->id]->setFrom(msg);
	dataMutex.unlock();
}

void KeyFrameGraphDisplay::addGraphMsg(lsd_slam_viewer::keyframeGraphMsgConstPtr msg)
{
	dataMutex.lock();

	constraints.resize(msg->numConstraints);
	assert(msg->constraintsData.size() == sizeof(GraphConstraint)*msg->numConstraints);
	GraphConstraint* constraintsIn = (GraphConstraint*)msg->constraintsData.data();
	for(int i=0;i<msg->numConstraints;i++)
	{
		constraints[i].err = constraintsIn[i].err;
		constraints[i].from = 0;
		constraints[i].to = 0;

		if(keyframesByID.count(constraintsIn[i].from) != 0)
			constraints[i].from = keyframesByID[constraintsIn[i].from];
//		else
//			printf("ERROR: graph update contains constraints for %d -> %d, but I dont have a frame %d!\n",
//					constraintsIn[i].from,
//					constraintsIn[i].to,
//					constraintsIn[i].from);


		if(keyframesByID.count(constraintsIn[i].to) != 0)
			constraints[i].to = keyframesByID[constraintsIn[i].to];
//		else
//			printf("ERROR: graph update contains constraints for %d -> %d, but I dont have a frame %d!\n",
//					constraintsIn[i].from,
//					constraintsIn[i].to,
//					constraintsIn[i].to);
	}



	GraphFramePose* graphPoses = (GraphFramePose*)msg->frameData.data();
	int numGraphPoses = msg->numFrames;
	assert(msg->frameData.size() == sizeof(GraphFramePose)*msg->numFrames);

	for(int i=0;i<numGraphPoses;i++)
	{
		if(keyframesByID.count(graphPoses[i].id) == 0)
		{
		//	printf("ERROR: graph update contains pose for frame %d, but I dont have a frame %d!\n", graphPoses[i].id, graphPoses[i].id);
		}
		else
			memcpy(keyframesByID[graphPoses[i].id]->camToWorld.data(), graphPoses[i].camToWorld, 7*sizeof(float));
	}

	dataMutex.unlock();

//	printf("graph update: %d constraints, %d poses\n", msg->numConstraints, msg->numFrames);
}

void KeyFrameGraphDisplay::beginPlaneTracking(){
	if(debugMode)
			std::cerr<<__PRETTY_FUNCTION__<<std::endl;

	std::vector<KeyFrameDisplay*> keyframes2;

	for(std::map<int, KeyFrameDisplay*>::iterator it = keyframesByID.begin(); it!=keyframesByID.end(); it++){
		keyframes2.push_back(it->second);
	}


	std::vector<Eigen::Vector3f> inliers = PlaneFitting::ransac(keyframes2, 25, 0.01);

	Eigen::MatrixXf mat((int)inliers.size(), 3);

	for(unsigned int i = 0; i<inliers.size(); i++){
		mat(i, 0) = inliers[i][0];

		mat(i, 1) = inliers[i][1];
		mat(i, 2) = inliers[i][2];
	}

	totalInlierNumber = inliers.size();

	covarianceMatrix = PlaneFitting::pcaPlaneFitting(mat, tangent, bitangent, center);

	planeTracking = true;
}

void KeyFrameGraphDisplay::refreshPlane(){
	if(debugMode)
			std::cerr<<__PRETTY_FUNCTION__<<std::endl;

	//Check whether a new keyframe Pointcloud is availble.
	if(keyframeUpdateTracker == keyframes.size()-1) return;

	keyframeUpdateTracker = keyframes.size()-1;

	Eigen::Matrix3f keyframeCovMatrix;
	Eigen::Vector3f keyframeCenter;

	int keyframePointNum = keyframes[keyframes.size()-1]->keyframePointcloud.size();

	keyframes[keyframes.size()-1]->calcKeyframeCovMatrix(keyframeCovMatrix, center, center, tangent, bitangent);

	covarianceMatrix = ( covarianceMatrix*totalInlierNumber + keyframeCovMatrix*keyframePointNum) / float(keyframePointNum+totalInlierNumber);

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(covarianceMatrix);

	tangent =  eig.eigenvectors().col(2);
	bitangent =  eig.eigenvectors().col(1);

	totalInlierNumber += keyframePointNum;

	Eigen::Vector3f triangleWidth = tangent/10;
	Eigen::Vector3f triangleHeight = bitangent/10;
	Eigen::Vector3f virtualCenter = center-2.5*tangent-2.5*bitangent;

	int verticeCount = 0;

	MyVertex triangleVertices[5000];

	while(verticeCount<4998){
		virtualCenter +=triangleHeight;

		for(int i = 0; i<50; i++)
		{
			for(int j = 1; j>=0; j--)
			{
				triangleVertices[verticeCount].point[0] = virtualCenter[0] + triangleWidth[0]*i+j*triangleHeight[0];
				triangleVertices[verticeCount].point[1] = virtualCenter[1] + triangleWidth[1]*i+j*triangleHeight[1];
				triangleVertices[verticeCount].point[2] = virtualCenter[2] + triangleWidth[2]*i+j*triangleHeight[2];

	//			if(debugMode)
	//				printf("x: %f, y: %f, z: %f\n", triangleVertices[verticeCount].point[0], triangleVertices[verticeCount].point[1], triangleVertices[verticeCount].point[2]);

				triangleVertices[verticeCount].color[0] = 100;
				triangleVertices[verticeCount].color[1] = planeColor[0];
				triangleVertices[verticeCount].color[2] = planeColor[1];
				triangleVertices[verticeCount].color[3] = planeColor[2];

				verticeCount++;
			}
		}
	}

	planeBufferNumPoints = verticeCount;

	planeBufferId = 0;
	glGenBuffers(1, &planeBufferId);
	glBindBuffer(GL_ARRAY_BUFFER, planeBufferId);         // for vertex coordinates
	glBufferData(GL_ARRAY_BUFFER, sizeof(MyVertex) * planeBufferNumPoints, triangleVertices, GL_STATIC_DRAW);
}
