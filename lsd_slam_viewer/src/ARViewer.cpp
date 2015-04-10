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
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#define GL_GLEXT_PROTOTYPES 1

#include "ARViewer.h"

#include <Eigen/Geometry>

#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <unordered_set>

#include <boost/thread.hpp>
#include <queue>

#include "settings.h"

const bool useImageDisplayThread = true;

boost::mutex openCVdisplayMutex;
boost::condition_variable  openCVdisplaySignal;

boost::mutex keyPressMutex;
boost::condition_variable keyPressCondition;
int lastKey=0;

unsigned int currentFrameID = 0;

unsigned int idOffset = 0;
bool firstPop = true;
PointCloudViewer * _viewer;
ARViewer* window = 0;

std::clock_t currentFrameTime = 0;

ARViewer::ARViewer(std::string name, QWidget *parent, PointCloudViewer * pcViewer) :
    QGLWidget(parent, pcViewer)
{
    setWindowTitle(tr(name.c_str()));

    image = 0;
    width_img = height_img = 0;
    ego = false;
    arDemo = false;

    show();

    resize(640, 480);
    glViewport(0,0,640,480);

    this->name = name;
    this->pcViewer = pcViewer;
    this->planeEstimator = new PlaneEstimator(this);
    this->arObject = new ARObject(planeEstimator);
    this->benchmarking = new Benchmarking(this);

	pcViewer->setPlaneEstimator(this->planeEstimator);
	pcViewer->setARObject(arObject);

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(5);
}

ARViewer::~ARViewer()
{
    if(image != 0) delete[] image;
    delete benchmarking;
    delete arObject;
    delete planeEstimator;
}

void ARViewer::initializeGL()
{

   GLfloat mat_specular[] = { 0.0, 0.0, 0.0, 1.0 };
   GLfloat mat_ambient[] = {0,0,0,1};
   GLfloat mat_diffuse[] = {1,1,1,1};
   GLfloat light_position[] = { 6.0, 10.0, 15.0, 1.0 };

	glEnable(GL_LIGHTING);                 	//enables lighting
	glEnable(GL_LIGHT0);                   	//enables a light
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,0);  //sets lighting to one-sided

	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT,GL_DIFFUSE);

	float black[4]={0,0,0,0};
	glMaterialfv(GL_FRONT,GL_AMBIENT,black);
	glMaterialfv(GL_FRONT,GL_SPECULAR,black);


	glLightfv(GL_LIGHT0,GL_POSITION,light_position);   	//updates the light's position
	glLightfv(GL_LIGHT0,GL_DIFFUSE,mat_diffuse);    //updates the light's diffuse colour
	glLightfv(GL_LIGHT0,GL_SPECULAR,mat_specular);  //updates the light's specular colour
	glLightfv(GL_LIGHT0,GL_AMBIENT,mat_ambient);    //updates the light's ambient colour

}


void ARViewer::paintGL()
{

	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	if(image != 0 && !ego)
	{
		glMatrixMode(GL_MODELVIEW);

		glPushMatrix();
			glLoadIdentity();

			glMatrixMode(GL_PROJECTION);

			glPushMatrix();
				glLoadIdentity();

				glRasterPos2f( -1,1);
				glPixelZoom( width() / (float)width_img, -height()/(float)height_img );
				glDrawPixels( width_img, height_img, GL_BGR, GL_UNSIGNED_BYTE, image);

			glPopMatrix();
		glPopMatrix();
	}

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();

		if(ego) {
			Eigen::Matrix4f planeTransformation = planeEstimator->getPlaneMatrix();
			Eigen::Matrix4f objectTransformation = *(arObject->getPose());

			planeTransformation.col(3) -= 0.05* planeTransformation.col(2);

			Eigen::Matrix4f transMatrix = Eigen::Matrix4f::Zero();
			transMatrix(0,2) = 1;
			transMatrix(1,0) = -1;
			transMatrix(2,1) = 1;
			transMatrix(3,3) = 1;

			Eigen::Matrix4f totalTransformation = planeTransformation*objectTransformation*transMatrix;
			totalTransformation = totalTransformation.inverse();

			glLoadMatrixf(totalTransformation.data());
		} else glLoadMatrixf(pcViewer->getModelViewMatrix().data());

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();

			glLoadMatrixf(pcViewer->getProjectionMatrix().data());

			glMatrixMode(GL_MODELVIEW);

			glEnable(GL_DEPTH_TEST);

			if(arDemo) {
//				planeEstimator->draw();

				glEnable(GL_LIGHTING);
				if(drawARObject && !ego) arObject->draw();
				glDisable(GL_LIGHTING);
			}

			if(drawARPointcloud) pcViewer->getGraphDisplay()->draw();

			glDisable(GL_DEPTH_TEST);

		glPopMatrix();
	glPopMatrix();

	glFlush();

}


void ARViewer::loadImage(cv::Mat m, bool resize)
{
    if(image == 0)
    	image = new unsigned char[3*m.cols*m.rows];
    else if(width_img*height_img != m.cols*m.rows)
    {
    	delete[] image;
    	image = new unsigned char[3*m.cols*m.rows];
    }

    width_img=m.cols;
    height_img=m.rows;

    if(resize && (width() != width_img || height() != height_img))
    {
    	glViewport(0,0,width_img,height_img);
    	this->resize(width_img, height_img);
    }

    if(m.type() == CV_8UC3)
    {
    	memcpy(image, m.data, 3*width_img*height_img*sizeof(unsigned char));
    }
    else  if(m.type() == CV_32FC3)
    {
    	float* dataM = (float*) m.data;
    	for(int i=0;i<width_img*height_img;i++)
    	{
    		image[3*i] = dataM[3*i];
    		image[3*i+1] = dataM[3*i+1];
    		image[3*i+2] = dataM[3*i+2];
    	}
    }
    else  if(m.type() == CV_32F)
    {
    	float* dataM = (float*) m.data;
    	for(int i=0;i<width_img*height_img;i++)
    	{
    		image[3*i+2] = image[3*i+1] = image[3*i] = dataM[i];
    	}
    }
    else
    {
    	printf("ERROR: unknown image encoding sent to GLImageWindow::loadImage(cv::Mat m)!\n");
    }

//    updateGL();

    //printf("load event for image \"%s\"!\n", name.c_str());
    return;
}

void ARViewer::reset(){
	delete planeEstimator;
	delete arObject;
	delete benchmarking;

    this->planeEstimator = new PlaneEstimator(this);
    this->arObject = new ARObject(planeEstimator);
    this->benchmarking = new Benchmarking(this);

	pcViewer->setPlaneEstimator(this->planeEstimator);
	pcViewer->setARObject(arObject);

    arDemo = false;
}

void ARViewer::keyPressEvent(QKeyEvent *ke)
{
	switch(ke->key()){
		case Qt::Key_R: {
			Eigen::Vector3f cameraCoordinates = pcViewer->getCurrentCamDisplay()->camToWorld.matrix().col(3).topRows(3);
			planeEstimator->beginPlaneTracking(cameraCoordinates);
			arObject->init(cameraCoordinates);
			arDemo = true;
		} break;
		case Qt::Key_Up:
			arObject->accelerate(-1);
			break;
		case Qt::Key_Down:
			arObject->accelerate(1);
			break;
		case Qt::Key_Left:
			arObject->rotate(1);
			break;
		case Qt::Key_Right:
			arObject->rotate(-1);
			break;
		case Qt::Key_F:
			break;
		case Qt::Key_E:
			if(arDemo) ego = !ego;
			break;
		case Qt::Key_Q:
			break;
		case Qt::Key_O:
			break;
		case Qt::Key_U:
			break;
		case Qt::Key_I:
			break;
		case Qt::Key_P:
			break;
		case Qt::Key_C:
			break;
		case Qt::Key_T:
			break;
		case Qt::Key_0:
			benchmarking->ransacIterationSensitivity();
			break;
		case Qt::Key_1:
			benchmarking->ransacToleranceSensitivity();
			break;
		case Qt::Key_2:
			benchmarking->ocRansacLeafSizeSensitivity();
			break;
		case Qt::Key_3:
			benchmarking->ocRansacIterationSensitivity();
			break;
		case Qt::Key_4:
			benchmarking->ocRansacLeafSizeSensitivitySidelengthBased();
			break;
		case Qt::Key_5:
			benchmarking->ocRansacIterationSensitivitySidelengthBased();
			break;
		case Qt::Key_6:
			benchmarking->ocRansacToleranceSensitivitySidelengthBased();
			break;
	}

	boost::unique_lock<boost::mutex> lock(keyPressMutex);
	lastKey = ke->key();
	keyPressCondition.notify_one();
}

void ARViewer::keyReleaseEvent(QKeyEvent *ke){

	switch(ke->key()){
		case Qt::Key_Up:
			if(ke->isAutoRepeat()) return;
			arObject->accelerate(0);
			break;
		case Qt::Key_Down:
			if(ke->isAutoRepeat()) return;
			arObject->accelerate(0);
			break;
		case Qt::Key_Left:
			break;
		case Qt::Key_Right:
			break;
		case Qt::Key_F:
			break;
	}
}

void ARViewer::enqueueTimestampedMat(TimestampedMat msg){

	if((std::clock()-currentFrameTime)/(double)CLOCKS_PER_SEC>0.02){
//		loadImage(msg.image, true);
		if(imageQueue.size()>60){
			imageQueue.pop();
		}

		imageQueue.push(msg);
		currentFrameTime = std::clock();
	}
}

void ARViewer::popImage(double timestamp){
	while(!imageQueue.empty() && (imageQueue.front().timestamp<=timestamp)){
		loadImage(imageQueue.front().image, true);
		imageQueue.pop();
	}
}

void ARViewer::checkReset(unsigned int poseId){
	if(currentFrameID>poseId){
		printf("AR Demo: detected backward-jump in id (%d to %d), resetting!\n", currentFrameID, poseId);

		while(!imageQueue.empty()) imageQueue.pop();
		reset();
	}

	currentFrameID = poseId;
}

