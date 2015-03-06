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

boost::thread* imageDisplayThread = 0;
std::queue<DisplayImageObect> displayQueue;
bool imageThreadKeepRunning = true;

std::queue<sensor_msgs::ImageConstPtr> msgQueue;

boost::mutex keyPressMutex;
boost::condition_variable keyPressCondition;
int lastKey=0;

unsigned int idOffset = 0;
bool firstPop = true;
PointCloudViewer * _viewer;

void displayThreadLoop(QApplication* app, PointCloudViewer * viewer)
{

	_viewer = viewer;
	ARViewer* window = new ARViewer("AR Viewer", 0, viewer);

	printf("started image display thread!\n");
	boost::unique_lock<boost::mutex> lock(openCVdisplayMutex);

	while(imageThreadKeepRunning)
	{
		lock.unlock();
		usleep(10000);
		app->processEvents();
		lock.lock();

		if(!imageThreadKeepRunning)
			break;

		while(displayQueue.size() > 0)
		{
			window->loadImage(displayQueue.front().img, displayQueue.front().autoSize);
			window->show();
			displayQueue.pop();
		}
	}

	delete [] window;

	printf("ended image display thread!\n");
}
void makeDisplayThread()
{
	imageThreadKeepRunning = true;
	//imageDisplayThread = new boost::thread(&displayThreadLoop);
}


void displayImage(const char* windowName, const cv::Mat& image, bool autoSize)
{
	if(useImageDisplayThread)
	{
		boost::unique_lock<boost::mutex> lock(openCVdisplayMutex);
		displayQueue.push(DisplayImageObect());
		displayQueue.back().autoSize = autoSize;
		displayQueue.back().img = image.clone();
		displayQueue.back().name = windowName;

		openCVdisplaySignal.notify_one();
	}
}

void enqueueImage(const sensor_msgs::ImageConstPtr& msg)
{

	msgQueue.push(msg);
}

void popImage(unsigned int imageId){

	if(msgQueue.empty()) return;

	if((msgQueue.front()->header.seq>imageId+idOffset) || firstPop){
		std::cout<<"Current ID offset: "<<idOffset<<std::endl;
		idOffset = msgQueue.front()->header.seq - imageId;

		std::cout<<"Detected reset. Flushing the frame buffer."<<std::endl;
		std::cout<<"Queue front: "<<msgQueue.front()->header.seq<<std::endl;
		std::cout<<"popped ID: "<<imageId<<std::endl;
		std::cout<<"New ID offset: "<<idOffset<<std::endl<<std::endl;

		firstPop = false;
	}

	while(msgQueue.front()->header.seq<=imageId+idOffset){
		cv::Mat image =  cv_bridge::toCvShare(msgQueue.front(), "rgb8")->image;

		displayImage("AR Demo", image, true);

		msgQueue.pop();

		if(msgQueue.empty()) break;
	}
}

int waitKey(int milliseconds)
{
	boost::unique_lock<boost::mutex> lock(keyPressMutex);

	if(milliseconds == 0)
	{
		keyPressCondition.wait(lock);
		return lastKey;
	}

	if(!keyPressCondition.timed_wait(lock, boost::posix_time::milliseconds(milliseconds)))
		return 0;
	else
		return lastKey;
}

void closeAllWindows()
{
	return;

}

ARViewer::ARViewer(std::string name, QWidget *parent, PointCloudViewer * viewer) :
    QGLWidget(parent, viewer)
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
    this->viewer = viewer;
    this->planeEstimator = new PlaneEstimator(viewer);
    this->arObject = new ARObject(planeEstimator);
    this->benchmarking = new Benchmarking(this);

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(10);
}

ARViewer::~ARViewer()
{
    if(image != 0) delete[] image;
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

    makeCurrent();
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

			Eigen::Matrix4f planeTransformation = planeEstimator->getPlaneParameters();
			Eigen::Matrix4f objectTransformation = *(arObject->getPose());

			Eigen::Matrix4f bufferMatrix = objectTransformation;
			objectTransformation.col(1) = objectTransformation.col(2);
			objectTransformation.col(0) = -bufferMatrix.col(1);
			objectTransformation.col(2) = bufferMatrix.col(0);

			Eigen::Matrix4f totalTransformation = planeTransformation*objectTransformation;

			totalTransformation = totalTransformation.inverse();

			Eigen::Matrix4f transMatrix = Eigen::Matrix4f::Identity();
			transMatrix(1, 1) = -1;
			transMatrix(2, 2) = -1;

			totalTransformation = transMatrix * totalTransformation;

			glLoadMatrixf(totalTransformation.data());
		} else glLoadMatrixf(viewer->getModelViewMatrix().data());

		glMatrixMode(GL_PROJECTION);
		glPushMatrix();

			glLoadMatrixf(viewer->getProjectionMatrix().data());

			glMatrixMode(GL_MODELVIEW);

			glEnable(GL_DEPTH_TEST);

			if(arDemo) {
				planeEstimator->draw();

				glEnable(GL_LIGHTING);
				if(drawARObject) arObject->draw();
				glDisable(GL_LIGHTING);
			}

			if(drawARPointcloud) viewer->getGraphDisplay()->draw();

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

    updateGL();

    //printf("load event for image \"%s\"!\n", name.c_str());
    return;
}

void ARViewer::reset(){
	delete planeEstimator;
	delete arObject;
	delete benchmarking;

    this->planeEstimator = new PlaneEstimator(viewer);
    this->arObject = new ARObject(planeEstimator);
    this->benchmarking = new Benchmarking(this);

    arDemo = false;
}

void ARViewer::keyPressEvent(QKeyEvent *ke)
{
	switch(ke->key()){
		case Qt::Key_R:
			planeEstimator->beginPlaneTracking((Eigen::Vector4f)(viewer->getModelViewMatrix().col(2)));
			arObject->init(-(Eigen::Vector4f)(viewer->getModelViewMatrix().col(2)));
			viewer->setPlaneEstimator(this->planeEstimator);
			viewer->setARObject(arObject);
			arDemo = true;
			break;
		case Qt::Key_A:
			break;
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


