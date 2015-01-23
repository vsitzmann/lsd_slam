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

#include "QGLImageWindow.h"

#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>

#include <string>
#include <unordered_set>

#include <boost/thread.hpp>
#include <queue>

#include "opencv2/opencv.hpp"
#include "settings.h"

const bool useImageDisplayThread = true;

std::vector<GLImageWindow*> openWindows;
boost::mutex openCVdisplayMutex;
boost::condition_variable  openCVdisplaySignal;

boost::thread* imageDisplayThread = 0;
std::queue<DisplayImageObect> displayQueue;
bool imageThreadKeepRunning = true;

std::queue<sensor_msgs::ImageConstPtr> msgQueue;

boost::mutex keyPressMutex;
boost::condition_variable keyPressCondition;
int lastKey=0;

int idOffset = 0;
unsigned currentFrameID = 0;
bool firstPop = true;
PointCloudViewer * _viewer;

void displayThreadLoop(QApplication* app, PointCloudViewer * viewer)
{

	_viewer = viewer;

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
			bool found = false;
			for(unsigned int i=0;i<openWindows.size();i++)
			{
				if(openWindows[i]->name == displayQueue.back().name)
				{
					openWindows[i]->loadImage(displayQueue.front().img, displayQueue.front().autoSize);
					openWindows[i]->show();
					found = true;
				}
			}

			if(!found)
			{
				GLImageWindow* window = new GLImageWindow(displayQueue.front().name, 0, viewer);
				openWindows.push_back(window);
				window->loadImage(displayQueue.front().img, displayQueue.front().autoSize);
				window->show();
			}
			displayQueue.pop();
		}
	}

	for(unsigned int i=0;i<openWindows.size();i++)
	{
		openWindows[i]->close();
		delete openWindows[i];
	}
	openWindows.clear();

	printf("ended image display thread!\n");
}
void makeDisplayThread()
{
	imageThreadKeepRunning = true;
	//imageDisplayThread = new boost::thread(&displayThreadLoop);
}


void displayImage(const char* windowName, const cv::Mat& image, bool autoSize)
{
	if(debugMode) std::cout<<__PRETTY_FUNCTION__<<std::endl;

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

void popImage(int imageId){

	if(msgQueue.empty()) return;

	if(currentFrameID>imageId | firstPop){
		idOffset = msgQueue.front()->header.seq - imageId;
		firstPop = false;
	}

	currentFrameID = imageId;

	while(msgQueue.front()->header.seq<=imageId+idOffset){
		cv::Mat image =  cv_bridge::toCvShare(msgQueue.front(), "rgb8")->image;

		displayImage("AR Demo", image, false);

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

GLImageWindow::GLImageWindow(std::string name, QWidget *parent, PointCloudViewer * viewer) :
    QGLWidget(parent, viewer)
{
    setWindowTitle(tr(name.c_str()));

    image = 0;
    width_img = height_img = 0;


    show();

    resize(640, 480);
    glViewport(0,0,640,480);

    this->name = name;
    this->viewer = viewer;
    this->car = 0;

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(10);
}

GLImageWindow::~GLImageWindow()
{
    if(image != 0) delete[] image;
	delete car;
}

void GLImageWindow::initializeGL()
{
}


void GLImageWindow::paintGL()
{
	if(debugMode) std::cout<<__PRETTY_FUNCTION__<<std::endl;

    makeCurrent();
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	if(image != 0)
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

		glLoadMatrixd(viewer->modelViewMatrix.data());
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();

			glLoadMatrixd(viewer->camProjectionMatrix.data());
			glMatrixMode(GL_MODELVIEW);

			glEnable(GL_DEPTH_TEST);

			viewer->graphDisplay->draw();

			viewer->planeEstimator->draw();

		glPopMatrix();
	glPopMatrix();

	glFlush();

}


void GLImageWindow::loadImage(cv::Mat m, bool resize)
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
//    	glViewport(0,0,width_img,height_img);
//    	this->resize(width_img, height_img);
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

void GLImageWindow::keyPressEvent(QKeyEvent *ke)
{
	switch(ke->key()){
		case Qt::Key_Q:
			viewer->planeEstimator->beginPlaneTracking();
			break;
		case Qt::Key_A:

			break;
		case Qt::Key_Up:
			viewer->planeEstimator->car->moveStraight(-1);
			break;
		case Qt::Key_Down:
			viewer->planeEstimator->car->moveStraight(1);
			break;
		case Qt::Key_Left:
			viewer->planeEstimator->car->strafe(1);
			break;
		case Qt::Key_Right:
			viewer->planeEstimator->car->strafe(-1);
			break;
	}

	boost::unique_lock<boost::mutex> lock(keyPressMutex);
	lastKey = ke->key();
	keyPressCondition.notify_one();
}

