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

#include "QGLImageWindow.h"

#include <opencv2/opencv.hpp>

#include <string>
#include <unordered_set>

#include <boost/thread.hpp>

const bool useImageDisplayThread = true;

std::vector<GLImageWindow*> openWindows;
boost::mutex openCVdisplayMutex;
boost::condition_variable  openCVdisplaySignal;


boost::thread* imageDisplayThread = 0;
std::vector<DisplayImageObect> displayQueue;
bool imageThreadKeepRunning = true;


boost::mutex keyPressMutex;
boost::condition_variable keyPressCondition;
int lastKey=0;

void displayThreadLoop(QApplication* app, PointCloudViewer * viewer)
{
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
					openWindows[i]->loadImage(displayQueue.back().img, displayQueue.back().autoSize);
					openWindows[i]->show();
					found = true;
				}
			}

			if(!found)
			{
				GLImageWindow* window = new GLImageWindow(displayQueue.back().name, 0, viewer);
				openWindows.push_back(window);
				window->loadImage(displayQueue.back().img, displayQueue.back().autoSize);
				window->show();
			}
			displayQueue.pop_back();
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
	if(useImageDisplayThread)
	{
		boost::unique_lock<boost::mutex> lock(openCVdisplayMutex);
		displayQueue.push_back(DisplayImageObect());
		displayQueue.back().autoSize = autoSize;
		displayQueue.back().img = image.clone();
		displayQueue.back().name = windowName;

		openCVdisplaySignal.notify_one();
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
    QGLWidget(parent)
{
    setWindowTitle(tr(name.c_str()));

    image = 0;
    width_img = height_img = 0;


    show();

    resize(640, 480);
    glViewport(0,0,640,480);

    this->name = name;
    this->planeEstimator = viewer->planeEstimator;
    this->graphDisplay = viewer->graphDisplay;
    this->viewer = viewer;
    this->car = 0;

}

GLImageWindow::~GLImageWindow()
{
    if(image != 0) delete[] image;
    delete planeEstimator;
}

void GLImageWindow::initializeGL()
{

}

void GLImageWindow::paintGL()
{
    makeCurrent();
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

	if(image != 0)
	{
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();

		glRasterPos2f( -1,1);
		glPixelZoom( width() / (float)width_img, -height()/(float)height_img );
		glDrawPixels( width_img, height_img, GL_BGR, GL_UNSIGNED_BYTE, image);
	}

	if(car != 0){
		car->draw();
	}

	glPushMatrix();
		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixd(viewer->modelViewMatrix.data());
		glMatrixMode(GL_PROJECTION);
		glLoadMatrixd(viewer->camProjectionMatrix.data());

		planeEstimator->draw();
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
    	//glViewport(0,0,width_img,height_img);
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

void GLImageWindow::keyPressEvent(QKeyEvent *ke)
{
	switch(ke->key()){
		case Qt::Key_Q:
			planeEstimator->beginPlaneTracking();
			break;
		case Qt::Key_A:
			initARDemo();
			break;
	}

	boost::unique_lock<boost::mutex> lock(keyPressMutex);
	lastKey = ke->key();
	keyPressCondition.notify_one();
}

void GLImageWindow::initARDemo(){
//	Eigen::Matrix4f initialCarPose = Eigen::Matrix4f::Identity();
//	initialCarPose.rightCols(3) = this->planeEstimator->center;
//	Eigen::Vector4f upVector = this->planeEstimator->bitangent.cross(this->planeEstimator->tangent);
//
//	this->car = new Car(initialCarPose, upVector);
}
