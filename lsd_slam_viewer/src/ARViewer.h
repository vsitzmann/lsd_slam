/*
 * ARViewer.h
 *
 *  Created on: Feb 10, 2015
 *      Author: vincent
 */

#ifndef SRC_ARVIEWER_H_
#define SRC_ARVIEWER_H_

#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <QtOpenGL>

#include "KeyFrameGraphDisplay.h"
#include "PlaneEstimator.h"
#include "PointCloudViewer.h"
#include "ARObject.h"
#include "Benchmarking.h"

class KeyFrameGraphDisplay;
class PlaneEstimator;
class PointCloudViewer;
class ARObject;
class Benchmarking;

struct DisplayImageObect
{
	cv::Mat img;
	std::string name;
	bool autoSize;
};

class ARViewer : public QGLWidget{
public:
	ARViewer(std::string name, QWidget *parent=0 , PointCloudViewer * viewer = 0);
	virtual ~ARViewer();
protected:
    void initializeGL();
    void paintGL();

    void keyPressEvent(QKeyEvent *ke);
    void keyReleaseEvent(QKeyEvent *ke);
public:
    void loadImage(cv::Mat m, bool resize=false);
    void setPointCloudViewerPointer(PointCloudViewer * viewer);
    void reset();

    std::string name;
    PointCloudViewer * viewer;
    PlaneEstimator * planeEstimator;
    ARObject * arObject;
private:

    void showTitlePage();
    void initARDemo();
private:
    Benchmarking * benchmarking;

    bool arDemo;
	bool ego;
    int width_img, height_img;
    unsigned char* image;
};

void displayImage(const char* windowName, const cv::Mat& image, bool autoSize = true);
void enqueueImage(const sensor_msgs::ImageConstPtr& msg);
void popImage(unsigned int imageId);
int waitKey(int milliseconds);
void closeAllWindows();
void stopDisplayThreadLoop();

void displayThreadLoop(QApplication* app=0, PointCloudViewer * viewer = 0);


#endif /* SRC_ARVIEWER_H_ */
