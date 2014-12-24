#pragma once
#include <opencv2/core/core.hpp>
#include <QtOpenGL>

#include "KeyFrameGraphDisplay.h"
#include "PlaneEstimator.h"
#include "PointCloudViewer.h"

class KeyFrameGraphDisplay;
class PlaneEstimator;
class PointCloudViewer;

struct DisplayImageObect
{
	cv::Mat img;
	std::string name;
	bool autoSize;
};


class GLImageWindow : public QGLWidget
{
    public:
		GLImageWindow(std::string name, QWidget *parent=0 , PointCloudViewer * viewer = 0);
       	~GLImageWindow();
    protected:
        void initializeGL();
        void paintGL();

        void keyPressEvent(QKeyEvent *ke);
    public:
        void loadImage(cv::Mat m, bool resize=false);
        void setPointCloudViewerPointer(PointCloudViewer * viewer);

        std::string name;
    private:

        void showTitlePage();

        unsigned char* image;
    private:
        PlaneEstimator * planeEstimator;
        KeyFrameGraphDisplay * graphDisplay;
        PointCloudViewer * viewer;

        int width_img, height_img;


    };

void displayImage(const char* windowName, const cv::Mat& image, bool autoSize = true);
int waitKey(int milliseconds);
void closeAllWindows();


void displayThreadLoop(QApplication* app=0, PointCloudViewer * viewer = 0);



