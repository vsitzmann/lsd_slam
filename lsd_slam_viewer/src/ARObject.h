/*
 * ARObject.h
 *
 *  Created on: Feb 10, 2015
 *      Author: vincent
 */

#ifndef SRC_AROBJECT_H_
#define SRC_AROBJECT_H_

#include <GL/glu.h>
#include "PlaneEstimator.h"

#include <Eigen/Core>

using namespace std;

class PlaneEstimator;

class ARObject {
public:
	ARObject(PlaneEstimator * planeEstimator);
	virtual ~ARObject();

	void draw();
	void rotate(int leftRight);
	void flipNormal();
	void init(const Eigen::Vector4f & cameraViewDirection);
	void accelerate(int direction);
	void stop();
	Eigen::Matrix4f *getPose();
	void toggleCollisionChecking();
	int getNormalSign();

private:
	Eigen::Matrix <float, 4, 4, Eigen::ColMajor> currentPose;

	vector<Eigen::Vector4f> vertices;
	vector<Eigen::Vector3f> normals;
	vector<Eigen::Vector3f> colors;
	vector<GLushort> elements;

	PlaneEstimator * planeEstimator;

	float maxVelocity;
	float velocity;
	float acceleration;
	float rotAngle;
	int accelerationDirection;

	unsigned int vbo_mesh_vertices, vbo_mesh_normals, ibo_mesh_elements, vbo_mesh_colors;

	void load_obj(const char* filename);
	void updatePosition(double deltaT);
};

#endif /* SRC_AROBJECT_H_ */
