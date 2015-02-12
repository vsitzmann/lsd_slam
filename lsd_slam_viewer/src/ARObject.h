/*
 * ARObject.h
 *
 *  Created on: Feb 10, 2015
 *      Author: vincent
 */

#ifndef SRC_AROBJECT_H_
#define SRC_AROBJECT_H_

#include <GL/glu.h>

#include <Eigen/Core>

using namespace std;

class ARObject {
public:
	ARObject();
	virtual ~ARObject();

	void draw();
	void moveStraight(int forwBackw);
	void rotate(int leftRight);
	void strafe(int leftRight);
	void updatePlane(Eigen::Vector3f normal, Eigen::Vector3f inlier);
	void flipNormal();
	void setPose(Eigen::Matrix4f initialPose);
	void setNormal(Eigen::Matrix4f planeParameters);

private:
	Eigen::Matrix <float, 4, 4, Eigen::ColMajor> currentPose;

	vector<Eigen::Vector4f> vertices;
	vector<Eigen::Vector3f> normals;
	vector<Eigen::Vector3f> colors;
	vector<GLushort> elements;

	float speed;
	float rotAngle;

	unsigned int vbo_mesh_vertices, vbo_mesh_normals, ibo_mesh_elements, vbo_mesh_colors;

	void load_obj(const char* filename);
};

#endif /* SRC_AROBJECT_H_ */
