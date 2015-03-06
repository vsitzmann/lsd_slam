/*
 * ARObject.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: vincent
 */

#define GL_GLEXT_PROTOTYPES 1

#include "ARObject.h"

#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <fstream>
#include <cmath>

#include "settings.h"

#include <cstdio>
#include <ctime>

using namespace std;

std::clock_t lastDrawTime = 0;

ARObject::ARObject(PlaneEstimator * planeEstimator) {
	load_obj("/home/vincent/suzanne.obj");

	this->planeEstimator = planeEstimator;

	rotAngle = M_PI/20;

	velocity = 0;
	maxVelocity = 0;
	acceleration = 0;
	accelerationDirection = 0;
	collisionChecking = false;
}

ARObject::~ARObject() {
	// TODO Auto-generated destructor stub
}

void ARObject::init(const Eigen::Vector4f & cameraViewDirection){
	this->currentPose = Eigen::Matrix4f::Identity();

	float sceneScale = planeEstimator->getSceneScale();
	maxVelocity = 0.5 * sceneScale;
	acceleration = 0.1 * sceneScale;
	accelerationDirection = 0;
	velocity = 0;

	//Check whether the plane normal points upwards or downwards by calculating the dot product of the
	//plane normal and the camera's view direction.
	if(planeEstimator->getPlaneParameters().col(2).dot(planeEstimator->getPlaneParameters().col(2))>0)
		currentPose.col(2) = currentPose.col(2)*(-1);
}

Eigen::Matrix4f * ARObject::getPose(){
	return &(this->currentPose);
}

void ARObject::draw(){
	glMatrixMode(GL_MODELVIEW);

	updatePosition((std::clock()-lastDrawTime)/ (double) CLOCKS_PER_SEC);
	lastDrawTime = std::clock();

	glGenBuffers(1, &vbo_mesh_vertices);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_mesh_vertices);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector4f)*vertices.size(), &vertices[0], GL_STATIC_DRAW);

	glGenBuffers(1, &vbo_mesh_normals);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_mesh_normals);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f)*normals.size(), &normals[0], GL_STATIC_DRAW);

	glGenBuffers(1, &vbo_mesh_colors);
	glBindBuffer(GL_ARRAY_BUFFER, vbo_mesh_colors);
	glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f)*colors.size(), &colors[0], GL_STATIC_DRAW);


	glGenBuffers(1, &ibo_mesh_elements);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_mesh_elements);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLushort)*elements.size(), &elements[0], GL_STATIC_DRAW);

	glPushMatrix();
		glMultMatrixf(planeEstimator->getPlaneParameters().data());
		glMultMatrixf(currentPose.data());

		  glEnableVertexAttribArray(0);
		  glBindBuffer(GL_ARRAY_BUFFER, vbo_mesh_vertices);
		  glVertexAttribPointer(
		    0,  // attribute
		    4,                  // number of elements per vertex, here (x,y,z,w)
		    GL_FLOAT,           // the type of each element
		    GL_FALSE,           // take our values as-is
		    0,                  // no extra data between each position
		    0                   // offset of first element
		  );

		  glEnableVertexAttribArray(2);
		  glBindBuffer(GL_ARRAY_BUFFER, vbo_mesh_normals);
		  glVertexAttribPointer(
		    2, // attribute
		    3,                  // number of elements per vertex, here (x,y,z)
		    GL_FLOAT,           // the type of each element
		    GL_FALSE,           // take our values as-is
		    0,                  // no extra data between each position
		    0                   // offset of first element
		  );

		  glEnableVertexAttribArray(1);
		  glBindBuffer(GL_ARRAY_BUFFER, vbo_mesh_colors);
		  glVertexAttribPointer(
		    1, // attribute
		    3,                  // number of elements per vertex, here (x,y,z)
		    GL_FLOAT,           // the type of each element
		    GL_FALSE,           // take our values as-is
		    0,                  // no extra data between each position
		    0                   // offset of first element
		  );

	 	  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo_mesh_elements);
		  glDrawElements(GL_TRIANGLES, elements.size(), GL_UNSIGNED_SHORT, 0);

		glDisableVertexAttribArray(0);
		glDisableVertexAttribArray(1);
		glDisableVertexAttribArray(2);

	glPopMatrix();

	glDeleteBuffers(1, &vbo_mesh_colors);
	glDeleteBuffers(1, &vbo_mesh_normals);
	glDeleteBuffers(1, &vbo_mesh_vertices);
	glDeleteBuffers(1, &ibo_mesh_elements);
}

void ARObject::rotate(int leftRight){

	Eigen::Vector4f rotAxis = currentPose.col(2);
	Eigen::Vector3f rotAxis_3d = rotAxis.topRows(3).normalized();
	Eigen::AngleAxis<float> rotation (rotAngle, rotAxis_3d);

	Eigen::Matrix3f rotMatrix;

	if(leftRight == 1) rotMatrix = rotation.matrix();
	else rotMatrix = rotation.matrix().inverse();

	currentPose.topLeftCorner(3,3) = rotMatrix * currentPose.topLeftCorner(3,3);
}

void ARObject::flipNormal(){
	currentPose.col(2) = currentPose.col(2) * -1;
}

void ARObject::load_obj(const char* filename) {
  ifstream in(filename, ios::in);
  if (!in) { cerr << "Cannot open " << filename << endl; exit(1); }

  string line;
  while (getline(in, line)) {
    if (line.substr(0,2) == "v ") {
      istringstream s(line.substr(2));
      Eigen::Vector4f v;

      s >> v[0];
      s >> v[1];
      s >> v[2];
      v[3] = 1.0f;

      colors.push_back(Eigen::Vector3f(1,1,1));
      vertices.push_back(v);
    }  else if (line.substr(0,2) == "f ") {
      istringstream s(line.substr(2));
      GLushort a,b,c;
      s >> a; s >> b; s >> c;
      a--; b--; c--;
      elements.push_back(a); elements.push_back(b); elements.push_back(c);
    }
    else if (line[0] == '#') { /* ignoring this line */ }
    else { /* ignoring this line */ }
  }

  normals.resize(vertices.size(), Eigen::Vector3f(0.0, 0.0, 0.0));

  for (unsigned int i = 0; i < elements.size(); i+=3) {
    GLushort ia = elements[i];
    GLushort ib = elements[i+1];
    GLushort ic = elements[i+2];

    Eigen::Vector3f bufferB = vertices[ib].topRows(3);
    Eigen::Vector3f bufferA = vertices[ia].topRows(3);
    Eigen::Vector3f bufferC = vertices[ic].topRows(3);

    Eigen::Vector3f cross = (bufferB - bufferA).cross(bufferC - bufferA);

    Eigen::Vector3f normal = cross.normalized();
    normals[ia] = normals[ib] = normals[ic] = normal;
  }
}

void ARObject::accelerate(int direction){
	accelerationDirection = direction;
}

void ARObject::stop(){
	accelerationDirection = 0;
	velocity = 0;
}

void ARObject::updatePosition(double deltaT){
	if(accelerationDirection == 0 ){
		if(velocity>0) velocity -= deltaT * acceleration;
		else if(velocity<0 ) velocity += deltaT * acceleration;

		if(std::abs(velocity)<maxVelocity/100 ) velocity = 0;
	} else{
		velocity += deltaT * acceleration * accelerationDirection;

		if(velocity>maxVelocity) velocity=maxVelocity;
	}

	Eigen::Vector4f newPosition = currentPose.col(3)  + currentPose.col(0) * velocity;

	if(!collisionChecking) currentPose.col(3) = newPosition;
	else if(!planeEstimator->checkCollision(newPosition)) currentPose.col(3) = newPosition;
	else(stop());
}

void ARObject::toggleCollisionChecking(){
	collisionChecking = !collisionChecking;
}

int ARObject::getNormalSign(){
	return currentPose(2,2);
}

