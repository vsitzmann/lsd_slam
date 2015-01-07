/*
 * Car.cpp
 *
 *  Created on: Jan 3, 2015
 *      Author: vincent
 */

#define GL_GLEXT_PROTOTYPES 1

#include "Car.h"

#include <GL/glx.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <iostream>
#include <cmath>

#include "settings.h"

float verticeArray [] = {
    -1.0f,-1.0f,-1.0f, // triangle 1 : begin
    -1.0f,-1.0f, 1.0f,
    -1.0f, 1.0f, 1.0f, // triangle 1 : end
    1.0f, 1.0f,-1.0f, // triangle 2 : begin
    -1.0f,-1.0f,-1.0f,
    -1.0f, 1.0f,-1.0f, // triangle 2 : end
    1.0f,-1.0f, 1.0f,
    -1.0f,-1.0f,-1.0f,
    1.0f,-1.0f,-1.0f,
    1.0f, 1.0f,-1.0f,
    1.0f,-1.0f,-1.0f,
    -1.0f,-1.0f,-1.0f,
    -1.0f,-1.0f,-1.0f,
    -1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f,-1.0f,
    1.0f,-1.0f, 1.0f,
    -1.0f,-1.0f, 1.0f,
    -1.0f,-1.0f,-1.0f,
    -1.0f, 1.0f, 1.0f,
    -1.0f,-1.0f, 1.0f,
    1.0f,-1.0f, 1.0f,
    1.0f, 1.0f, 1.0f,
    1.0f,-1.0f,-1.0f,
    1.0f, 1.0f,-1.0f,
    1.0f,-1.0f,-1.0f,
    1.0f, 1.0f, 1.0f,
    1.0f,-1.0f, 1.0f,
    1.0f, 1.0f, 1.0f,
    1.0f, 1.0f,-1.0f,
    -1.0f, 1.0f,-1.0f,
    1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f,-1.0f,
    -1.0f, 1.0f, 1.0f,
    1.0f, 1.0f, 1.0f,
    -1.0f, 1.0f, 1.0f,
    1.0f,-1.0f, 1.0f
};

Car::Car(Eigen::Matrix4f initialPose, Eigen::Vector4f upVector, float sizeFactor) {
	// TODO Auto-generated constructor stub

	this->rotAngle = 0;
	this->currentPose = initialPose;
	this->upVector = upVector;

	vertexBufferID = 0;

	this->rotAngle = (10/360)*2*M_PI;

	for(int i = 0; i<36*3; i++){
		verticeArray[i] *= sizeFactor;
	}

	glGenBuffers(1, &vertexBufferID);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBufferID);         // for vertex coordinates
	glBufferData(GL_ARRAY_BUFFER, sizeof(verticeArray), verticeArray, GL_STATIC_DRAW);
}

Car::~Car() {
	// TODO Auto-generated destructor stub
}

void Car::draw(){
	if(debugMode)
		std::cerr<<__PRETTY_FUNCTION__<<std::endl;

	glMatrixMode(GL_MODELVIEW);

	glPushMatrix();
		glMultMatrixf(currentPose.data());

	// 1rst attribute buffer : vertices
		glEnableVertexAttribArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, vertexBufferID);
		glVertexAttribPointer(
		   0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
		   3,                  // size
		   GL_FLOAT,           // type
		   GL_FALSE,           // normalized?
		   0,                  // stride
		   (void*)0            // array buffer offset
		);

		// Draw the triangle !
		glDrawArrays(GL_TRIANGLES, 0, 12*3);

		glDisableVertexAttribArray(0);

	glPopMatrix();
}


void Car::rotate(int leftRight){
	Eigen::AngleAxis<float> rotation (rotAngle, upVector.normalized().topRows(3));

	Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
	rotationMatrix.topLeftCorner(3,3) = rotation.matrix();

	///rodriguezMatrix(rotationMatrix, upVector, rotAngle);

	currentPose = rotationMatrix * currentPose;
	direction = rotationMatrix * direction;
}

void Car::moveStraight(int forwBackw){
	currentPose.rightCols(1) += direction;
}

void Car::rodriguezRotate(Eigen::Vector4f axis, float theta, Eigen::Vector4f vector){
	float cosine = cos(theta);
	float sine = sin(theta);

	Eigen::Vector4f crossProduct = homogenousCrossProduct(axis, vector);

	float dotProduct = axis.dot(vector);

	vector = vector * cosine + crossProduct * sine + axis * dotProduct * (1-cosine);
}

void Car::rodriguezMatrix(Eigen::Matrix4f matrix, Eigen::Vector4f axis, float theta){
	if(debugMode)
		std::cerr<<__PRETTY_FUNCTION__<<" Identify inliers..." << std::endl;

	Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f crossProdMatrixK = Eigen::Matrix4f::Zero();

	crossProdMatrixK(0,1) = -axis(3);
	crossProdMatrixK(0,2) = axis(2);
	crossProdMatrixK(1,2) = -axis(1);

	crossProdMatrixK(1,0) = -crossProdMatrixK(0,1);
	crossProdMatrixK(2,0) = -crossProdMatrixK(0,2);
	crossProdMatrixK(2,1) = -crossProdMatrixK(1,2);

	float cosine = cos(theta);
	float sine = sin(theta);

	matrix = identity + sine * crossProdMatrixK + (1-cosine) * crossProdMatrixK * crossProdMatrixK;
}

Eigen::Vector4f Car::homogenousCrossProduct(Eigen::Vector4f a, Eigen::Vector4f b){
	Eigen::Vector3f bufferA = a.topRows(3);
	Eigen::Vector3f bufferB = b.topRows(3);

	Eigen::Vector4f resultBuffer = Eigen::Vector4f::Zero();
	resultBuffer.topRows(3) = bufferA.cross(bufferB);

	return resultBuffer;
}

