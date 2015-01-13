/*
 * Car.h
 *
 *  Created on: Jan 3, 2015
 *      Author: vincent
 */

#ifndef SRC_CAR_H_
#define SRC_CAR_H_

#include <Eigen/Core>
#include <Eigen/Geometry>


class Car {
public:
	Car(Eigen::Matrix4f initialPose, Eigen::Vector4f upVector, float sizeFactor);
	virtual ~Car();

	void draw();
	void moveStraight(int forwBackw);
	void rotate(int leftRight);
	void strafe(int leftRight);
	void updatePlane(Eigen::Vector3f normal, Eigen::Vector3f inlier);

	Eigen::Matrix <float, 4, 4, Eigen::ColMajor> currentPose;

private:
	Eigen::Vector4f upVector;
	Eigen::Vector4f baseRotMatrix;
	Eigen::Vector4f direction;

	float speed;
	float rotAngle;
	unsigned int vertexBufferID;
	int vertexBufferNumPoints;

	Eigen::Vector4f homogenousCrossProduct(Eigen::Vector4f a, Eigen::Vector4f b);
	void rodriguezRotate(Eigen::Vector4f axis, float theta, Eigen::Vector4f vector);
	void rodriguezMatrix(Eigen::Matrix4f matrix, Eigen::Vector4f axis, float theta);

};

#endif /* SRC_CAR_H_ */
