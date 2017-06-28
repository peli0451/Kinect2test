#pragma once
#include <Kinect.h>
#include <Kinect.VisualGestureBuilder.h>

#include "Buffer.h"
#include "Eigen/Dense"

#include "GestureRecognition.h"
#include "MotionParameters.h"
#include "Person.h"

class ControlWidget {
public: virtual void pickModel(float x, float y) {};
};

class StateMachine {
public:
	enum State {
		IDLE,
		CAMERA_TRANSLATE,
		CAMERA_ROTATE,
		OBJECT_MANIPULATE,
		FLY
	};
	
	void setState(State newState);
	State getState();

	void setMotionParameters(MotionParameters motionParameters);
	MotionParameters getMotionParameters();

	void setMaster(Person newMaster);
	Person getMaster();

	void bufferGestureConfidence();
	void compute();
	void switchState();

	void assignWidget(ControlWidget *_widget);

	StateMachine();

	static Eigen::Vector3f convToVec3(CameraSpacePoint csp);
	static Eigen::Vector3f convToVec3(CameraSpacePoint *csp);
	static Eigen::AngleAxisf getRotationAngleAxis(Eigen::Vector3f originAxis, Eigen::Vector3f targetAxis);
private:
	State state;
	GestureRecognition gestureRecognition;
	MotionParameters motionParameters;
	Person master;

	const float smoothingFactor[9] = { 1, 2, 4, 8, 16, 32, 64, 128, 256 };
	const float smoothingSum = 511.0f; //Summe obiger Einträge

	const float cameraRotationSmoothingFactor[20] = { .0f, .0f, .0f, .0f, .0f, .0f, .0f, .0f, .0f, .0f, 1.f, 1.5f, 
													2.25f, 3.375f, 5.0625f, 7.6f, 11.39f, 17.086f, 25.63f, 38.44f };
	float cameraRotationSmoothingSum = 0.0f; //Summe obiger Einträge
	
	const float rotationSmoothingFactor[20] = { 1.0f, 1.3f, 1.69f, 2.197f, 2.856f, 3.713f, 4.827f, 6.275f, 8.157f, 10.604f, 
										10.604f, 10.604f, 10.604f, 10.604f, 10.604f, 10.604f, 10.604f, 10.604f, 10.604f, 10.604f };
	float rotationSmoothingSum = 0.0f; //Summe obiger Einträge

	const float FLY_TRANSLATION_FACTOR = -0.01f;
	const float FLY_ROTATION_FACTOR = 0.015f;
	const float FLY_ROTATION_ROLL_FACTOR = 0.02f;
	const float FLY_SEGMENT2_DEGREE = 10.0f;
	const float FLY_SEGMENT2_FACTOR = 1.5f;
	const float OBJECT_MAX_ROTATION = 0.2f;
	const float OBJECT_ROTATION_FACTOR = 10.f;
	CameraSpacePoint smoothSpeed(Buffer<CameraSpacePoint>* buffer);
	Eigen::Quaternionf smoothRotation(Buffer<Eigen::Quaternionf> *buffer, const float* smoothingFactor, float smoothingSum);

	ControlWidget *widget;
};