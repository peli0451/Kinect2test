#pragma once
#include <Kinect.h>
#include <Kinect.VisualGestureBuilder.h>

#include "Buffer.h"
#include "Eigen/Dense"

#include "GestureRecognition.h"
#include "MotionParameters.h"

class StateMachine {
public:
	enum State {
		IDLE,
		CAMERA_TRANSLATE,
		CAMERA_ROTATE,
		OBJECT_MANIPULATE
	};

	void setState(State newState);
	State getState();

	void setMotionParameters(MotionParameters motionParameters);
	MotionParameters getMotionParameters();

	void bufferGestureConfidence();
	void compute();
	void switchState();

	StateMachine();
private:
	State state;
	GestureRecognition gestureRecognition;
	MotionParameters motionParameters;
};