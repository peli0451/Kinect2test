#pragma once
#include <Kinect.h>
#include <Kinect.VisualGestureBuilder.h>

#include "Buffer.h"
#include "Eigen/Dense"

#include "Person.h"
#include "StateMachine.h"
#include "MotionParameters.h"

class ControlWidget;

class KinectControl {
	public:
		void init(ControlWidget *_widget);
		MotionParameters run();
		void assignMaster();
		KinectControl();
	private:
		//Kinect Basics
		IKinectSensor *kinectSensor;
		IBodyFrameSource *bodyFrameSource;
		IBodyFrameReader *bodyFrameReader;

		boolean masterDetermined;

		//Kinect Tracking-Variablen
		INT32 numberOfTrackedBodies;
		IBody *trackedBodies[BODY_COUNT] = { 0,0,0,0,0,0 };
		UINT64 trackingId[BODY_COUNT];
		bool collectDeviation[BODY_COUNT];
		Buffer<float> *deviations[BODY_COUNT];
		const int NUMBER_OF_COLLECTED_FRAMES = 20;
		float evaluateDeviationBuffer(Buffer<float> *deviationBuffer);
		bool isInConfigurationPose(Joint* joints);

		//@TODO in die .cpp?
		HRESULT result;

		//State-Machine für KinectControl
		StateMachine stateMachine;
};