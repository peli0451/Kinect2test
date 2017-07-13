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
		WAITABLE_HANDLE frameArrivedHandle;

		boolean masterDetermined;
		boolean collectFrames;
		int framesLeftToCollect;
		int maxFramesToCollect;
		const int MIN_FRAMES_TO_COLLECT = 60;

		//Kinect Tracking-Variablen
		INT32 numberOfTrackedBodies;
		IBody *trackedBodies[BODY_COUNT] = { 0,0,0,0,0,0 };
		UINT64 trackingId[BODY_COUNT];
		bool collectDeviation[BODY_COUNT];
		Buffer<float> *deviationBuffer[BODY_COUNT];
		const int NUMBER_OF_COLLECTED_FRAMES = 20;
		float evaluateDeviationBuffer(Buffer<float> *deviationBuffer);
		bool isInConfigurationPose(Joint* joints);
		const float MASTER_ALLOWED_DEVIATION = 50.f;

		float getSaneValue(float old, float fresh);
		const float MAX_SANE_DISTANCE = 0.1f; // Wert geraten. Könnte man auch getSaneValue übergeben, wenn es variieren soll
		const float MAX_STEP = 0.05f; // muss <= MAX_SANE_DISTANCe sein

		//State-Machine für KinectControl
		StateMachine stateMachine;
};