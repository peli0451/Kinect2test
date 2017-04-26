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
		KinectControl();
	private:
		//Kinect Basics
		IKinectSensor *kinectSensor;
		IBodyFrameSource *bodyFrameSource;
		IBodyFrameReader *bodyFrameReader;


		//Kinect Tracking-Variablen
		INT32 numberOfTrackedBodies;
		IBody *trackedBodies[BODY_COUNT] = { 0,0,0,0,0,0 };

		//@TODO in die .cpp?
		HRESULT result;

		//State-Machine für KinectControl
		StateMachine stateMachine;

		int compareToMasterProperties(Person::BodyProperties* propertiesForComparison);
};