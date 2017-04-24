#pragma once
#include <Kinect.h>
#include <Kinect.VisualGestureBuilder.h>

#include "Buffer.h"
#include "Eigen/Dense"

#include "StateMachine.h"
#include "MotionParameters.h"

class ControlWidget {
public: virtual void pickModel(float x, float y) {};
};

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
		Joint joints[JointType_Count];

		struct BodyProperties {
			// float headHeight;
			// float neckHeight;
			float neckToLeftShoulder;
			float neckToRightShoulder;
			float rightUpperArmLength;
			float leftUpperArmLength;
			float rightUpperLegLength;
			float leftUpperLegLength;
			float shoulderWidth;
			float torsoLength;
			float ratioBetweenTorsoLengthAndRightLeg;
			float ratioBetweenTorsoLengthAndLeftLeg;
			float ratioBetweenTorsoLengthAndShoulderWidth;
		};

		struct Person {
			INT32 id;

			_CameraSpacePoint leftHandCurrentPosition;
			_CameraSpacePoint rightHandCurrentPosition;

			_CameraSpacePoint leftHandLastPosition;
			_CameraSpacePoint rightHandLastPosition;

			HandState leftHandState;
			HandState rightHandState;

			FLOAT z;

			BodyProperties bodyProperties;
		};
		Person master;

		//@TODO in die .cpp?
		HRESULT result;

		const int POS_BUFFER_SIZE = 10;
		Buffer<CameraSpacePoint> *leftHandPositionBuffer;
		Buffer<CameraSpacePoint> *rightHandPositionBuffer;
		const int ROT_BUFFER_SIZE = 10;
		Eigen::Quaternionf lastHandOrientation; 
		bool lastHandOrientationInitialized;
		Buffer<Eigen::Quaternionf> *rotationBuffer; 
		float smoothingFactor[9] = { 1, 2, 4, 8, 16, 32, 64, 128, 256 };
		float smoothingSum;

		CameraSpacePoint* smoothSpeed(Buffer<CameraSpacePoint>* buffer);
		Eigen::Quaternionf smoothRotation(Buffer<Eigen::Quaternionf> *buffer);


		//State-Machine für KinectControl
		StateMachine stateMachine;

		void extractBodyProperties(BodyProperties* extractedBodyProperties);
		int compareToMasterProperties(BodyProperties* propertiesForComparison);

		ControlWidget *widget;
};