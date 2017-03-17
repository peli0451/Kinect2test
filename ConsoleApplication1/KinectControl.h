#pragma once
#include <Kinect.h>
#include <Kinect.VisualGestureBuilder.h>

#include "Buffer.h"
#include "Eigen/Dense"


class KinectControl {
	public:
		struct MotionParameters {
			float translateX;
			float translateY;
			float translateZ;
			Eigen::Quaternionf rotate;
		};

		void init();
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

		struct Person {
			INT32 id;

			_CameraSpacePoint leftHandCurrentPosition;
			_CameraSpacePoint rightHandCurrentPosition;

			_CameraSpacePoint leftHandLastPosition;
			_CameraSpacePoint rightHandLastPosition;

			HandState leftHandState;
			HandState rightHandState;

			FLOAT z;
		};
		Person master;

		MotionParameters motionParameters;

		enum Gesture {
			UNKNOWN,
			TRANSLATE_GESTURE,
			ROTATE_GESTURE,
			GRAB_GESTURE
		};
		const int GESTURE_COUNT = 4;
		Gesture recognizedGesture;
		const int GESTURE_BUFFER_SIZE = 10;
		Buffer<Gesture> *recognizedGesturesBuffer;
		Gesture evaluateGestureBuffer();

		Gesture getRecognizedGesture();
		void setRecognizedGesture(Gesture gesture);

		//@TODO in die .cpp?
		HRESULT result;

		const int POS_BUFFER_SIZE = 10;
		Buffer<CameraSpacePoint> *leftHandPositionBuffer;
		Buffer<CameraSpacePoint> *rightHandPositionBuffer;
		float smoothing_factor[9] = { 1, 2, 4, 8, 16, 32, 64, 128, 256 };
		float smoothing_sum;

		CameraSpacePoint* smooth_speed(Buffer<CameraSpacePoint>* buffer);


		//State-Machine für KinectControl
		enum KinectControlState {
			CAMERA_IDLE,
			CAMERA_TRANSLATE,
			CAMERA_ROTATE,
			OBJECT_IDLE,
			OBJECT_TRANSLATE,
			OBJECT_ROTATE
		};

		KinectControlState state;
		void setState(KinectControlState newState);
		KinectControlState getState();

		MotionParameters getMotion();
		void setMotion(float translateX, float translateY, float translateZ, Eigen::Quaternionf rotate);
		void setTranslation(float translateX, float translateY, float translateZ);
		void setRotation(Eigen::Quaternionf rotate);
		void resetMotion();
		void resetTranslation();
		void resetRotation();

		void stateMachineCompute();
		void stateMachineSwitchState();
};