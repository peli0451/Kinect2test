#pragma once
#include <Kinect.h>
#include <Kinect.VisualGestureBuilder.h>

#include "Buffer.h"
#include "Eigen/Dense"

class ControlWidget {
	public: virtual void pickModel(float x, float y) {};
};

class KinectControl {
	public:
		enum MotionTarget {
			TARGET_OBJECT = false,
			TARGET_CAMERA = true
		};
		struct MotionParameters {
			float translateX;
			float translateY;
			float translateZ;
			Eigen::Quaternionf rotate;
			MotionTarget target; //0-verändere Model, 1-verändere Kamera
		};

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

		MotionParameters motionParameters;

		//TODO Translate -> TranslateCamera?
		enum Gesture {
			UNKNOWN,
			TRANSLATE_GESTURE,
			ROTATE_GESTURE,
			GRAB_GESTURE,
		};

		struct GestureConfidence {
			float unknownConfidence;
			float translateCameraConfidence;
			float rotateCameraConfidence;
			float grabConfidence;
		};

		const int GESTURE_COUNT = 4;
		Gesture recognizedGesture;
		const int GESTURE_BUFFER_SIZE = 10;
		Buffer<GestureConfidence> *gestureConfidenceBuffer;
		Gesture evaluateGestureBuffer();
		float gestureSmooth[10] = { 1, 2, 4, 8, 16, 32, 64, 128, 256, 512 };
		float gestureSmoothSum = 1023;

		Gesture getRecognizedGesture();
		void setRecognizedGesture(Gesture gesture);

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
		enum KinectControlState {
			CAMERA_IDLE,
			CAMERA_TRANSLATE,
			CAMERA_ROTATE,
			OBJECT_MANIPULATE
		};

		KinectControlState state;
		void setState(KinectControlState newState);
		KinectControlState getState();

		enum ControlHand {
			HAND_LEFT = 0,
			HAND_RIGHT = 1
		};
		ControlHand controlHand;
		ControlHand risenHand;

		MotionParameters getMotion();
		void setMotion(float translateX, float translateY, float translateZ, Eigen::Quaternionf rotate, MotionTarget target);
		void setTranslation(float translateX, float translateY, float translateZ);
		void setRotation(Eigen::Quaternionf rotate);
		void setTarget(MotionTarget target);
		void resetMotion();
		void resetTranslation();
		void resetRotation();

		void stateMachineBufferGestureConfidence();
		void stateMachineCompute();
		void stateMachineSwitchState();

		void extractBodyProperties(BodyProperties* extractedBodyProperties);
		int compareToMasterProperties(BodyProperties* propertiesForComparison);

		ControlWidget *widget;
};