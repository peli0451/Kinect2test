#pragma once
#include <Kinect.h>
#include <Kinect.VisualGestureBuilder.h>

#include "Buffer.h"

class KinectControl {
	public:
		void init();
		void run();
		KinectControl();
	private:
		IKinectSensor *kinectSensor;
		IBodyFrameSource *bodyFrameSource;
		IBodyFrameReader *bodyFrameReader;

		INT32 numberOfTrackedBodies;
		IBody *trackedBodies[BODY_COUNT];
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

		enum ControlMode {
			DEFAULT_MODE,
			CAMERA_MODE,
			OBJECT_MODE
		};

		ControlMode currentControlMode;

		struct MotionParameters {
			float translateX;
			float translateY;
			float translateZ;
			float rotateX;
			float rotateY;
			float rotateZ;
		};

		MotionParameters motionParameters;

		HRESULT result;
};