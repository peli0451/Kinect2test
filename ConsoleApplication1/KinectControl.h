#pragma once
#include <Kinect.h>
#include <Kinect.VisualGestureBuilder.h>

#include "Buffer.h"

class KinectControl {
	public:
		struct MotionParameters {
			float translateX;
			float translateY;
			float translateZ;
			float rotateX;
			float rotateY;
			float rotateZ;
		};

		void init();
		void run(MotionParameters *motionParameters);
		KinectControl();
	private:
		IKinectSensor *kinectSensor;
		IBodyFrameSource *bodyFrameSource;
		IBodyFrameReader *bodyFrameReader;

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

		enum ControlMode {
			DEFAULT_MODE,
			CAMERA_MODE,
			OBJECT_MODE
		};

		ControlMode currentControlMode;

		HRESULT result;

		const int POS_BUFFER_SIZE = 10;
		Buffer<CameraSpacePoint> *leftHandPositionBuffer;
		Buffer<CameraSpacePoint> *rightHandPositionBuffer;
		float smoothing_factor[9] = { 1,2,3,4,6,8,12,14,20 };
		float smoothing_sum;

		CameraSpacePoint* smooth_speed(Buffer<CameraSpacePoint>* buffer);
};