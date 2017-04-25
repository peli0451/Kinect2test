#pragma once
#include <Kinect.h>
#include "Buffer.h"
#include "Eigen/Dense"
#include "GestureRecognition.h"

class Person {
public:
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

	static const int POS_BUFFER_SIZE = 10;
	static const int ROT_BUFFER_SIZE = 10;

	Person();

	void extractBodyProperties(BodyProperties* extractedBodyProperties);

	void setId(int id);
	int getId();

	void setZ(float newZ);
	float getZ();
	
	void setJoints(Joint* newJoints);
	Joint* getJoints();

	void setJointOrientations(JointOrientation* newOrientations);
	JointOrientation* getJointOrientations();

	void setLeftHandCurPos(_CameraSpacePoint newPos);
	_CameraSpacePoint getLeftHandCurPos();

	void setLeftHandLastPos(_CameraSpacePoint newPos);
	_CameraSpacePoint getLeftHandLastPos();

	void setRightHandCurPos(_CameraSpacePoint newPos);
	_CameraSpacePoint getRightHandCurPos();

	void setRightHandLastPos(_CameraSpacePoint newPos);
	_CameraSpacePoint getRightHandLastPos();

	void setLeftHandState(HandState newState);
	HandState getLeftHandState();

	void setRightHandState(HandState newState);
	HandState getRightHandState();

	Buffer<CameraSpacePoint>* getLeftHandPosBuffer();
	Buffer<CameraSpacePoint>* getRightHandPosBuffer();
	Buffer<Eigen::Quaternionf>* getRotationBuffer();

	BodyProperties getBodyProperties();

	void setLastHandOrientation(Eigen::Quaternionf orientation);
	Eigen::Quaternionf getLastHandOrientation();
	void setLastHandOrientationInitialized(bool val);
	bool isLastHandOrientationInitialized();

	void setControlHand(GestureRecognition::ControlHand hand);
	GestureRecognition::ControlHand getControlHand();
	void setRisenHand(GestureRecognition::ControlHand hand);
	GestureRecognition::ControlHand getRisenHand();
private:
	int id;

	_CameraSpacePoint leftHandCurrentPosition;
	_CameraSpacePoint rightHandCurrentPosition;

	_CameraSpacePoint leftHandLastPosition;
	_CameraSpacePoint rightHandLastPosition;

	HandState leftHandState;
	HandState rightHandState;

	float z;

	BodyProperties bodyProperties;

	Joint joints[JointType_Count];
	JointOrientation jointOrientations[JointType_Count];

	Buffer<CameraSpacePoint> *leftHandPositionBuffer;
	Buffer<CameraSpacePoint> *rightHandPositionBuffer;

	Buffer<Eigen::Quaternionf> *rotationBuffer;

	Eigen::Quaternionf lastHandOrientation;
	bool lastHandOrientationInitialized;

	GestureRecognition::ControlHand controlHand;
	GestureRecognition::ControlHand risenHand;
};