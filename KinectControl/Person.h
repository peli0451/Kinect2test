#pragma once
#include <Kinect.h>
#include "Buffer.h"
#include "Eigen/Dense"
#include "GestureRecognition.h"

class Person {
public:

	static const int POS_BUFFER_SIZE = 10;
	static const int ROT_BUFFER_SIZE = 10;
	static const int NUMBER_OF_BODY_PROPERTIES = 11;

	Person();

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

	void setLastHandOrientation(Eigen::Quaternionf orientation);
	Eigen::Quaternionf getLastHandOrientation();
	void setLastHandOrientationInitialized(bool val);
	bool isLastHandOrientationInitialized();

	void setControlHand(GestureRecognition::ControlHand hand);
	GestureRecognition::ControlHand getControlHand();
	void setRisenHand(GestureRecognition::ControlHand hand);
	GestureRecognition::ControlHand getRisenHand();

	void saveBodyProperties();
	float compareBodyProperties(Joint* inputJoints);

private:
	struct Limits
	{
		float min;
		float max;
	};

	enum BODY_PROPERTIES {NECK_TO_LEFT_SHOULDER, NECK_TO_RIGHT_SHOULDER, LEFT_UPPER_ARM_LENGTH,
		RIGHT_UPPER_ARM_LENGTH, LEFT_UPPER_LEG_LENGTH, RIGHT_UPPER_LEG_LENGTH, SHOULDER_WIDTH,
		TORSO_LENGTH, RATIO_BETWEEN_TORSO_LENGTH_AND_LEFT_LEG, RATIO_BETWEEN_TORSO_LENGTH_AND_RIGHT_LEG,
		RATIO_BETWEEN_TORSO_LENGTH_AND_SHOULDER_WIDTH
	};

	Limits bodyPropertiesLimits[NUMBER_OF_BODY_PROPERTIES];
	float bodyProperties[NUMBER_OF_BODY_PROPERTIES] = { 0.f };
	float bodyPropertiesWeights[6] = { 1.0f, 0.7f, 0.5f, 0.3f, 0.1f, 0.0f };
	int numberOfWeights;

	int id;

	_CameraSpacePoint leftHandCurrentPosition;
	_CameraSpacePoint rightHandCurrentPosition;

	_CameraSpacePoint leftHandLastPosition;
	_CameraSpacePoint rightHandLastPosition;

	HandState leftHandState;
	HandState rightHandState;

	float z;

	Joint joints[JointType_Count];
	JointOrientation jointOrientations[JointType_Count];

	Buffer<CameraSpacePoint> *leftHandPositionBuffer;
	Buffer<CameraSpacePoint> *rightHandPositionBuffer;

	Buffer<Eigen::Quaternionf> *rotationBuffer;

	Eigen::Quaternionf lastHandOrientation;
	bool lastHandOrientationInitialized;

	GestureRecognition::ControlHand controlHand;
	GestureRecognition::ControlHand risenHand;

	void extractBodyProperties(float* extractedBodyProperties, Joint* inputJoints);
};