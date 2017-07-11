#pragma once
#include <Kinect.h>
#include "Buffer.h"
#include "Eigen/Dense"
#include "GestureRecognition.h"
#include <list>

class Person {
public:

	static const int POS_BUFFER_SIZE = 10;
	static const int ROT_BUFFER_SIZE = 40;

	Person();

	void setId(int id);
	int getId();

	void setTrackingId(UINT64 tid);
	UINT64 getTrackingId();

	void setZ(float newZ);
	float getZ();

	void setJoints(Joint* newJoints);
	Joint* getJoints();

	void setJointOrientations(JointOrientation* newOrientations);
	JointOrientation* getJointOrientations();

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
	bool collectBodyProperties();
	void deleteCollectedBodyProperties();
	bool calculateBodyProperties();
	

private:
	struct Limits
	{
		float min;
		float max;
	};

	enum BODY_PROPERTIES {
		LEFT_UPPER_ARM_LENGTH, RIGHT_UPPER_ARM_LENGTH, LEFT_LOWER_ARM_LENGTH, RIGHT_LOWER_ARM_LENGTH,
		SHOULDER_WIDTH, HIP_WIDTH, TORSO_LENGTH, NECK_TO_HEAD, NUMBER_OF_BODY_PROPERTIES
	};

	Limits bodyPropertiesLimits[NUMBER_OF_BODY_PROPERTIES];
	std::list<float*> bodyPropertiesBuffer;
	float bodyProperties[NUMBER_OF_BODY_PROPERTIES] = { 0.f, 0.f, 0.0f, 0.0f, 0.0f, 0.f, 0.f,0.f};
	float bodyPropertiesWeights[6] = { 1.0f, 0.7f, 0.5f, 0.3f, 0.1f, 0.0f };
	int numberOfWeights;
	float standardDeviations[NUMBER_OF_BODY_PROPERTIES];
	float bodyPropertiesFactors[NUMBER_OF_BODY_PROPERTIES] = { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
	float PERMITTED_QUANTIL = 2.0f;

	int id;
	UINT64 trackingId;

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

	bool extractBodyProperties(float* extractedBodyProperties, Joint* inputJoints);
	bool isTracked(float* testBodyProperties);
};