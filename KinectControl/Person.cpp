#include "stdafx.h"
#include "Person.h"


const int ERROR_NTLS = 0;
const int ERROR_NTRS = 0;
const int ERROR_LUAL = 0;
const int ERROR_RUAL = 0;
const int ERROR_RULL = 0;
const int ERROR_SW = 0;
const int ERROR_TL = 0;
const int ERROR_RBTLALL = 0;
const int ERROR_RBTLARL = 0;
const int ERROR_RBTLASW = 0;

/**********************************************************
* Konstruktoren
**********************************************************/

Person::Person()
{
	id = -1;

	leftHandCurrentPosition = { 0,0,0 };
	rightHandCurrentPosition = { 0,0,0 };

	leftHandLastPosition = { 0,0,0 };
	rightHandLastPosition = { 0,0,0 };

	leftHandState = HandState_Unknown;
	rightHandState = HandState_Unknown;

	z = FLT_MAX;

	// Handpositionenbuffer
	leftHandPositionBuffer = new Buffer<CameraSpacePoint>(POS_BUFFER_SIZE);
	rightHandPositionBuffer = new Buffer<CameraSpacePoint>(POS_BUFFER_SIZE);

	// Rotationenbuffer
	rotationBuffer = new Buffer<Eigen::Quaternionf>(ROT_BUFFER_SIZE);
}



/**********************************************************
* Getter und Setter
**********************************************************/

void Person::setId(int newId) {
	id = newId;
}

int Person::getId() { 
	return id; 
}



void Person::setZ(float newZ) {
	z = newZ; 
}

float Person::getZ() { 
	return z; 
}



void Person::setJoints(Joint* newJoints) {
	for(int i = 0; i<JointType_Count; i++)
		joints[i] = newJoints[i];
}

Joint* Person::getJoints() { 
	return joints; 
}



void Person::setJointOrientations(JointOrientation* newOrientations) {
	for (int i = 0; i<JointType_Count; i++)
		jointOrientations[i] = newOrientations[i];
}

JointOrientation* Person::getJointOrientations() {
	return jointOrientations;
}



void Person::setLeftHandCurPos(_CameraSpacePoint newPos) { 
	leftHandCurrentPosition = newPos; 
};

_CameraSpacePoint Person::getLeftHandCurPos() {
	return leftHandCurrentPosition; 
};



void Person::setLeftHandLastPos(_CameraSpacePoint newPos) { 
	leftHandLastPosition = newPos; 
};

_CameraSpacePoint Person::getLeftHandLastPos() { 
	return leftHandLastPosition; 
};



void Person::setRightHandCurPos(_CameraSpacePoint newPos) { 
	rightHandCurrentPosition = newPos; 
};

_CameraSpacePoint Person::getRightHandCurPos() {
	return rightHandCurrentPosition;
};



void Person::setRightHandLastPos(_CameraSpacePoint newPos) { 
	rightHandLastPosition = newPos; 
};

_CameraSpacePoint Person::getRightHandLastPos() {
	return rightHandLastPosition; 
};



void Person::setLeftHandState(HandState newState) {
	leftHandState = newState; 
};

HandState Person::getLeftHandState() { 
	return leftHandState; 
};



void Person::setRightHandState(HandState newState) { 
	rightHandState = newState; 
};

HandState Person::getRightHandState() {
	return rightHandState; 
};



Buffer<_CameraSpacePoint>* Person::getLeftHandPosBuffer() {
	return leftHandPositionBuffer;
}



Buffer<_CameraSpacePoint>* Person::getRightHandPosBuffer() {
	return rightHandPositionBuffer;
}



Buffer<Eigen::Quaternionf>* Person::getRotationBuffer() {
	return rotationBuffer;
}



void Person::setLastHandOrientation(Eigen::Quaternionf orientation) {
	lastHandOrientation = orientation;
	lastHandOrientationInitialized = true;
}

Eigen::Quaternionf Person::getLastHandOrientation() {
	return lastHandOrientation;
}



void Person::setLastHandOrientationInitialized(bool val) {
	lastHandOrientationInitialized = val;
}

bool Person::isLastHandOrientationInitialized() {
	return lastHandOrientationInitialized;
}



void Person::setControlHand(GestureRecognition::ControlHand hand) {
	controlHand = hand;
}

GestureRecognition::ControlHand Person::getControlHand() {
	return controlHand;
}



void Person::setRisenHand(GestureRecognition::ControlHand hand) {
	risenHand = hand;
}

GestureRecognition::ControlHand Person::getRisenHand() {
	return risenHand;
}
/**********************************************************
* Funktionen
**********************************************************/

void Person::extractBodyProperties(float* extractedBodyProperties, Joint* inputJoints)
{
	_CameraSpacePoint shoulderLeft = inputJoints[JointType::JointType_ShoulderLeft].Position;
	_CameraSpacePoint shoulderRight = inputJoints[JointType::JointType_ShoulderRight].Position;
	_CameraSpacePoint spineShoulder = inputJoints[JointType::JointType_SpineShoulder].Position;
	_CameraSpacePoint elbowLeft = inputJoints[JointType::JointType_ElbowLeft].Position;
	_CameraSpacePoint elbowRight = inputJoints[JointType::JointType_ElbowRight].Position;
	_CameraSpacePoint neck = inputJoints[JointType::JointType_Neck].Position;
	_CameraSpacePoint spineBase = inputJoints[JointType::JointType_SpineBase].Position;
	_CameraSpacePoint hipLeft = inputJoints[JointType::JointType_HipLeft].Position;
	_CameraSpacePoint hipRight = inputJoints[JointType::JointType_HipRight].Position;
	_CameraSpacePoint kneeLeft = inputJoints[JointType::JointType_KneeLeft].Position;
	_CameraSpacePoint kneeRight = inputJoints[JointType::JointType_KneeRight].Position;


	extractedBodyProperties[NECK_TO_LEFT_SHOULDER] = sqrt(pow(shoulderLeft.X - neck.X, 2) +
		pow(shoulderLeft.Y - neck.Y, 2) + pow(shoulderLeft.Z - neck.Z, 2));
	extractedBodyProperties[NECK_TO_RIGHT_SHOULDER] = sqrt(pow(shoulderRight.X - neck.X, 2) +
		pow(shoulderRight.Y - neck.Y, 2) + pow(shoulderRight.Z - neck.Z, 2));
	extractedBodyProperties[LEFT_UPPER_ARM_LENGTH] = sqrt(pow(shoulderLeft.X - elbowLeft.X, 2) +
		pow(shoulderLeft.Y - elbowLeft.Y, 2) + pow(shoulderLeft.Z - elbowLeft.Z, 2));
	extractedBodyProperties[RIGHT_UPPER_ARM_LENGTH] = sqrt(pow(shoulderRight.X - elbowRight.X, 2) +
		pow(shoulderRight.Y - elbowRight.Y, 2) + pow(shoulderRight.Z - elbowRight.Z, 2));
	extractedBodyProperties[LEFT_UPPER_LEG_LENGTH] = sqrt(pow(hipLeft.X - kneeLeft.X, 2) +
		pow(hipLeft.Y - kneeLeft.Y, 2) + pow(hipLeft.Z - kneeLeft.Z, 2));
	extractedBodyProperties[RIGHT_UPPER_LEG_LENGTH] = sqrt(pow(hipRight.X - kneeRight.X, 2) +
		pow(hipRight.Y - kneeRight.Y, 2) + pow(hipRight.Z - kneeRight.Z, 2));
	extractedBodyProperties[SHOULDER_WIDTH] = sqrt(pow(shoulderLeft.X - shoulderRight.X, 2) +
		pow(shoulderLeft.Y - shoulderRight.Y, 2) + pow(shoulderLeft.Z - shoulderRight.Z, 2));
	extractedBodyProperties[TORSO_LENGTH] = sqrt(pow(spineShoulder.X - spineBase.X, 2) +
		pow(spineShoulder.Y - spineBase.Y, 2) + pow(spineShoulder.Z - spineBase.Z, 2));

	extractedBodyProperties[RATIO_BETWEEN_TORSO_LENGTH_AND_LEFT_LEG] =
		extractedBodyProperties[TORSO_LENGTH] / extractedBodyProperties[LEFT_UPPER_LEG_LENGTH];
	extractedBodyProperties[RATIO_BETWEEN_TORSO_LENGTH_AND_RIGHT_LEG] =
		extractedBodyProperties[TORSO_LENGTH] / extractedBodyProperties[RIGHT_UPPER_LEG_LENGTH];
	extractedBodyProperties[RATIO_BETWEEN_TORSO_LENGTH_AND_SHOULDER_WIDTH] =
		extractedBodyProperties[TORSO_LENGTH] / extractedBodyProperties[SHOULDER_WIDTH];
}

void Person::saveBodyProperties()
{
	extractBodyProperties(bodyProperties, joints);
}


float Person::compareBodyProperties (Joint* inputJoints) {
	float propertiesForComparison[NUMBER_OF_BODY_PROPERTIES];
	extractBodyProperties(propertiesForComparison, inputJoints);
	
	float confidence;

	for (int i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {
		if (bodyProperties[i] < propertiesForComparison[i])	confidence += bodyProperties[i]/propertiesForComparison[i];
		else confidence += propertiesForComparison[i]/bodyProperties[i];
	}
	
	return confidence / NUMBER_OF_BODY_PROPERTIES;
}
