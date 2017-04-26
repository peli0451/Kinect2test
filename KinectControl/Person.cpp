#include "stdafx.h"
#include "Person.h"



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

	bodyProperties = { 0.f };

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



Person::BodyProperties Person::getBodyProperties() {
	return bodyProperties;
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

void Person::extractBodyProperties(BodyProperties* extractedBodyProperties)
{
	_CameraSpacePoint shoulderLeft = joints[JointType::JointType_ShoulderLeft].Position;
	_CameraSpacePoint shoulderRight = joints[JointType::JointType_ShoulderRight].Position;
	_CameraSpacePoint spineShoulder = joints[JointType::JointType_SpineShoulder].Position;
	_CameraSpacePoint elbowLeft = joints[JointType::JointType_ElbowLeft].Position;
	_CameraSpacePoint elbowRight = joints[JointType::JointType_ElbowRight].Position;
	_CameraSpacePoint neck = joints[JointType::JointType_Neck].Position;
	_CameraSpacePoint spineBase = joints[JointType::JointType_SpineBase].Position;
	_CameraSpacePoint hipLeft = joints[JointType::JointType_HipLeft].Position;
	_CameraSpacePoint hipRight = joints[JointType::JointType_HipRight].Position;
	_CameraSpacePoint kneeLeft = joints[JointType::JointType_KneeLeft].Position;
	_CameraSpacePoint kneeRight = joints[JointType::JointType_KneeRight].Position;


	extractedBodyProperties->neckToLeftShoulder = sqrt(pow(shoulderLeft.X - neck.X, 2) +
		pow(shoulderLeft.Y - neck.Y, 2) + pow(shoulderLeft.Z - neck.Z, 2));
	extractedBodyProperties->neckToRightShoulder = sqrt(pow(shoulderRight.X - neck.X, 2) +
		pow(shoulderRight.Y - neck.Y, 2) + pow(shoulderRight.Z - neck.Z, 2));
	extractedBodyProperties->leftUpperArmLength = sqrt(pow(shoulderLeft.X - elbowLeft.X, 2) +
		pow(shoulderLeft.Y - elbowLeft.Y, 2) + pow(shoulderLeft.Z - elbowLeft.Z, 2));
	extractedBodyProperties->rightUpperArmLength = sqrt(pow(shoulderRight.X - elbowRight.X, 2) +
		pow(shoulderRight.Y - elbowRight.Y, 2) + pow(shoulderRight.Z - elbowRight.Z, 2));
	extractedBodyProperties->leftUpperLegLength = sqrt(pow(hipLeft.X - kneeLeft.X, 2) +
		pow(hipLeft.Y - kneeLeft.Y, 2) + pow(hipLeft.Z - kneeLeft.Z, 2));
	extractedBodyProperties->leftUpperLegLength = sqrt(pow(hipRight.X - kneeRight.X, 2) +
		pow(hipRight.Y - kneeRight.Y, 2) + pow(hipRight.Z - kneeRight.Z, 2));
	extractedBodyProperties->shoulderWidth = sqrt(pow(shoulderLeft.X - shoulderRight.X, 2) +
		pow(shoulderLeft.Y - shoulderRight.Y, 2) + pow(shoulderLeft.Z - shoulderRight.Z, 2));
	extractedBodyProperties->torsoLength = sqrt(pow(spineShoulder.X - spineBase.X, 2) +
		pow(spineShoulder.Y - spineBase.Y, 2) + pow(spineShoulder.Z - spineBase.Z, 2));

	extractedBodyProperties->ratioBetweenTorsoLengthAndLeftLeg =
		extractedBodyProperties->torsoLength / extractedBodyProperties->leftUpperLegLength;
	extractedBodyProperties->ratioBetweenTorsoLengthAndRightLeg =
		extractedBodyProperties->torsoLength / extractedBodyProperties->rightUpperLegLength;
	extractedBodyProperties->ratioBetweenTorsoLengthAndShoulderWidth =
		extractedBodyProperties->torsoLength / extractedBodyProperties->shoulderWidth;
}

