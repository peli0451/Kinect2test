#include "stdafx.h"
#include "Person.h"
#include <cmath>

//#define DEBUG_BODY_PROPERTIES

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


	bodyPropertiesLimits[LEFT_UPPER_ARM_LENGTH].min = 0.1f;
	bodyPropertiesLimits[LEFT_UPPER_ARM_LENGTH].max = 0.6f;
	bodyPropertiesLimits[RIGHT_UPPER_ARM_LENGTH].min = bodyPropertiesLimits[LEFT_UPPER_ARM_LENGTH].min;
	bodyPropertiesLimits[RIGHT_UPPER_ARM_LENGTH].max = bodyPropertiesLimits[LEFT_UPPER_ARM_LENGTH].max;
	bodyPropertiesLimits[LEFT_LOWER_ARM_LENGTH].min = 0.1f;
	bodyPropertiesLimits[LEFT_LOWER_ARM_LENGTH].max = 0.6f;
	bodyPropertiesLimits[RIGHT_LOWER_ARM_LENGTH].min = bodyPropertiesLimits[LEFT_UPPER_ARM_LENGTH].min;
	bodyPropertiesLimits[RIGHT_LOWER_ARM_LENGTH].max = bodyPropertiesLimits[LEFT_UPPER_ARM_LENGTH].max;
	bodyPropertiesLimits[LEFT_UPPER_LEG_LENGTH].min = 0.2f;
	bodyPropertiesLimits[LEFT_UPPER_LEG_LENGTH].max = 0.8f;
	bodyPropertiesLimits[RIGHT_UPPER_LEG_LENGTH].min = bodyPropertiesLimits[LEFT_UPPER_LEG_LENGTH].min;
	bodyPropertiesLimits[RIGHT_UPPER_LEG_LENGTH].max = bodyPropertiesLimits[LEFT_UPPER_LEG_LENGTH].max;
	bodyPropertiesLimits[SHOULDER_WIDTH].min = 0.2f;
	bodyPropertiesLimits[SHOULDER_WIDTH].max = 0.7f;
	bodyPropertiesLimits[HIP_WIDTH].min = 0.2f;
	bodyPropertiesLimits[HIP_WIDTH].max = 0.7f;
	bodyPropertiesLimits[TORSO_LENGTH].min = 0.2f;
	bodyPropertiesLimits[TORSO_LENGTH].max = 1.0f;
	bodyPropertiesLimits[HEIGHT_OF_HEAD].min = 1.0f;
	bodyPropertiesLimits[HEIGHT_OF_HEAD].max = 2.5f;

	numberOfWeights = sizeof(bodyPropertiesWeights) / sizeof(float);
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
	for (int i = 0; i<JointType_Count; i++)
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
	_CameraSpacePoint handLeft = inputJoints[JointType::JointType_HandLeft].Position;
	_CameraSpacePoint handRight = inputJoints[JointType::JointType_HandRight].Position;
	_CameraSpacePoint spineShoulder = inputJoints[JointType::JointType_SpineShoulder].Position;
	_CameraSpacePoint elbowLeft = inputJoints[JointType::JointType_ElbowLeft].Position;
	_CameraSpacePoint elbowRight = inputJoints[JointType::JointType_ElbowRight].Position;
	_CameraSpacePoint neck = inputJoints[JointType::JointType_Neck].Position;
	_CameraSpacePoint spineBase = inputJoints[JointType::JointType_SpineBase].Position;
	_CameraSpacePoint hipLeft = inputJoints[JointType::JointType_HipLeft].Position;
	_CameraSpacePoint hipRight = inputJoints[JointType::JointType_HipRight].Position;
	_CameraSpacePoint kneeLeft = inputJoints[JointType::JointType_KneeLeft].Position;
	_CameraSpacePoint kneeRight = inputJoints[JointType::JointType_KneeRight].Position;
	_CameraSpacePoint head = inputJoints[JointType::JointType_Head].Position;

	JointTrackingState shoulderLeftState = inputJoints[JointType::JointType_ShoulderLeft].TrackingState;
	JointTrackingState shoulderRightState = inputJoints[JointType::JointType_ShoulderRight].TrackingState;
	JointTrackingState handLeftState = inputJoints[JointType::JointType_HandLeft].TrackingState;
	JointTrackingState handRightState = inputJoints[JointType::JointType_HandRight].TrackingState;
	JointTrackingState spineShoulderState = inputJoints[JointType::JointType_SpineShoulder].TrackingState;
	JointTrackingState elbowLeftState = inputJoints[JointType::JointType_ElbowLeft].TrackingState;
	JointTrackingState elbowRightState = inputJoints[JointType::JointType_ElbowRight].TrackingState;
	JointTrackingState neckState = inputJoints[JointType::JointType_Neck].TrackingState;
	JointTrackingState spineBaseState = inputJoints[JointType::JointType_SpineBase].TrackingState;
	JointTrackingState hipLeftState = inputJoints[JointType::JointType_HipLeft].TrackingState;
	JointTrackingState hipRightState = inputJoints[JointType::JointType_HipRight].TrackingState;
	JointTrackingState kneeLeftState = inputJoints[JointType::JointType_KneeLeft].TrackingState;
	JointTrackingState kneeRightState = inputJoints[JointType::JointType_KneeRight].TrackingState;
	JointTrackingState headState = inputJoints[JointType::JointType_Head].TrackingState;


	if (shoulderLeftState == Tracked && elbowLeftState == Tracked) {
		extractedBodyProperties[LEFT_UPPER_ARM_LENGTH] = sqrt(pow(shoulderLeft.X - elbowLeft.X, 2.0f) +
			pow(shoulderLeft.Y - elbowLeft.Y, 2.0f) + pow(shoulderLeft.Z - elbowLeft.Z, 2.0f));
	}
	else {
		extractedBodyProperties[LEFT_UPPER_ARM_LENGTH] = 0.0f;
	}

	if (shoulderRightState == Tracked && elbowRightState == Tracked) {
		extractedBodyProperties[RIGHT_UPPER_ARM_LENGTH] = sqrt(pow(shoulderRight.X - elbowRight.X, 2.0f) +
			pow(shoulderRight.Y - elbowRight.Y, 2.0f) + pow(shoulderRight.Z - elbowRight.Z, 2.0f));
	}
	else {
		extractedBodyProperties[RIGHT_UPPER_ARM_LENGTH] = 0.0f;
	}

	if (elbowLeftState == Tracked && handLeftState == Tracked) {
		extractedBodyProperties[LEFT_LOWER_ARM_LENGTH] = sqrt(pow(handLeft.X - elbowLeft.X, 2.0f) +
			pow(handLeft.Y - elbowLeft.Y, 2.0f) + pow(handLeft.Z - elbowLeft.Z, 2.0f));
	}
	else {
		extractedBodyProperties[LEFT_LOWER_ARM_LENGTH] = 0.0f;
	}

	if (elbowRightState == Tracked && handRightState == Tracked) {
		extractedBodyProperties[RIGHT_LOWER_ARM_LENGTH] = sqrt(pow(handRight.X - elbowRight.X, 2.0f) +
			pow(handRight.Y - elbowRight.Y, 2.0f) + pow(handRight.Z - elbowRight.Z, 2.0f));
	}
	else {
		extractedBodyProperties[RIGHT_LOWER_ARM_LENGTH] = 0.0f;
	}

	if (hipLeftState == Tracked && kneeLeftState == Tracked) {
		extractedBodyProperties[LEFT_UPPER_LEG_LENGTH] = sqrt(pow(hipLeft.X - kneeLeft.X, 2.0f) +
			pow(hipLeft.Y - kneeLeft.Y, 2.0f) + pow(hipLeft.Z - kneeLeft.Z, 2.0f));
	}
	else {
		extractedBodyProperties[LEFT_UPPER_LEG_LENGTH] = 0.0f;
	}

	if (hipRightState == Tracked && kneeRightState == Tracked) {
		extractedBodyProperties[RIGHT_UPPER_LEG_LENGTH] = sqrt(pow(hipRight.X - kneeRight.X, 2.0f) +
			pow(hipRight.Y - kneeRight.Y, 2.0f) + pow(hipRight.Z - kneeRight.Z, 2.0f));
	}
	else {
		extractedBodyProperties[RIGHT_UPPER_LEG_LENGTH] = 0.0f;
	}

	if (shoulderLeftState == Tracked && shoulderRightState == Tracked) {
		extractedBodyProperties[SHOULDER_WIDTH] = sqrt(pow(shoulderLeft.X - shoulderRight.X, 2.0f) +
			pow(shoulderLeft.Y - shoulderRight.Y, 2.0f) + pow(shoulderLeft.Z - shoulderRight.Z, 2.0f));
	}
	else {
		extractedBodyProperties[SHOULDER_WIDTH] = 0.0f;
	}

	if (hipLeftState == Tracked && hipRightState == Tracked) {
		extractedBodyProperties[HIP_WIDTH] = sqrt(pow(hipLeft.X - hipRight.X, 2.0f) +
			pow(hipLeft.Y - hipRight.Y, 2.0f) + pow(hipLeft.Z - hipRight.Z, 2.0f));
	}
	else {
		extractedBodyProperties[HIP_WIDTH] = 0.0f;
	}

	if (spineShoulderState == Tracked && spineBaseState == Tracked) {
		extractedBodyProperties[TORSO_LENGTH] = sqrt(pow(spineShoulder.X - spineBase.X, 2.0f) +
			pow(spineShoulder.Y - spineBase.Y, 2.0f) + pow(spineShoulder.Z - spineBase.Z, 2.0f));
	}
	else {
		extractedBodyProperties[TORSO_LENGTH] = 0.0f;
	}

	if (headState == Tracked) {
		extractedBodyProperties[HEIGHT_OF_HEAD] = head.Y;
	}
	else {
		extractedBodyProperties[HEIGHT_OF_HEAD] = 0.0f;
	}
}

void Person::saveBodyProperties()
{
	extractBodyProperties(bodyProperties, joints);
}

void Person::collectBodyProperties()
{
	float* bodyPropertiesTemp = new float[NUMBER_OF_BODY_PROPERTIES];
	extractBodyProperties(bodyPropertiesTemp, joints);

	bodyPropertiesBuffer.push_back(bodyPropertiesTemp);
}

void Person::calculateBodyProperties()
{
	std::list<float*>::iterator liter;
	float* bodyPropertiesTemp;
	int numberOfSamples[NUMBER_OF_BODY_PROPERTIES];
	int i;

	if (bodyPropertiesBuffer.empty())
		return;

	for (i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {
		bodyProperties[i] = 0.0f;
	}

	for (i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {
		numberOfSamples[i] = 0;
	}


	for (liter = bodyPropertiesBuffer.begin(); liter != bodyPropertiesBuffer.end(); liter++) {
		bodyPropertiesTemp = *liter;

		for (i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {
			if (bodyPropertiesTemp[i] != 0.0f) {
				bodyProperties[i] += bodyPropertiesTemp[i];
				numberOfSamples[i]++;
			}
		}
		delete[] bodyPropertiesTemp;
	}

	for (i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {
		if (numberOfSamples[i] != 0 && bodyProperties[i] != 0.0f) {
			bodyProperties[i] /= numberOfSamples[i];
		}
	}

	bodyPropertiesBuffer.clear();
}


float Person::compareBodyProperties(Joint* inputJoints) {
	float propertiesForComparison[NUMBER_OF_BODY_PROPERTIES];
	extractBodyProperties(propertiesForComparison, inputJoints);
	float deviation;
	int weightIndex;

	float sumOfWeights = 0.0f;
	float confidence = 0.0f;

	for (int i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {
		if (propertiesForComparison[i] < bodyPropertiesLimits[i].min) {
			deviation = bodyPropertiesLimits[i].min - propertiesForComparison[i];
		}
		else if (bodyProperties[i] <= bodyPropertiesLimits[i].max) {
			deviation = 0.0f;
		}
		else {
			deviation = propertiesForComparison[i] - bodyPropertiesLimits[i].max;
		}

		weightIndex = static_cast<int>(deviation*10.0f);
		//OutputDebugStringA(std::to_string(weightIndex).c_str());

		if (weightIndex >= numberOfWeights) {
			weightIndex = numberOfWeights - 1;
		}

		if (bodyProperties[i] == 0.0f || propertiesForComparison[i] == 0.0f) {
			weightIndex = numberOfWeights - 1;
		}

		sumOfWeights += bodyPropertiesWeights[weightIndex];

#ifdef DEBUG_BODY_PROPERTIES
		OutputDebugStringA("Confidence for BodyProperty No ");
		OutputDebugStringA(std::to_string(i).c_str());
		OutputDebugStringA(": ");
		OutputDebugStringA(std::to_string(min(bodyProperties[i] / propertiesForComparison[i], propertiesForComparison[i] / bodyProperties[i])).c_str());
		OutputDebugStringA("\n");
#endif

		if (bodyProperties[i] < propertiesForComparison[i]) {
			confidence += (bodyProperties[i] / propertiesForComparison[i]) * bodyPropertiesWeights[weightIndex];
		}
		else {
			confidence += (propertiesForComparison[i] / bodyProperties[i]) * bodyPropertiesWeights[weightIndex];
		}
	}

	if (sumOfWeights != 0.0f)
		return confidence / sumOfWeights;
	else
		return 0.0f;
}