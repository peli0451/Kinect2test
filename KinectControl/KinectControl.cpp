/**
* KinectControl.cpp
* Managerklasse, Schnittstelle zur Kinect
*/

#include "stdafx.h"
#include <string>
#include <iostream>
#include "KinectControl.h"
//TODO in Konfiguration kann man Directories für Headerdateien angeben

/* 
* Globale Konstanten
*/

const float MAX_SANE_DISTANCE = 0.1f; // Wert geraten. Könnte man auch getSaneValue übergeben, wenn es variieren soll
const float MAX_STEP = 0.05f; // muss <= MAX_SANE_DISTANCe sein


/**
* Konstruktor
*/
KinectControl::KinectControl() {
	// Master
	master = {
		-1,						// id
		{ 0,0,0 },				// leftHandCurrentPosition
		{ 0,0,0 },				// rightHandCurrentPosition
		{ 0,0,0 },				// leftHandLastPosition
		{ 0,0,0 },				// rightHandLastPosition
		HandState_Unknown,		// leftHandState
		HandState_Unknown,		// rightHandState
		100.0,					// z
		0.0f
	};

	//TODO [deprecated] hier war früher Init für Statemachine und Gesture

	// Handpositionenbuffer
	leftHandPositionBuffer = new Buffer<CameraSpacePoint>(POS_BUFFER_SIZE);
	rightHandPositionBuffer = new Buffer<CameraSpacePoint>(POS_BUFFER_SIZE);

	// Rotationenbuffer
	rotationBuffer = new Buffer<Eigen::Quaternionf>(ROT_BUFFER_SIZE);
}

/**
* Initialisiert eine KinectControl-Instanz
* Bereitet Kinect-Sensor und -Frame-Reader vor
* Bereitet Puffer vor
*/
void KinectControl::init(ControlWidget *_widget) {
	widget = _widget;
	GetDefaultKinectSensor(&kinectSensor);
	kinectSensor->Open();
	kinectSensor->get_BodyFrameSource(&bodyFrameSource);
	bodyFrameSource->get_BodyCount(&numberOfTrackedBodies); //Anzahl Personen?
	bodyFrameSource->OpenReader(&bodyFrameReader);
	smoothingSum = 0;
	for (int i = 0; i < POS_BUFFER_SIZE; i++) {
		smoothingSum += smoothingFactor[i];
	}
	//[deprecated] Initialisierung der motionParameters
	//stateMachine.getMotionParameters().resetMotion();
}

/**
* Glättet gepufferte Positionen
*
* @param buffer Puffer für Positionen
*/
CameraSpacePoint* KinectControl::smoothSpeed(Buffer<CameraSpacePoint>* buffer) {
	CameraSpacePoint speedPoint = { 0,0,0 };
	// läuft atm vom ältesten zum jüngsten, deshalb schleife absteigend
	for (int i = buffer->end(); i >= 1; i--) {
		// von jung zu alt, stoppt eins später für ableitung
		// ich achte mal noch nich drauf was bei nicht vollem puffer genau passiert
		CameraSpacePoint *curPoint = buffer->get(i);
		CameraSpacePoint *nextPoint = buffer->get(i - 1);
		float smoothing = smoothingFactor[i-1] / smoothingSum;

		/*
		OutputDebugStringA(std::to_string(smoothing).c_str());
		OutputDebugStringA("\t");
		OutputDebugStringA(std::to_string(smoothing_factor[i-1]).c_str());
		OutputDebugStringA("\t");
		OutputDebugStringA(std::to_string(smoothing_sum).c_str());
		OutputDebugStringA("\n");
		*/

		speedPoint.X += (curPoint->X - nextPoint->X) * smoothing;
		speedPoint.Y += (curPoint->Y - nextPoint->Y) * smoothing;
		speedPoint.Z += (curPoint->Z - nextPoint->Z) * smoothing;
	}
	return &speedPoint;
}

/**
* Glättet gepufferte Rotationen (in Form von Quaternions)
*
* @param buffer Puffer für Rotationen
*/
Eigen::Quaternionf KinectControl::smoothRotation(Buffer<Eigen::Quaternionf> *buffer) {
	Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
	for (int i = buffer->end(); i >= 0; i--) {
		Eigen::Quaternionf *cur_rot = buffer->get(i);
		float smoothing = smoothingFactor[i] / smoothingSum; // verwendet zurzeit dieselben faktoren wie die Positionsglättung
		Eigen::Quaternionf downscaled_rot = Eigen::Quaternionf::Identity().slerp(smoothing, *cur_rot); // skaliert einen Puffereintrag
		rotation *= downscaled_rot;
	}
	return rotation;
}

/**
* Testet die Differenz zweier (X-/Y-/Z-)Positionswerte, ob sie vernünftig ist
* @param old Die Position aus dem letzten Frame
* @param fresh Die Position aus dem aktuellen Frame
* @return Position, die definierte Abweichung nicht überschreitet, sich aber fresh annähert
*/
float getSaneValue(float old, float fresh) {

	if (fresh - old > MAX_SANE_DISTANCE) {
		return old + MAX_STEP;
	}
	if (fresh - old < -MAX_SANE_DISTANCE) {
		return old - MAX_STEP;
	}
	return fresh; // Bewegungsdistanz liegt innerhalb der Schwelle
}

void KinectControl::extractBodyProperties(BodyProperties* extractedBodyProperties)
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

int KinectControl::compareToMasterProperties(BodyProperties* propertiesForComparison) {
	int matchedProperties = 0;

	if (abs(propertiesForComparison->neckToLeftShoulder - master.bodyProperties.neckToLeftShoulder) < ERROR_NTLS)
		matchedProperties++;
	if (abs(propertiesForComparison->neckToRightShoulder - master.bodyProperties.neckToRightShoulder) < ERROR_NTRS)
		matchedProperties++;
	if (abs(propertiesForComparison->leftUpperArmLength - master.bodyProperties.leftUpperArmLength) < ERROR_LUAL)
		matchedProperties++;
	if (abs(propertiesForComparison->rightUpperArmLength - master.bodyProperties.rightUpperArmLength) < ERROR_RUAL)
		matchedProperties++;
	if (abs(propertiesForComparison->leftUpperLegLength - master.bodyProperties.leftUpperLegLength) < ERROR_RULL)
		matchedProperties++;
	if (abs(propertiesForComparison->shoulderWidth - master.bodyProperties.shoulderWidth) < ERROR_SW)
		matchedProperties++;
	if (abs(propertiesForComparison->torsoLength - master.bodyProperties.torsoLength) < ERROR_TL)
		matchedProperties++;
	if (abs(propertiesForComparison->ratioBetweenTorsoLengthAndLeftLeg -
		master.bodyProperties.ratioBetweenTorsoLengthAndLeftLeg) < ERROR_RBTLALL)
		matchedProperties++;
	if (abs(propertiesForComparison->ratioBetweenTorsoLengthAndRightLeg -
		master.bodyProperties.ratioBetweenTorsoLengthAndRightLeg) < ERROR_RBTLARL)
		matchedProperties++;
	if (abs(propertiesForComparison->ratioBetweenTorsoLengthAndShoulderWidth -
		master.bodyProperties.ratioBetweenTorsoLengthAndShoulderWidth) < ERROR_RBTLASW)
		matchedProperties++;

	return matchedProperties;
}

/**
* Hole und bearbeite Frame, den Kinect liefert
*
* @return motionParameters Transformationsparameter
*/
MotionParameters KinectControl::run() {

	//Plan: Iterieren über Köpfe, den niedrigsten z-Wert als Master wählen, Mastervariable
	master.id = -1;
	master.z = 100.0;

	IBodyFrame *bodyFrame;

	result = bodyFrameReader->AcquireLatestFrame(&bodyFrame);
	if (result != S_OK) { return motionParameters; }

	result = bodyFrame->GetAndRefreshBodyData(numberOfTrackedBodies, trackedBodies); //Update für bodies-Array
	if (result != S_OK) { bodyFrame->Release(); return motionParameters; }

	for (int i = 0; i < numberOfTrackedBodies; ++i)
	{
		BOOLEAN isTracked;
		trackedBodies[i]->get_IsTracked(&isTracked); //ist i-te potentielle Person getrackt

		//Tracking erkannter Personen, Identifikation des Masters
		if (isTracked == TRUE) { //Wenn getrackt, lege Gelenkmodell an (Gelenkarray)
			result = trackedBodies[i]->GetJoints(JointType_Count, joints);

			if (SUCCEEDED(result)) { //Falls Gelenke erfolgreich geholt
				auto position(joints[JointType::JointType_Head].Position);  //Weißt position den Position-struct vom joint zu (referenziell)
				if (position.Z < master.z) {
					master.id = i;
					master.z = position.Z;
				}

			}
		}
	}

	//@TODO Wenn Master wechselt muss der Ringpuffer für die Positionen neu initialisiert werden (mit der Position des neuen Masters)
	//@TODO Mastererkennung mit Confidence, nicht direkt
	if (master.id != -1) {
		//Hole Gelenkobjekte und wichtige Positionen des Masters
		trackedBodies[master.id]->GetJoints(JointType_Count, joints);
		trackedBodies[master.id]->get_HandLeftState(&master.leftHandState);
		trackedBodies[master.id]->get_HandRightState(&master.rightHandState);
		master.leftHandCurrentPosition = joints[JointType::JointType_HandLeft].Position;
		master.rightHandCurrentPosition = joints[JointType::JointType_HandRight].Position;

		// Sanity-Check der Positionswerte
		CameraSpacePoint sane_value_left;
		CameraSpacePoint sane_value_right;
		
		if (leftHandPositionBuffer->get(leftHandPositionBuffer->end()) != NULL && rightHandPositionBuffer->get(rightHandPositionBuffer->end()) != NULL) {
			sane_value_left.X = getSaneValue(leftHandPositionBuffer->get(leftHandPositionBuffer->end())->X, master.leftHandCurrentPosition.X);
			sane_value_left.Y = getSaneValue(leftHandPositionBuffer->get(leftHandPositionBuffer->end())->Y, master.leftHandCurrentPosition.Y);
			sane_value_left.Z = getSaneValue(leftHandPositionBuffer->get(leftHandPositionBuffer->end())->Z, master.leftHandCurrentPosition.Z);
			sane_value_right.X = getSaneValue(rightHandPositionBuffer->get(rightHandPositionBuffer->end())->X, master.rightHandCurrentPosition.X);
			sane_value_right.Y = getSaneValue(rightHandPositionBuffer->get(rightHandPositionBuffer->end())->Y, master.rightHandCurrentPosition.Y);
			sane_value_right.Z = getSaneValue(rightHandPositionBuffer->get(rightHandPositionBuffer->end())->Z, master.rightHandCurrentPosition.Z);
		}
		else {
			sane_value_left = master.leftHandCurrentPosition;
			sane_value_right = master.rightHandCurrentPosition;
		}

		// Neue Werte in die Buffer schreiben
		leftHandPositionBuffer->push(sane_value_left);
		rightHandPositionBuffer->push(sane_value_right);

		// Berechnungsschritt der State-Machine
		stateMachine.bufferGestureConfidence();
		stateMachine.compute();
		stateMachine.switchState();
		
		switch (stateMachine.getState())
		{
		case StateMachine::State::IDLE:
			OutputDebugStringA("IDLE\n");
			break;
		case StateMachine::State::CAMERA_TRANSLATE:
			OutputDebugStringA("CAMERA_TRANSLATE\n");
			break;
		case StateMachine::State::CAMERA_ROTATE:
			OutputDebugStringA("CAMERA_ROTATE\n");
			break;
		case StateMachine::State::OBJECT_MANIPULATE:
			OutputDebugStringA("OBJECT_MANIPULATE\n");
			break;
		default:
			break;
		}
		
		
	}

	// Frame - Speicher freigeben
	bodyFrame->Release();

	return stateMachine.getMotionParameters();
}
