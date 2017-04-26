/**
* KinectControl.cpp
* Managerklasse, Schnittstelle zur Kinect
*/

#include "stdafx.h"
#include <string>
#include <iostream>
#include "KinectControl.h"

/* 
* Globale Konstanten
*/

//TODO in den Header
const float MAX_SANE_DISTANCE = 0.1f; // Wert geraten. Könnte man auch getSaneValue übergeben, wenn es variieren soll
const float MAX_STEP = 0.05f; // muss <= MAX_SANE_DISTANCe sein



/**********************************************************
* Konstruktoren
**********************************************************/

KinectControl::KinectControl() { }



/**********************************************************
* Funktionen
**********************************************************/

/**
* Initialisiert eine KinectControl-Instanz
* Bereitet Kinect-Sensor und -Frame-Reader vor
* Bereitet Puffer vor
*/
void KinectControl::init(ControlWidget *_widget) {
	stateMachine.assignWidget(_widget);
	GetDefaultKinectSensor(&kinectSensor);
	kinectSensor->Open();
	kinectSensor->get_BodyFrameSource(&bodyFrameSource);
	bodyFrameSource->get_BodyCount(&numberOfTrackedBodies); //Anzahl Personen?
	bodyFrameSource->OpenReader(&bodyFrameReader);
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

int KinectControl::compareToMasterProperties(Person::BodyProperties* propertiesForComparison) {
	int matchedProperties = 0;
	Person::BodyProperties masterProperties = stateMachine.getMaster().getBodyProperties();

	//TODO Definitionen erstmal nur, damit kompilierbar, später ersetzen und woanders platzieren
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

	if (abs(propertiesForComparison->neckToLeftShoulder - masterProperties.neckToLeftShoulder) < ERROR_NTLS)
		matchedProperties++;
	if (abs(propertiesForComparison->neckToRightShoulder - masterProperties.neckToRightShoulder) < ERROR_NTRS)
		matchedProperties++;
	if (abs(propertiesForComparison->leftUpperArmLength - masterProperties.leftUpperArmLength) < ERROR_LUAL)
		matchedProperties++;
	if (abs(propertiesForComparison->rightUpperArmLength - masterProperties.rightUpperArmLength) < ERROR_RUAL)
		matchedProperties++;
	if (abs(propertiesForComparison->leftUpperLegLength - masterProperties.leftUpperLegLength) < ERROR_RULL)
		matchedProperties++;
	if (abs(propertiesForComparison->shoulderWidth - masterProperties.shoulderWidth) < ERROR_SW)
		matchedProperties++;
	if (abs(propertiesForComparison->torsoLength - masterProperties.torsoLength) < ERROR_TL)
		matchedProperties++;
	if (abs(propertiesForComparison->ratioBetweenTorsoLengthAndLeftLeg -
		masterProperties.ratioBetweenTorsoLengthAndLeftLeg) < ERROR_RBTLALL)
		matchedProperties++;
	if (abs(propertiesForComparison->ratioBetweenTorsoLengthAndRightLeg -
		masterProperties.ratioBetweenTorsoLengthAndRightLeg) < ERROR_RBTLARL)
		matchedProperties++;
	if (abs(propertiesForComparison->ratioBetweenTorsoLengthAndShoulderWidth -
		masterProperties.ratioBetweenTorsoLengthAndShoulderWidth) < ERROR_RBTLASW)
		matchedProperties++;

	return matchedProperties;
}

/**
* Hole und bearbeite Frame, den Kinect liefert
*
* @return motionParameters Transformationsparameter
*/
MotionParameters KinectControl::run() {
	Person master = stateMachine.getMaster();
	MotionParameters motionParameters = stateMachine.getMotionParameters();
	//[deprecated]
	//Plan: Iterieren über Köpfe, den niedrigsten z-Wert als Master wählen, Mastervariable
	master.setId(-1);
	master.setZ(FLT_MAX);

	IBodyFrame *bodyFrame;

	//Hole aktuellen Kinect-Frame (falls möglich)
	//Bei Fehlschlag arbeite mit alten Parametern
	result = bodyFrameReader->AcquireLatestFrame(&bodyFrame);
	if (result != S_OK) {
		return motionParameters;
	}

	//Hole aktuelle Kinect-Body-Daten (falls möglich)
	//Bei Fehlschlag verwerfe Frame, arbeite mit alten Daten
	result = bodyFrame->GetAndRefreshBodyData(numberOfTrackedBodies, trackedBodies); //Update für bodies-Array
	if (result != S_OK) {
		bodyFrame->Release();
		return motionParameters;
	}

	//TODO hier noch alte, primitive Mastererkennung
	for (int i = 0; i < numberOfTrackedBodies; ++i)
	{
		BOOLEAN isTracked;
		trackedBodies[i]->get_IsTracked(&isTracked); //ist i-te potentielle Person getrackt

		//Tracking erkannter Personen, Identifikation des Masters
		if (isTracked == TRUE) {
			//Wenn getrackt, lege Gelenkmodell an (Gelenkarray)
			Joint joints[JointType_Count];
			result = trackedBodies[i]->GetJoints(JointType_Count, joints);
			if (SUCCEEDED(result)) {
				//Falls Gelenke erfolgreich geholt
				master.setJoints(joints);
				_CameraSpacePoint headPosition = (stateMachine.getMaster().getJoints())[JointType::JointType_Head].Position;
				if (headPosition.Z < stateMachine.getMaster().getZ()) {
					master.setId(i);
					master.setZ(headPosition.Z);
				}

			}
		}
	}

	stateMachine.setMaster(master);

	//@TODO Wenn Master wechselt muss der Ringpuffer für die Positionen neu initialisiert werden (mit der Position des neuen Masters)
	//@TODO Mastererkennung mit Confidence, nicht direkt
	if (master.getId() != -1) {


		//Hole Gelenkobjekte und wichtige Positionen des Masters
		Joint joints[JointType_Count];
		trackedBodies[master.getId()]->GetJoints(JointType_Count, joints);

		JointOrientation jointOrientations[JointType_Count];
		trackedBodies[master.getId()]->GetJointOrientations(JointType_Count, jointOrientations);

		HandState leftHandState;
		trackedBodies[master.getId()]->get_HandLeftState(&leftHandState);

		HandState rightHandState;
		trackedBodies[master.getId()]->get_HandRightState(&rightHandState);

		master.setJoints(joints);
		master.setJointOrientations(jointOrientations);
		master.setLeftHandState(leftHandState);
		master.setRightHandState(rightHandState);
		master.setLeftHandCurPos(joints[JointType::JointType_HandLeft].Position);
		master.setRightHandCurPos(joints[JointType::JointType_HandRight].Position);

		Buffer<_CameraSpacePoint>* leftHandPositionBuffer = master.getLeftHandPosBuffer();
		Buffer<_CameraSpacePoint>* rightHandPositionBuffer = master.getRightHandPosBuffer();

		// Sanity-Check der Positionswerte
		CameraSpacePoint sane_value_left;
		CameraSpacePoint sane_value_right;
		
		if (leftHandPositionBuffer->get(leftHandPositionBuffer->end()) != NULL && rightHandPositionBuffer->get(rightHandPositionBuffer->end()) != NULL) {
			sane_value_left.X = getSaneValue(leftHandPositionBuffer->get(leftHandPositionBuffer->end())->X, master.getLeftHandCurPos().X);
			sane_value_left.Y = getSaneValue(leftHandPositionBuffer->get(leftHandPositionBuffer->end())->Y, master.getLeftHandCurPos().Y);
			sane_value_left.Z = getSaneValue(leftHandPositionBuffer->get(leftHandPositionBuffer->end())->Z, master.getLeftHandCurPos().Z);
			sane_value_right.X = getSaneValue(rightHandPositionBuffer->get(rightHandPositionBuffer->end())->X, master.getRightHandCurPos().X);
			sane_value_right.Y = getSaneValue(rightHandPositionBuffer->get(rightHandPositionBuffer->end())->Y, master.getRightHandCurPos().Y);
			sane_value_right.Z = getSaneValue(rightHandPositionBuffer->get(rightHandPositionBuffer->end())->Z, master.getRightHandCurPos().Z);
		}
		else {
			sane_value_left = master.getLeftHandCurPos();
			sane_value_right = master.getRightHandCurPos();
		}

		// Neue Werte in die Buffer schreiben
		leftHandPositionBuffer->push(sane_value_left);
		rightHandPositionBuffer->push(sane_value_right);

		stateMachine.setMaster(master);

		// Berechnungsschritt der State-Machine
		stateMachine.bufferGestureConfidence();
		stateMachine.compute();
		stateMachine.switchState();
		

		
		//Debug: Ausgabe des Zustands der State-Machine auf der Konsole
		
		switch (stateMachine.getState()) {
		case StateMachine::State::IDLE: OutputDebugStringA("IDLE\n"); break;
		case StateMachine::State::CAMERA_TRANSLATE: OutputDebugStringA("CAMERA_TRANSLATE\n"); break;
		case StateMachine::State::CAMERA_ROTATE: OutputDebugStringA("CAMERA_ROTATE\n"); break;
		case StateMachine::State::OBJECT_MANIPULATE: OutputDebugStringA("OBJECT_MANIPULATE\n"); break;
		default: break; }
		

		
	}

	// Frame - Speicher freigeben
	bodyFrame->Release();

	/*
	OutputDebugStringA("KINECT CONTROL MP:\t");
	OutputDebugStringA(std::to_string(stateMachine.getMotionParameters().getTranslateX()).c_str());
	OutputDebugStringA("\n\n");
	*/

	return stateMachine.getMotionParameters();
}
