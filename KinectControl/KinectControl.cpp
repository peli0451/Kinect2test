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

KinectControl::KinectControl() { 
	masterDetermined = false;
}



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

/**
* Hole und bearbeite Frame, den Kinect liefert
*
* @return motionParameters Transformationsparameter
*/

bool collectFrames = false;
int collectedFrames = 0;
MotionParameters KinectControl::run() {
	Person master = stateMachine.getMaster();
	MotionParameters motionParameters = stateMachine.getMotionParameters();
	float identificationError, identificationErrorMin = FLT_MAX;;

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
		for (int i = 0; i < numberOfTrackedBodies; i++)
		{
			BOOLEAN isTracked;
			trackedBodies[i]->get_IsTracked(&isTracked); //ist i-te potentielle Person getrackt
			//trackedBodies[i]->get_TrackingId(&trackingId); //Tracking ID der i-ten getrackten Person

			//Tracking erkannter Personen, Identifikation des Masters
			if (isTracked == TRUE) {
				//Wenn getrackt, lege Gelenkmodell an (Gelenkarray)
				Joint joints[JointType_Count];
				result = trackedBodies[i]->GetJoints(JointType_Count, joints);
				if (SUCCEEDED(result)) {
					//Falls Gelenke erfolgreich geholt
					_CameraSpacePoint headPosition = joints[JointType::JointType_Head].Position;

					if (masterDetermined && !collectFrames) {
						master.compareBodyProperties(joints);
						/*
						identificationError = master.compareBodyProperties(joints);
						if (identificationError < identificationErrorMin) {
							master.setId(i);
							identificationError = identificationErrorMin;
							
							OutputDebugStringA("Neue Master-Id:\t");
							OutputDebugStringA(std::to_string(i).c_str());
							OutputDebugStringA("\n");
							
						}
						
						OutputDebugStringA("Abweichung:\t");
						OutputDebugStringA(std::to_string(master.compareBodyProperties(joints)).c_str());
						OutputDebugStringA("\n");
						*/
					}
					else if (masterDetermined && collectFrames){
						if (collectedFrames < 20) {
							master.setJoints(joints); //@TODO Fragwürdige Lösung? Nochmal z-TEst oder so
							master.collectBodyProperties();
							collectedFrames++;
						}
						else {
							master.calculateBodyProperties();
							collectedFrames = 0;
							collectFrames = false;
						}
					}
					else {
						if (headPosition.Z < master.getZ()) {
							master.setJoints(joints);
							master.setId(i);
							master.setZ(headPosition.Z);
						}
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
		CameraSpacePoint leftHandCurPos = joints[JointType::JointType_HandLeft].Position;
		CameraSpacePoint rightHandCurPos = joints[JointType::JointType_HandRight].Position;

		Buffer<_CameraSpacePoint>* leftHandPositionBuffer = master.getLeftHandPosBuffer();
		Buffer<_CameraSpacePoint>* rightHandPositionBuffer = master.getRightHandPosBuffer();

		// Sanity-Check der Positionswerte
		CameraSpacePoint sane_value_left;
		CameraSpacePoint sane_value_right;
		
		if (leftHandPositionBuffer->get(leftHandPositionBuffer->end()) != NULL && rightHandPositionBuffer->get(rightHandPositionBuffer->end()) != NULL) {
			sane_value_left.X = getSaneValue(leftHandPositionBuffer->get(leftHandPositionBuffer->end())->X,leftHandCurPos.X);
			sane_value_left.Y = getSaneValue(leftHandPositionBuffer->get(leftHandPositionBuffer->end())->Y,leftHandCurPos.Y);
			sane_value_left.Z = getSaneValue(leftHandPositionBuffer->get(leftHandPositionBuffer->end())->Z,leftHandCurPos.Z);
			sane_value_right.X = getSaneValue(rightHandPositionBuffer->get(rightHandPositionBuffer->end())->X, rightHandCurPos.X);
			sane_value_right.Y = getSaneValue(rightHandPositionBuffer->get(rightHandPositionBuffer->end())->Y, rightHandCurPos.Y);
			sane_value_right.Z = getSaneValue(rightHandPositionBuffer->get(rightHandPositionBuffer->end())->Z, rightHandCurPos.Z);
		}
		else {
			sane_value_left = leftHandCurPos;
			sane_value_right = rightHandCurPos;
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
		/*
		switch (stateMachine.getState()) {
		case StateMachine::State::IDLE: OutputDebugStringA("IDLE\n"); break;
		case StateMachine::State::CAMERA_TRANSLATE: OutputDebugStringA("CAMERA_TRANSLATE\n"); break;
		case StateMachine::State::CAMERA_ROTATE: OutputDebugStringA("CAMERA_ROTATE\n"); break;
		case StateMachine::State::OBJECT_MANIPULATE: OutputDebugStringA("OBJECT_MANIPULATE\n"); break;
		case StateMachine::State::FLY: OutputDebugStringA("FLY\n"); break;
		default: break; }
		*/

		
	}

	// Frame - Speicher freigeben
	bodyFrame->Release();

	return stateMachine.getMotionParameters();
}

void KinectControl::assignMaster() {
	masterDetermined = true;
	collectFrames = true;
	collectedFrames = 0;
}