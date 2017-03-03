/**
* KinectControl.cpp
* Managerklasse, Schnittstelle zur Kinect
*/

#include "stdafx.h"
#include <string>
#include <iostream>

#include "KinectControl.h"


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
		100.0					// z
	};

	// State Machine
	setState(CAMERA_IDLE);		// Initialzustand

	// Gestenbuffer
	recognizedGesture = UNKNOWN;
	recognizedGesturesBuffer = new Buffer<Gesture>(GESTURE_BUFFER_SIZE);

	// Handpositionenbuffer
	leftHandPositionBuffer = new Buffer<CameraSpacePoint>(POS_BUFFER_SIZE);
	rightHandPositionBuffer = new Buffer<CameraSpacePoint>(POS_BUFFER_SIZE);
}





/**
* Wechselt den aktuellen Zustand
*
* @param newState neuer Zustand
*/
void KinectControl::setState(KinectControlState newState) {
	state = newState;
}

/**
* Liest den aktuellen Zustand
*
* @return state aktueller Zustand
*/
KinectControl::KinectControlState KinectControl::getState() {
	return state;
}

/**
* Liest die aktuellen motionParameters
*
* @return motionParameters aktuelle (zuvor berechnete) Manipulationsparameter
*/
KinectControl::MotionParameters KinectControl::getMotion() {
	return motionParameters;
}

/**
* Setzt neue motionParameters
*
* @param translateX neuer Translationswert in x-Richtung
* @param translateY neuer Translationswert in y-Richtung
* @param translateZ neuer Translationswert in z-Richtung
* @param rotateX neuer Rotationswinkel um x-Achse
* @param rotateY neuer Rotationswinkel um y-Achse
* @param rotateZ neuer Rotationswinkel um z-Achse
*/
void KinectControl::setMotion(float translateX, float translateY, float translateZ, float rotateX, float rotateY, float rotateZ) {
	setTranslation(translateX, translateY, translateZ);
	setRotation(rotateX, rotateY, rotateZ);
}

/**
* Setzt neue motionParameters, nur Translation
*
* @param translateX neuer Translationswert in x-Richtung
* @param translateY neuer Translationswert in y-Richtung
* @param translateZ neuer Translationswert in z-Richtung
*/
void KinectControl::setTranslation(float translateX, float translateY, float translateZ) {
	motionParameters.translateX = translateX;
	motionParameters.translateY = translateY;
	motionParameters.translateZ = translateZ;
}

/**
* Setzt neue motionParameters, nur Rotation
*
* @param rotateX neuer Rotationswinkel um x-Achse
* @param rotateY neuer Rotationswinkel um y-Achse
* @param rotateZ neuer Rotationswinkel um z-Achse
*/
void KinectControl::setRotation(float rotateX, float rotateY, float rotateZ) {

	motionParameters.rotateX = rotateX;
	motionParameters.rotateY = rotateY;
	motionParameters.rotateZ = rotateZ;
}

/**
* Nullt die motionParameters
*/
void KinectControl::resetMotion() {
	setMotion(.0f, .0f, .0f, .0f, .0f, .0f);
}

/**
* Nullt die motionParameters, nur Translation
*/
void KinectControl::resetTranslation() {
	setTranslation(.0f, .0f, .0f);
}

/**
* Nullt die motionParameters, nur Rotation
*/
void KinectControl::resetRotation() {
	setRotation(.0f, .0f, .0f);
}

/**
* Liest die erkannte Geste
* @return recognizedGesture erkannte Geste
*/
KinectControl::Gesture KinectControl::getRecognizedGesture() {
	return recognizedGesture;
}

/**
* Setzt die erkannte Geste
*/
void KinectControl::setRecognizedGesture(Gesture gesture) {
	recognizedGesture = gesture;
}

/**
* Realisiert die Berechnungen der MotionParameters in den Zuständen der State Machine.
*/
void KinectControl::stateMachineCompute() {
	KinectControlState currentState = getState();
	Gesture recognizedGesture = getRecognizedGesture();

	switch (currentState) {
	case KinectControl::CAMERA_IDLE:
		//TODO
		break;
	case KinectControl::CAMERA_TRANSLATE:
		switch (recognizedGesture) {
		case ROTATE_GESTURE:
			resetTranslation();
			break;
		case TRANSLATE_GESTURE: {
			CameraSpacePoint smoothenedLeftHandPosition = *smooth_speed(leftHandPositionBuffer);
			CameraSpacePoint smoothenedRightHandPosition = *smooth_speed(rightHandPositionBuffer);
			float translateX = (smoothenedLeftHandPosition.X + smoothenedRightHandPosition.X) / 2;
			float translateY = (smoothenedLeftHandPosition.Y + smoothenedRightHandPosition.Y) / 2;
			float translateZ = (smoothenedLeftHandPosition.Z + smoothenedRightHandPosition.Z) / 2;
			setTranslation(translateX, translateX, translateZ);
			break;
			}
		default:
			resetMotion();
			break;
		}
		break;
	case KinectControl::CAMERA_ROTATE:
		switch (recognizedGesture) {
		case ROTATE_GESTURE: 
			//TODO
			break;
		case TRANSLATE_GESTURE:
			resetRotation();
			break;
		default:
			resetMotion();
			break;
		}
		break;

	case KinectControl::OBJECT_IDLE:
		break;
	case KinectControl::OBJECT_TRANSLATE:
		break;
	case KinectControl::OBJECT_ROTATE:
		break;
	default:
		break;
	}
}

/**
* Realisiert die Zustandsübergänge der StateMachine
*/
void KinectControl::stateMachineSwitchState() {
	KinectControlState currentState = getState();
	Gesture recognizedGesture = getRecognizedGesture();

	switch (currentState) {
	case CAMERA_IDLE:		//Fallthrough
	case CAMERA_TRANSLATE:	//Fallthrough
	case CAMERA_ROTATE:
		switch (recognizedGesture) {
		case ROTATE_GESTURE:
			setState(CAMERA_ROTATE);
			break;
		case TRANSLATE_GESTURE:
			setState(CAMERA_TRANSLATE);
			break;
		case GRAB_GESTURE:
			//setState(OBJECT_IDLE);
			break;
		default:
			//Zustand nicht wechseln
			break;
		}
		break;

	case OBJECT_IDLE:		//Fallthrough
	case OBJECT_TRANSLATE:  //Fallthrough
	case OBJECT_ROTATE:
		break;

	default:
		break;
	}
}

/**
* Initialisiert eine KinectControl-Instanz
* Bereitet Kinect-Sensor und -Frame-Reader vor
* Bereitet Puffer vor
*/
void KinectControl::init() {
	GetDefaultKinectSensor(&kinectSensor);
	kinectSensor->Open();
	kinectSensor->get_BodyFrameSource(&bodyFrameSource);
	bodyFrameSource->get_BodyCount(&numberOfTrackedBodies); //Anzahl Personen?
	bodyFrameSource->OpenReader(&bodyFrameReader);
	smoothing_sum = 0;
	for (int i = 0; i < POS_BUFFER_SIZE; i++) {
		smoothing_sum += smoothing_factor[i];
	}
	//Initialisierung der motionParameters
	resetMotion();
}

/**
* Glättet gepufferte Parameter
*
* @TODO Doku ergänzen
*/
CameraSpacePoint* KinectControl::smooth_speed(Buffer<CameraSpacePoint>* buffer) {
	CameraSpacePoint speed_point = { 0,0,0 };
	// läuft atm vom ältesten zum jüngsten, deshalb schleife absteigend
	for (int i = buffer->end(); i >= 1; i--) {
		// von jung zu alt, stoppt eins später für ableitung
		// ich achte mal noch nich drauf was bei nicht vollem puffer genau passiert
		CameraSpacePoint *cur_point = buffer->get(i);
		CameraSpacePoint *next_point = buffer->get(i - 1);
		float smoothing = smoothing_factor[i-1] / smoothing_sum;

		/*
		OutputDebugStringA(std::to_string(smoothing).c_str());
		OutputDebugStringA("\t");
		OutputDebugStringA(std::to_string(smoothing_factor[i-1]).c_str());
		OutputDebugStringA("\t");
		OutputDebugStringA(std::to_string(smoothing_sum).c_str());
		OutputDebugStringA("\n");
		*/

		speed_point.X += (cur_point->X - next_point->X) * smoothing;
		speed_point.Y += (cur_point->Y - next_point->Y) * smoothing;
		speed_point.Z += (cur_point->Z - next_point->Z) * smoothing;
	}
	return &speed_point;
}

/**
* Wertet den Buffer mit den erkannten Gesten aus
* Führt dabei einen gewichteten Mehrheitsentscheid durch
*
* @TODO eventuell ist noch mit Puffergröße und Gewichten zu spielen
*
* @return maxConfidenceGesture Geste, die basierend auf den gepufferten Gesten wahrscheinlich vorgeführt wurde
*/
KinectControl::Gesture KinectControl::evaluateGestureBuffer() {
	int gestureCounter[4] = { 0 }; //4 Gesten
	float gestureWeights[4] = { .1f,.3f,.3f,.3f };
	for (int i = 0; i < recognizedGesturesBuffer->end(); i++)
		gestureCounter[*recognizedGesturesBuffer->get(i)]++;
	float weightedGestures[4] = { 0 };
	Gesture maxConfidenceGesture = UNKNOWN;
	for (int i = 0; i < 4; i++) {
		weightedGestures[i] = gestureCounter[i] * gestureWeights[i];
		if (weightedGestures[i] > weightedGestures[maxConfidenceGesture])
			maxConfidenceGesture = static_cast<Gesture>(i);
	}
	return maxConfidenceGesture;
}

/**
* Hole und bearbeite Frame, den Kinect liefert
*
* @return motionParameters Transformationsparameter
*/
KinectControl::MotionParameters KinectControl::run() {

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

		// Erkennen der Grundtypen von Gesten
		if (master.leftHandState == HandState_Closed && master.rightHandState == HandState_Closed)
			recognizedGesturesBuffer->push(ROTATE_GESTURE);
		else if (master.leftHandState == HandState_Open && master.rightHandState == HandState_Open)
			recognizedGesturesBuffer->push(TRANSLATE_GESTURE);
		else
			recognizedGesturesBuffer->push(UNKNOWN);
		setRecognizedGesture(evaluateGestureBuffer());

		// Neue Werte in die Buffer schreiben
		leftHandPositionBuffer->push(master.leftHandCurrentPosition);
		rightHandPositionBuffer->push(master.rightHandCurrentPosition);

		// Berechnungsschritt der State-Machine
		stateMachineCompute();
		stateMachineSwitchState();
		
		switch (state)
		{
		case KinectControl::CAMERA_IDLE:
			OutputDebugStringA("CAMERA_IDLE\t");
			break;
		case KinectControl::CAMERA_TRANSLATE:
			OutputDebugStringA("CAMERA_TRANSLATE\t");
			break;
		case KinectControl::CAMERA_ROTATE:
			OutputDebugStringA("CAMERA_ROTATE\t");
			break;
		case KinectControl::OBJECT_IDLE:
			OutputDebugStringA("OBJECT_IDLE\t");
			break;
		case KinectControl::OBJECT_TRANSLATE:
			OutputDebugStringA("OBJECT_TRANSLATE\t");
			break;
		case KinectControl::OBJECT_ROTATE:
			OutputDebugStringA("OBJECT_ROTATE\t");
			break;
		default:
			break;
		}
		
		
	}

	// Frame - Speicher freigeben
	bodyFrame->Release();

	return motionParameters;
}
