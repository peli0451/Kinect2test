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
* Führt einen Berechnungsschritt der StateMachine durch
* Dies sind die für die Parameter benötigen Rechnung und die Zustandswechsel
*/
void KinectControl::stateMachineStep() {
	// Neue Werte in die Buffer schreiben
	leftHandPositionBuffer->push(master.leftHandCurrentPosition);
	rightHandPositionBuffer->push(master.rightHandCurrentPosition);

	CameraSpacePoint result_left;
	CameraSpacePoint result_right;

	float translateX;
	float translateY;
	float translateZ;

	switch (state)
	{
	case KinectControl::CAMERA_IDLE:
		if (recognizedGesture == ROTATE_GESTURE)
			setState(CAMERA_ROTATE);
		else if (recognizedGesture == TRANSLATE_GESTURE)
			setState(CAMERA_TRANSLATE);
		break;

		//Translation der Kamera + Zustandswechsel
	case KinectControl::CAMERA_TRANSLATE:
		result_left = *smooth_speed(leftHandPositionBuffer);
		result_right = *smooth_speed(rightHandPositionBuffer);

		translateX = (result_left.X + result_right.X) / 2;
		translateY = (result_left.Y + result_right.Y) / 2;
		translateZ = (result_left.Z + result_right.Z) / 2;
		/*
		TBD: entfernen, wenn durch Glättung nicht mehr nötig
		float threshold = .002f;

		if (std::abs(translateX) < threshold) translateX = .0f;
		if (std::abs(translateY) < threshold) translateY = .0f;
		if (std::abs(translateZ) < threshold) translateZ = .0f;
		*/

		motionParameters.translateX = translateX;
		motionParameters.translateY = translateY;
		motionParameters.translateZ = translateZ;

		//On-Screen-Debug-Gehacke
		//OutputDebugStringA("Verschiebe Kamera\n");
		//std::cout << "Verschiebe Kamera ( " << (int)(motionParameters->translateX * 100) << " "
		//	<< (int)(motionParameters->translateY * 100) << " " << (int)(motionParameters->translateZ * 100) << " )";

		if (recognizedGesture == ROTATE_GESTURE) {
			motionParameters.translateX = .0f;
			motionParameters.translateY = .0f;
			motionParameters.translateZ = .0f;
			setState(CAMERA_ROTATE);
		}
		else if (recognizedGesture == TRANSLATE_GESTURE) {
			setState(CAMERA_TRANSLATE);
		}
		else {
			motionParameters.translateX = .0f;
			motionParameters.translateY = .0f;
			motionParameters.translateZ = .0f;
			motionParameters.rotateX = .0f;
			motionParameters.rotateY = .0f;
			motionParameters.rotateZ = .0f;
			setState(CAMERA_IDLE);
		}
		break;

		//Rotation der Kamera + Zustandswechsel
	case KinectControl::CAMERA_ROTATE:
		//TBD: Kamerarotation

		//OutputDebugStringA("Rotiere Kamera\n");
		//std::cout << "Rotiere Kamera";

		//Zustandswechsel
		if (recognizedGesture == ROTATE_GESTURE)
			setState(CAMERA_ROTATE);
		else if (recognizedGesture == TRANSLATE_GESTURE) {
			motionParameters.rotateX = .0f;
			motionParameters.rotateY = .0f;
			motionParameters.rotateZ = .0f;
			setState(CAMERA_TRANSLATE);
		} else {
			motionParameters.translateX = .0f;
			motionParameters.translateY = .0f;
			motionParameters.translateZ = .0f;
			motionParameters.rotateX = .0f;
			motionParameters.rotateY = .0f;
			motionParameters.rotateZ = .0f;
			setState(CAMERA_IDLE);
		}
		break;

	case KinectControl::OBJECT_IDLE:		// TDB
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
	motionParameters.translateX = .0f;
	motionParameters.translateY = .0f;
	motionParameters.translateZ = .0f;
	motionParameters.rotateX = .0f;
	motionParameters.rotateY = .0f;
	motionParameters.rotateZ = .0f;
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
		recognizedGesture = evaluateGestureBuffer();

		// Berechnungsschritt der State-Machine
		stateMachineStep();
		
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
