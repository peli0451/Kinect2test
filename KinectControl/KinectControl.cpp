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
		100.0					// z
	};

	// State Machine
	setState(CAMERA_IDLE);		// Initialzustand

	// Gestenbuffer
	recognizedGesture = UNKNOWN;
	gestureConfidenceBuffer = new Buffer<GestureConfidence>(GESTURE_BUFFER_SIZE);

	// Handpositionenbuffer
	leftHandPositionBuffer = new Buffer<CameraSpacePoint>(POS_BUFFER_SIZE);
	rightHandPositionBuffer = new Buffer<CameraSpacePoint>(POS_BUFFER_SIZE);

	// Rotationenbuffer
	rotationBuffer = new Buffer<Eigen::Quaternionf>(ROT_BUFFER_SIZE);
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
* @param target Objekt oder Kamera bewegen?
*/
void KinectControl::setMotion(float translateX, float translateY, float translateZ, Eigen::Quaternionf rotate, MotionTarget target) {
	setTranslation(translateX, translateY, translateZ);
	setRotation(rotate);
	setTarget(target);
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
void KinectControl::setRotation(Eigen::Quaternionf rotate) {
	motionParameters.rotate = rotate;
}

/**
* Setzt neue motionParameters, nur Objekt/Kamera-Modus
*
* @param target neuer Modus (Objekt oder Kamera)
*/
void KinectControl::setTarget(MotionTarget target) {
	motionParameters.target = target;
}

/**
* Nullt die motionParameters
*/
void KinectControl::resetMotion() {
	setMotion(.0f, .0f, .0f, Eigen::Quaternionf::Identity(), TARGET_CAMERA);
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
	setRotation(Eigen::Quaternionf::Identity());
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
* Bestimmt eine Matrix, die den ersten Einheitsvektor auf den Argumentvektor dreht
* @param Vektor, auf den gedreht werden soll
* @return Rotationsmatrix
*
* Deprecated
*/
/*
Eigen::Matrix3f getRotationMatrix(Eigen::Vector3f target) {
	Eigen::Matrix3f ret;
	ret.row(0) = target; // Erste Spalte der Rotationsmatrix ist der Zielvektor selbst
	ret.row(1) = target.cross(Eigen::Vector3f::UnitX()); // Zweite Spalte ist das Kreuzprodukt aus Zielvektor und (1,0,0)
	ret.row(1).normalize(); // geteilt durch die Norm davon
	ret.row(2) = ret.row(0).cross(ret.row(1)); // Dritte Spalte ist das Kreuzprodukt der ersten und zweiten Spalte
	return ret; // diese Konstruktion stellt sicher, dass die Matrix orthogonal ist
}
*/

/**
* Berechnet aus der Trackingrückgabe Konfidenzwerte für die verschiedenen Gesten
*/
void KinectControl::stateMachineBufferGestureConfidence() {
	//aktueller Zustand
	KinectControlState currentState = getState();
	
	//Ermittelter Konfidenzwert
	GestureConfidence newConfidence = { 0,0,0,0 };
	//Erinnerung Konfidenzschema: { unknown, translate, rotate, grab }


	//Hilfsvariablen für Kombinationen von von Handstates
	boolean leftHandOpen = (master.leftHandState == HandState_Open);
	boolean leftHandClosed = (master.leftHandState == HandState_Closed);
	boolean rightHandOpen = (master.rightHandState == HandState_Open);
	boolean rightHandClosed = (master.rightHandState == HandState_Closed);

	boolean bothHandsOpen = (leftHandOpen && rightHandOpen);
	boolean bothHandsClosed = (leftHandClosed && rightHandClosed);

	boolean handRisen;
	boolean risenHandOpen;
	boolean risenHandUnknown;
	
	
	if (master.leftHandCurrentPosition.Y - master.rightHandCurrentPosition.Y > .4) {
		risenHand = HAND_LEFT;
		handRisen = true;
		risenHandOpen = (master.leftHandState == HandState_Open);
		risenHandUnknown = (master.leftHandState == HandState_Unknown);
	} else if (master.rightHandCurrentPosition.Y - master.leftHandCurrentPosition.Y > .4) {
		risenHand = HAND_RIGHT;
		handRisen = true;
		risenHandOpen = (master.rightHandState == HandState_Open);
		risenHandUnknown = (master.rightHandState == HandState_Unknown);
	} else {
		handRisen = false;
		risenHandUnknown = false;
		risenHandOpen = false;
	}


	//Je nach Zustand sind die Bewertungen der Trackingergebnisse anders zu bewerten
	switch (currentState) {

	//Zustand IDLE -> alle Gesten werden nur eindeutig betrachtet
	case KinectControl::CAMERA_IDLE:
		if (risenHandOpen) newConfidence = { .0f,.0f,.0f,1.f }; //Grabgeste
		else if (bothHandsClosed) newConfidence = { .0f,.0f,1.f,.0f }; //beide geschlossen
		else if (bothHandsOpen) newConfidence = { .0f,1.f,.0f,.0f }; //beide offen
		else newConfidence = { 1.f,.0f,.0f,.0f }; //Unbekannt
		break;

	//Zustand TRANSLATE -> neben eindeutigen Gesten sind doppeldeutige mit einer geöffneten Hand vermutlich ein Translate
	case KinectControl::CAMERA_TRANSLATE :
		if (risenHandOpen) newConfidence = { .0f,.0f,.0f,1.f }; //Grabgeste
		else if (bothHandsClosed) newConfidence = { .0f,.0f,1.f,.0f }; //beide geschlossen
		else if (bothHandsOpen) newConfidence = { .0f,1.f,.0f,.0f }; //beide offen
		else if (leftHandOpen && !rightHandOpen && !rightHandClosed || rightHandOpen && !leftHandOpen && !leftHandClosed)
			newConfidence = { .0f,1.f,.0f,.0f }; //sehr wahrscheinlich Translate
		else if (leftHandOpen && rightHandClosed || leftHandClosed && rightHandOpen)
			newConfidence = { .0f,.75f,.25f,.0f }; //doppeldeutig, halte Translate für wahrscheinlicher
		else newConfidence = { .7f,.1f,.1f,.1f }; //Unbekannt
		break;

	//Zustand ROTATE -> neben eindeutigen Gesten sind doppeldeutige mit einer geschlossenen Hand vermutlich ein Rotate
	case KinectControl::CAMERA_ROTATE :
		if (risenHandOpen) newConfidence = { .0f,.0f,.0f,1.f }; //Grabgeste
		else if (bothHandsClosed) newConfidence = { .0f,.0f,1.f,.0f }; //beide geschlossen
		else if (bothHandsOpen) newConfidence = { .0f,1.f,.0f,.0f }; //beide offen
		else if (leftHandClosed && !rightHandClosed && !rightHandOpen || rightHandClosed && !leftHandClosed && !leftHandOpen)
			newConfidence = { .0f,.0f,1.f,.0f }; //sehr wahrscheinlich Rotate
		else if (leftHandOpen && rightHandClosed || leftHandClosed && rightHandOpen)
			newConfidence = { .0f,.25f,.75f,.0f }; //doppeldeutig, halte Rotate für wahrscheinlicher
		else newConfidence = { .7f,.1f,.1f,.1f }; //Unbekannt
		break;

	//Zustand MANIPULATE -> alle Gesten werden nur eindeutig betrachtet
	case KinectControl::OBJECT_MANIPULATE :
		if (risenHandOpen) newConfidence = { .0f,.0f,.0f,1.f }; //Grabgeste
		else if (risenHandUnknown) newConfidence = { .4f, .0f, .0f, .6f };
		else if (bothHandsClosed) newConfidence = { .0f,.0f,1.f,.0f }; //beide geschlossen
		else if (bothHandsOpen) newConfidence = { .0f,1.f,.0f,.0f }; //beide offen
		else newConfidence = { .7f,.1f,.1f,.1f }; //Unbekannt
		break;
	
	default: break;
	}
	
	OutputDebugStringA(std::to_string(newConfidence.unknownConfidence).c_str());
	OutputDebugStringA("\t");
	OutputDebugStringA(std::to_string(newConfidence.translateCameraConfidence).c_str());
	OutputDebugStringA("\t");
	OutputDebugStringA(std::to_string(newConfidence.rotateCameraConfidence).c_str());
	OutputDebugStringA("\t");
	OutputDebugStringA(std::to_string(newConfidence.grabConfidence).c_str());
	OutputDebugStringA("\t");

	//Berechnete Konfidenzien in den Pufferschreiben
	gestureConfidenceBuffer->push(newConfidence);

	//Puffer auswerten
	setRecognizedGesture(evaluateGestureBuffer());
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
			CameraSpacePoint smoothenedLeftHandPosition = *smoothSpeed(leftHandPositionBuffer);
			CameraSpacePoint smoothenedRightHandPosition = *smoothSpeed(rightHandPositionBuffer);
			float translateX = (smoothenedLeftHandPosition.X + smoothenedRightHandPosition.X) / 2;
			float translateY = (smoothenedLeftHandPosition.Y + smoothenedRightHandPosition.Y) / 2;
			float translateZ = (smoothenedLeftHandPosition.Z + smoothenedRightHandPosition.Z) / 2;
			setTranslation(translateX, translateY, translateZ);
			break;
			}
		default:
			resetMotion();
			break;
		}
		break;
	case KinectControl::CAMERA_ROTATE:
		switch (recognizedGesture) {
		case ROTATE_GESTURE: {

			Eigen::Vector3f origin_axis;
			Eigen::Vector3f target_axis; //TODO: Smoothing
			origin_axis(0) = leftHandPositionBuffer->get(leftHandPositionBuffer->end() - 1)->X - rightHandPositionBuffer->get(rightHandPositionBuffer->end() - 1)->X;
			origin_axis(1) = leftHandPositionBuffer->get(leftHandPositionBuffer->end() - 1)->Y - rightHandPositionBuffer->get(rightHandPositionBuffer->end() - 1)->Y;
			origin_axis(2) = leftHandPositionBuffer->get(leftHandPositionBuffer->end() - 1)->Z - rightHandPositionBuffer->get(rightHandPositionBuffer->end() - 1)->Z;
			origin_axis.normalize();
			// origin_axis enthält nun den normierten Vektor zwischen der rechten und linken Hand im letzten Frame

			target_axis(0) = leftHandPositionBuffer->get(leftHandPositionBuffer->end())->X - rightHandPositionBuffer->get(rightHandPositionBuffer->end())->X;
			target_axis(1) = leftHandPositionBuffer->get(leftHandPositionBuffer->end())->Y - rightHandPositionBuffer->get(rightHandPositionBuffer->end())->Y;
			target_axis(2) = leftHandPositionBuffer->get(leftHandPositionBuffer->end())->Z - rightHandPositionBuffer->get(rightHandPositionBuffer->end())->Z;
			target_axis.normalize();
			// target_axis enthält nun den normierten Vektor zwischen der rechten und linken Hand im aktuellen Frame

			Eigen::Vector3f rotation_axis;
			rotation_axis = origin_axis.cross(target_axis); // Rotationsachse ist der zur von origin_ und target_axis aufgespannten Ebene orthogonale Vektor
			rotation_axis.normalize(); // nicht sicher, ob das notwendig ist, weil die Ursprungsvektoren ja normiert waren
			Eigen::AngleAxisf rot(std::acos(origin_axis.dot(target_axis)), rotation_axis); // Rotation wird beschrieben durch Winkel und Rotationsachse
			rotationBuffer->push(Eigen::Quaternionf(rot));
			setRotation(smoothRotation(rotationBuffer));

			/* // ursprünglicher Ansatz
			Eigen::Matrix3f rot1 = getRotationMatrix(origin_axis).inverse(); // rot1 ist die Rotationsmatrix, die origin_axis auf den 1. Einheitsvektor dreht 
			Eigen::Matrix3f rot2 = getRotationMatrix(target_axis); // rot2 ist die Rotationsmatrix, die den 1. Einheitsvektor auf target_axis dreht 
			Eigen::Matrix3f rot3 = rot2 * rot1.inverse(); // rot3 ist die Gesamtrotation (wenn es genau falsch herum rotiert, rot3 invertieren ;) )
			// Quelle für den Lösungsansatz: http://matheplanet.com/default3.html?call=viewtopic.php?topic=64323 Antwort 1
			setRotation(Eigen::Quaternionf(rot3));
			*/
			break;
		}
		case TRANSLATE_GESTURE:
			resetRotation();
			break;
		default:
			resetMotion();
			break;
		}
		break;

	case KinectControl::OBJECT_MANIPULATE:
	{
		// Bewegung
		CameraSpacePoint smoothenedHandPosition;
		if (controlHand == HAND_LEFT) { // welche Hand wird zum Bewegen verwendet?
			smoothenedHandPosition = *smoothSpeed(leftHandPositionBuffer);
		}
		else {
			smoothenedHandPosition = *smoothSpeed(rightHandPositionBuffer);
		}
		setTranslation(smoothenedHandPosition.X, smoothenedHandPosition.Y, smoothenedHandPosition.Z);

		// Rotation
		JointOrientation joint_orient[JointType::JointType_Count];
		trackedBodies[master.id]->GetJointOrientations(JointType::JointType_Count, joint_orient); //Ausrichtung der Gelenke holen
		Eigen::Quaternionf currentHandOrientation = Eigen::Quaternionf::Identity();
		Vector4 handOrientation;
		if (controlHand == HAND_LEFT) { // welche Hand wird zum Rotieren verwendet?
			handOrientation = joint_orient[JointType::JointType_HandLeft].Orientation; // Ausrichtung der linken Hand
		}
		else {
			handOrientation = joint_orient[JointType::JointType_HandRight].Orientation; // Ausrichtung der rechten Hand
		}
		// übertrage Werte von Kinect Vector4 in Eigen Quaternion
		currentHandOrientation = Eigen::Quaternionf(handOrientation.w, handOrientation.x, handOrientation.y, handOrientation.z);
		if (lastHandOrientationInitialized) { // nur rotieren, wenn lastHandOrientation initialisiert ist
			rotationBuffer->push(currentHandOrientation * lastHandOrientation.inverse()); // Quaternion-Mult. ist Rotation von last auf current
			setRotation(smoothRotation(rotationBuffer)); 
		}
		Eigen::AngleAxisf aa = Eigen::AngleAxisf(currentHandOrientation);
		OutputDebugStringA(std::to_string(aa.axis().x()).c_str()); OutputDebugStringA("\t");
		OutputDebugStringA(std::to_string(aa.axis().y()).c_str()); OutputDebugStringA("\t");
		OutputDebugStringA(std::to_string(aa.axis().z()).c_str()); OutputDebugStringA("\t");
		OutputDebugStringA(std::to_string(aa.angle()).c_str());
		OutputDebugStringA("\n");
		lastHandOrientation = currentHandOrientation;
		lastHandOrientationInitialized = true;
		break;
	}
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
	case OBJECT_MANIPULATE : // Fallthrough (TODO Fix)
	case CAMERA_ROTATE:
		switch (recognizedGesture) {
		case ROTATE_GESTURE:
			// Initialisiere den Rotationsbuffer mit (0,0,0,1)-Werten
			for (int i = 0; i < ROT_BUFFER_SIZE; i++) {
				rotationBuffer->push(Eigen::Quaternionf::Identity());
			}
			setTarget(TARGET_CAMERA);
			setState(CAMERA_ROTATE);
			break;
		case TRANSLATE_GESTURE:
			setTarget(TARGET_CAMERA);
			setState(CAMERA_TRANSLATE);
			break;
		case GRAB_GESTURE:
			if(currentState != OBJECT_MANIPULATE) lastHandOrientationInitialized = false;
			controlHand = risenHand;
			// Initialisiere den Rotationsbuffer mit (0,0,0,1)-Werten
			for (int i = 0; i < ROT_BUFFER_SIZE; i++) {
				rotationBuffer->push(Eigen::Quaternionf::Identity());
			}
			widget->pickModel(0, 0); // löst Ray cast im widget aus
			setTarget(TARGET_OBJECT);
			setState(OBJECT_MANIPULATE);
			break;
		default:
			setTarget(TARGET_CAMERA);
			setState(CAMERA_IDLE);//Zustand nicht wechseln
			resetMotion();
			break;
		}
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
	//Initialisierung der motionParameters
	resetMotion();
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
* Wertet den Buffer mit den erkannten Confidence-Werten aus
* Die Werte werden mittels einer Exponentialfunktion geglättet, dann wird der Maximalwert herausgesucht
*
* @return maxConfidenceGesture Geste, die basierend auf den gepufferten Konfidenzen wahrscheinlich vorgeführt wurde
*/
KinectControl::Gesture KinectControl::evaluateGestureBuffer() {

	//Konfidenz aus dem Puffer
	GestureConfidence currentConfidence;

	//Ergebniskonfidenz der Auswertung
	GestureConfidence finalConfidence = { 0,0,0,0 };

	//Gehe durch den Puffer und glätte alle Komponenten
	for (int i = 0; i < GESTURE_BUFFER_SIZE; i++) {
		currentConfidence = *gestureConfidenceBuffer->get(i);
		finalConfidence.grabConfidence += currentConfidence.grabConfidence * gestureSmooth[i] / gestureSmoothSum;
		finalConfidence.translateCameraConfidence += currentConfidence.translateCameraConfidence * gestureSmooth[i] / gestureSmoothSum;
		finalConfidence.rotateCameraConfidence += currentConfidence.rotateCameraConfidence * gestureSmooth[i] / gestureSmoothSum;
		finalConfidence.unknownConfidence += currentConfidence.unknownConfidence * gestureSmooth[i] / gestureSmoothSum;
	}

	//Werte aus, welche Komponente den höchsten Konfidenzwert hat
	float maxConfidence = finalConfidence.unknownConfidence; Gesture maxConfidenceGesture = UNKNOWN;
	if (maxConfidence < finalConfidence.grabConfidence) { maxConfidence = finalConfidence.grabConfidence; maxConfidenceGesture = GRAB_GESTURE; }
	if (maxConfidence < finalConfidence.translateCameraConfidence) { maxConfidence = finalConfidence.translateCameraConfidence; maxConfidenceGesture = TRANSLATE_GESTURE; }
	if (maxConfidence < finalConfidence.rotateCameraConfidence) { maxConfidence = finalConfidence.rotateCameraConfidence;  maxConfidenceGesture = ROTATE_GESTURE; }

	return maxConfidenceGesture;
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
		stateMachineBufferGestureConfidence();
		stateMachineCompute();
		stateMachineSwitchState();
		
		switch (state)
		{
		case KinectControl::CAMERA_IDLE:
			OutputDebugStringA("CAMERA_IDLE\n");
			break;
		case KinectControl::CAMERA_TRANSLATE:
			OutputDebugStringA("CAMERA_TRANSLATE\n");
			break;
		case KinectControl::CAMERA_ROTATE:
			OutputDebugStringA("CAMERA_ROTATE\n");
			break;
		case KinectControl::OBJECT_MANIPULATE:
			OutputDebugStringA("OBJECT_MANIPULATE\n");
			break;
		default:
			break;
		}
		
		
	}

	// Frame - Speicher freigeben
	bodyFrame->Release();

	return motionParameters;
}
