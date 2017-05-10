#include "stdafx.h"
#include <string>
#include <iostream>
#include "StateMachine.h"

const float Pi = 3.14159f;

/**********************************************************
* Konstruktoren
**********************************************************/

StateMachine::StateMachine()
{
	setState(IDLE);
	motionParameters.resetMotion();
}



/**********************************************************
* Getter und Setter
**********************************************************/


void StateMachine::setState(State newState) {
	state = newState;
}

StateMachine::State StateMachine::getState() {
	return state;
}



void StateMachine::setMotionParameters(MotionParameters newParameters) {
	motionParameters = newParameters;
}

MotionParameters StateMachine::getMotionParameters() {
	return motionParameters;
}



void StateMachine::setMaster(Person newMaster) {
	master = newMaster;
}

Person StateMachine::getMaster() {
	return master;
}



void StateMachine::assignWidget(ControlWidget* _widget) {
	widget = _widget;
}

Eigen::AngleAxisf getRotationAngleAxis(Eigen::Vector3f originAxis, Eigen::Vector3f targetAxis) {
	Eigen::Vector3f rotationAxis;
	rotationAxis = originAxis.cross(targetAxis); // Rotationsachse ist der zur von origin_ und target_axis aufgespannten Ebene orthogonale Vektor
	rotationAxis.normalize();
	Eigen::AngleAxisf rot(std::acos(originAxis.dot(targetAxis)), rotationAxis); // Rotation wird beschrieben durch Winkel und Rotationsachse
	return rot;
}

float radToDeg(float rad) {
	return (rad * 180 / Pi);
}

float degToRad(float deg) {
	return (deg * Pi / 180);
}

Eigen::Vector3f convToVec3(CameraSpacePoint csp) {
	Eigen::Vector3f vec;
	vec(0) = csp.X;
	vec(1) = csp.Y;
	vec(2) = csp.Z;
	return vec;
}

Eigen::Vector3f convToVec3(CameraSpacePoint *csp) {
	Eigen::Vector3f vec;
	vec(0) = csp->X;
	vec(1) = csp->Y;
	vec(2) = csp->Z;
	return vec;
}

/**********************************************************
* Funktionen
**********************************************************/

/**
* Berechnet aus der Trackingrückgabe Konfidenzwerte für die verschiedenen Gesten
*/
void StateMachine::bufferGestureConfidence() {
	//aktueller Zustand
	State currentState = getState();

	//Ermittelter Konfidenzwert
	GestureRecognition::GestureConfidence newConfidence = { 0,0,0,0,0 };
	//Erinnerung Konfidenzschema: { unknown, translate, rotate, grab, fly }


	//Hilfsvariablen für Kombinationen von von Handstates
	boolean leftHandOpen = (master.getLeftHandState() == HandState_Open);
	boolean leftHandClosed = (master.getLeftHandState() == HandState_Closed);
	boolean rightHandOpen = (master.getRightHandState() == HandState_Open);
	boolean rightHandClosed = (master.getRightHandState() == HandState_Closed);

	boolean bothHandsOpen = (leftHandOpen && rightHandOpen);
	boolean bothHandsClosed = (leftHandClosed && rightHandClosed);

	Buffer<_CameraSpacePoint>* leftHandPositionBuffer = master.getLeftHandPosBuffer();
	Buffer<_CameraSpacePoint>* rightHandPositionBuffer = master.getRightHandPosBuffer();
	CameraSpacePoint* leftHandCurPos = leftHandPositionBuffer->get(leftHandPositionBuffer->end());
	CameraSpacePoint* rightHandCurPos = rightHandPositionBuffer->get(leftHandPositionBuffer->end());

	//Hilfsvariable für FLY_GESTURE
	Eigen::Vector3f leftVector = convToVec3(leftHandCurPos) - convToVec3(master.getJoints()[JointType_SpineShoulder].Position);
	Eigen::Vector3f rightVector = convToVec3(rightHandCurPos) - convToVec3(master.getJoints()[JointType_SpineShoulder].Position);

	boolean handsTogetherInFront = abs(leftHandCurPos->X - rightHandCurPos->X) < .1f &&
		abs(leftHandCurPos->Y - rightHandCurPos->Y) < .1f &&
		abs(leftHandCurPos->Z - rightHandCurPos->Z) < .1f &&
		leftVector.norm() > .3f && rightVector.norm() > .3f;

	//Hilfsvariablen für GRAB_GESTURE
	boolean handRisen;
	boolean risenHandOpen;
	boolean risenHandUnknown;
	if (leftHandCurPos->Y - rightHandCurPos->Y > .4) {
		master.setRisenHand(GestureRecognition::ControlHand::HAND_LEFT);
		handRisen = true;
		risenHandOpen = (master.getLeftHandState() == HandState_Open);
		risenHandUnknown = (master.getLeftHandState() == HandState_Unknown);
	}
	else if (rightHandCurPos->Y - leftHandCurPos->Y > .4) {
		master.setRisenHand(GestureRecognition::ControlHand::HAND_RIGHT);
		handRisen = true;
		risenHandOpen = (master.getRightHandState() == HandState_Open);
		risenHandUnknown = (master.getRightHandState() == HandState_Unknown);
	}
	else {
		handRisen = false;
		risenHandUnknown = false;
		risenHandOpen = false;
	}


	//Je nach Zustand sind die Bewertungen der Trackingergebnisse anders zu bewerten
	switch (currentState) {

	//Zustand IDLE -> alle Gesten werden nur eindeutig betrachtet
	case State::IDLE:
		if (handsTogetherInFront) newConfidence = { .0f,.0f,.0f,.0f,1.f }; //Flygeste
		else if (risenHandOpen) newConfidence = { .0f,.0f,.0f,1.f,.0f }; //Grabgeste
		else if (bothHandsClosed) newConfidence = { .0f,.0f,1.f,.0f,.0f }; //beide geschlossen
		else if (bothHandsOpen) newConfidence = { .0f,1.f,.0f,.0f,.0f }; //beide offen
		else newConfidence = { 1.f,.0f,.0f,.0f,.0f }; //Unbekannt
		break;

	//Zustand TRANSLATE -> neben eindeutigen Gesten sind doppeldeutige mit einer geöffneten Hand vermutlich ein Translate
	case State::CAMERA_TRANSLATE:
		if (handsTogetherInFront) newConfidence = { .0f,.0f,.0f,.0f,1.f }; //Flygeste
		else if (risenHandOpen) newConfidence = { .0f,.0f,.0f,1.f,.0f }; //Grabgeste
		else if (bothHandsClosed) newConfidence = { .0f,.0f,1.f,.0f,.0f }; //beide geschlossen
		else if (bothHandsOpen) newConfidence = { .0f,1.f,.0f,.0f,.0f }; //beide offen
		else if (leftHandOpen && !rightHandOpen && !rightHandClosed || rightHandOpen && !leftHandOpen && !leftHandClosed)
			newConfidence = { .0f,1.f,.0f,.0f,.0f }; //sehr wahrscheinlich Translate
		else if (leftHandOpen && rightHandClosed || leftHandClosed && rightHandOpen)
			newConfidence = { .0f,.75f,.25f,.0f,.0f }; //doppeldeutig, halte Translate für wahrscheinlicher
		else newConfidence = { .7f,.1f,.1f,.1f,.0f }; //Unbekannt
		break;

	//Zustand ROTATE -> neben eindeutigen Gesten sind doppeldeutige mit einer geschlossenen Hand vermutlich ein Rotate
	case State::CAMERA_ROTATE:
		if (handsTogetherInFront) newConfidence = { .0f,.0f,.0f,.0f,1.f }; //Flygeste
		else if (risenHandOpen) newConfidence = { .0f,.0f,.0f,1.f,.0f }; //Grabgeste
		else if (bothHandsClosed) newConfidence = { .0f,.0f,1.f,.0f,.0f }; //beide geschlossen
		else if (bothHandsOpen) newConfidence = { .0f,1.f,.0f,.0f,.0f }; //beide offen
		else if (leftHandClosed && !rightHandClosed && !rightHandOpen || rightHandClosed && !leftHandClosed && !leftHandOpen)
			newConfidence = { .0f,.0f,1.f,.0f,.0f }; //sehr wahrscheinlich Rotate
		else if (leftHandOpen && rightHandClosed || leftHandClosed && rightHandOpen)
			newConfidence = { .0f,.25f,.75f,.0f,.0f }; //doppeldeutig, halte Rotate für wahrscheinlicher
		else newConfidence = { .7f,.1f,.1f,.1f,.0f }; //Unbekannt
		break;

	//Zustand MANIPULATE -> alle Gesten werden nur eindeutig betrachtet
	case State::OBJECT_MANIPULATE:
		if (handsTogetherInFront) newConfidence = { .0f,.0f,.0f,.0f,1.f }; //Flygeste
		else if (risenHandOpen) newConfidence = { .0f,.0f,.0f,1.f,.0f }; //Grabgeste
		else if (risenHandUnknown) newConfidence = { .4f, .0f, .0f, .6f,.0f };
		else if (bothHandsClosed) newConfidence = { .0f,.0f,1.f,.0f,.0f }; //beide geschlossen
		else if (bothHandsOpen) newConfidence = { .0f,1.f,.0f,.0f,.0f }; //beide offen
		else newConfidence = { .7f,.1f,.1f,.1f,.0f }; //Unbekannt
		break;

	//Zustand FLY -> alle Gesten werden nur eindeutig betrachtet
	case State::FLY: // wie IDLE
		if (handsTogetherInFront) newConfidence = { .0f,.0f,.0f,.0f,1.f }; //Flygeste
		else if (risenHandOpen) newConfidence = { .0f,.0f,.0f,1.f,.0f }; //Grabgeste
		else if (bothHandsClosed) newConfidence = { .0f,.0f,1.f,.0f,.0f }; //beide geschlossen
		else if (bothHandsOpen) newConfidence = { .0f,1.f,.0f,.0f,.0f }; //beide offen
		else newConfidence = { 1.f,.0f,.0f,.0f,.0f }; //Unbekannt
		break;
		break;
	default: break;
	}

	/*
	OutputDebugStringA(std::to_string(newConfidence.unknownConfidence).c_str());
	OutputDebugStringA("\t");
	OutputDebugStringA(std::to_string(newConfidence.translateCameraConfidence).c_str());
	OutputDebugStringA("\t");
	OutputDebugStringA(std::to_string(newConfidence.rotateCameraConfidence).c_str());
	OutputDebugStringA("\t");
	OutputDebugStringA(std::to_string(newConfidence.grabConfidence).c_str());
	OutputDebugStringA("\t");
	*/

	//Berechnete Konfidenzien in den Pufferschreiben
	gestureRecognition.getConfidenceBuffer()->push(newConfidence);

	//Puffer auswerten
	gestureRecognition.recognize();
}

/**
* Realisiert die Berechnungen der MotionParameters in den Zuständen der State Machine.
*/
void StateMachine::compute() {
	//TODO: die folgenden drei "sind konstant", muss man sich nicht ständig holen
	Buffer<_CameraSpacePoint>* leftHandPositionBuffer = master.getLeftHandPosBuffer();
	Buffer<_CameraSpacePoint>* rightHandPositionBuffer = master.getRightHandPosBuffer();
	Buffer<Eigen::Quaternionf>* rotationBuffer = master.getRotationBuffer();

	State currentState = getState();
	GestureRecognition::Gesture recognizedGesture = gestureRecognition.getRecognizedGesture();

	switch (currentState) {
	case State::IDLE:
		//keine Berechnung
		break;
	case State::CAMERA_TRANSLATE:
		switch (recognizedGesture) {
		case GestureRecognition::Gesture::ROTATE_GESTURE:
			break;
		case GestureRecognition::Gesture::TRANSLATE_GESTURE: {
			CameraSpacePoint smoothenedLeftHandPosition = smoothSpeed(leftHandPositionBuffer);
			CameraSpacePoint smoothenedRightHandPosition = smoothSpeed(rightHandPositionBuffer);
			float translateX = (smoothenedLeftHandPosition.X + smoothenedRightHandPosition.X) / 2;
			float translateY = (smoothenedLeftHandPosition.Y + smoothenedRightHandPosition.Y) / 2;
			float translateZ = (smoothenedLeftHandPosition.Z + smoothenedRightHandPosition.Z) / 2;
			motionParameters.setTranslation(translateX, translateY, translateZ);
			break;
		}
		default:
			break;
		}
		break;
	case State::CAMERA_ROTATE:
		switch (recognizedGesture) {
		case GestureRecognition::Gesture::ROTATE_GESTURE: {

			Eigen::Vector3f origin_axis;
			Eigen::Vector3f target_axis; //TODO: Smoothing
			origin_axis = convToVec3(leftHandPositionBuffer->get(leftHandPositionBuffer->end() - 1)) - convToVec3(rightHandPositionBuffer->get(rightHandPositionBuffer->end() - 1));
			origin_axis.normalize();
			// origin_axis enthält nun den normierten Vektor zwischen der rechten und linken Hand im letzten Frame

			/*
			OutputDebugStringA(std::to_string(leftHandPositionBuffer->get(leftHandPositionBuffer->end() - 1)->X).c_str());
			OutputDebugStringA("\t");
			OutputDebugStringA(std::to_string(rightHandPositionBuffer->get(rightHandPositionBuffer->end() - 1)->X).c_str());
			OutputDebugStringA("\n");
			*/
			target_axis = convToVec3(leftHandPositionBuffer->get(leftHandPositionBuffer->end())) - convToVec3(rightHandPositionBuffer->get(rightHandPositionBuffer->end()));
			target_axis.normalize();
			// target_axis enthält nun den normierten Vektor zwischen der rechten und linken Hand im aktuellen Frame

			Eigen::AngleAxisf rot = getRotationAngleAxis(origin_axis, target_axis);
			rotationBuffer->push(Eigen::Quaternionf(rot));
			motionParameters.setRotation(smoothRotation(rotationBuffer));
			break;
		}
		case GestureRecognition::Gesture::TRANSLATE_GESTURE:
			break;
		default:
			break;
		}
		break;

	case State::OBJECT_MANIPULATE:
	{
		// Bewegung
		CameraSpacePoint smoothenedHandPosition;
		if (master.getControlHand() == GestureRecognition::ControlHand::HAND_LEFT) { // welche Hand wird zum Bewegen verwendet?
			smoothenedHandPosition = smoothSpeed(leftHandPositionBuffer);
		}
		else {
			smoothenedHandPosition = smoothSpeed(rightHandPositionBuffer);
		}
		motionParameters.setTranslation(smoothenedHandPosition.X, smoothenedHandPosition.Y, smoothenedHandPosition.Z);

		// Rotation
		Eigen::Quaternionf currentHandOrientation = Eigen::Quaternionf::Identity();
		Vector4 handOrientation;
		if (master.getControlHand() == GestureRecognition::ControlHand::HAND_LEFT) { // welche Hand wird zum Rotieren verwendet?
			handOrientation = (master.getJointOrientations())[JointType::JointType_HandLeft].Orientation; // Ausrichtung der linken Hand
		}
		else {
			handOrientation = (master.getJointOrientations())[JointType::JointType_HandRight].Orientation; // Ausrichtung der rechten Hand
		}
		// übertrage Werte von Kinect Vector4 in Eigen Quaternion
		currentHandOrientation = Eigen::Quaternionf(handOrientation.w, handOrientation.x, handOrientation.y, handOrientation.z);
		if (master.isLastHandOrientationInitialized()) { // nur rotieren, wenn lastHandOrientation initialisiert ist
			Eigen::AngleAxisf orientationDiffAA = Eigen::AngleAxisf(currentHandOrientation * master.getLastHandOrientation().inverse()); // Quaternion-Mult. ist Rotation von last auf current
			orientationDiffAA.angle() = max(orientationDiffAA.angle(), -OBJECT_MAX_ROTATION); //größte plausible Rotation für einen Frame (im Bogenmaß)
			orientationDiffAA.angle() = min(orientationDiffAA.angle(), OBJECT_MAX_ROTATION); //in beide Rotationsrichtungen
			rotationBuffer->push(Eigen::Quaternionf(orientationDiffAA));
			motionParameters.setRotation(smoothRotation(rotationBuffer));
		}
		Eigen::AngleAxisf aa = Eigen::AngleAxisf(currentHandOrientation);
		
		master.setLastHandOrientation(currentHandOrientation);
		master.setLastHandOrientationInitialized(true);
		break;
	}
	case State::FLY: 
	{
		motionParameters.setTranslation(0.0f, 0.0f, FLY_TRANSLATION_FACTOR); // immer leichte Bewegung nach vorn

		CameraSpacePoint *leftHandPosition = leftHandPositionBuffer->get(leftHandPositionBuffer->end());
		CameraSpacePoint *rightHandPosition = rightHandPositionBuffer->get(rightHandPositionBuffer->end());
		Eigen::Vector3f handPosition = (convToVec3(leftHandPosition) + convToVec3(rightHandPosition)) / 2;

		//CameraSpacePoint leftShoulderPosition = master.getJoints()[JointType::JointType_ShoulderLeft].Position;
		//CameraSpacePoint rightShoulderPosition = master.getJoints()[JointType::JointType_ShoulderRight].Position; 
		//Eigen::Vector3f shoulderPosition = (convToVec3(leftShoulderPosition) + convToVec3(rightShoulderPosition)) / 2; // mittlere Schulterposition, wenn notwendig puffern und filtern
		Eigen::Vector3f shoulderPosition = convToVec3(master.getJoints()[JointType::JointType_SpineShoulder].Position);

		Eigen::Vector3f originAxis(0.0f, 0.0f, 1.0f); // im Moment immer (0,0,1), später vllt Körpernormale
		Eigen::Vector3f targetAxis = shoulderPosition - handPosition;
		targetAxis.normalize();
		Eigen::AngleAxisf flyRotation = getRotationAngleAxis(originAxis, targetAxis);
		
		float rotationDegrees = radToDeg(flyRotation.angle());
		if (rotationDegrees > FLY_SEGMENT2_DEGREE) { // bei recht großem Winkel stärkere Drehung als linear
			rotationDegrees = ((rotationDegrees - FLY_SEGMENT2_DEGREE) * FLY_SEGMENT2_FACTOR) + FLY_SEGMENT2_DEGREE;
		}
		flyRotation.angle() = degToRad(rotationDegrees);

		flyRotation.angle() *= FLY_ROTATION_FACTOR;
		motionParameters.setRotation(Eigen::Quaternionf(flyRotation));

		
		break;
	}
	default:
		break;
	}
}

/**
* Realisiert die Zustandsübergänge der StateMachine
*/
void StateMachine::switchState() {
	State currentState = getState();
	GestureRecognition::Gesture recognizedGesture = gestureRecognition.getRecognizedGesture();
	Buffer<Eigen::Quaternionf>* rotationBuffer = master.getRotationBuffer();
	State newState = currentState;

	switch (recognizedGesture) {
		case GestureRecognition::Gesture::ROTATE_GESTURE:
			// Initialisiere den Rotationsbuffer mit (0,0,0,1)-Werten
			for (int i = 0; i < Person::ROT_BUFFER_SIZE; i++) {
				rotationBuffer->push(Eigen::Quaternionf::Identity());
			}
			motionParameters.setTarget(MotionParameters::MotionTarget::TARGET_CAMERA);
			newState = CAMERA_ROTATE;
			break;
		case GestureRecognition::Gesture::TRANSLATE_GESTURE:
			motionParameters.setTarget(MotionParameters::MotionTarget::TARGET_CAMERA);
			newState = CAMERA_TRANSLATE;
			break;
		case GestureRecognition::Gesture::GRAB_GESTURE:
			if (currentState != OBJECT_MANIPULATE)
				master.setLastHandOrientationInitialized(false);
			master.setControlHand(master.getRisenHand());
			// Initialisiere den Rotationsbuffer mit (0,0,0,1)-Werten
			for (int i = 0; i < Person::ROT_BUFFER_SIZE; i++) {
				rotationBuffer->push(Eigen::Quaternionf::Identity());
			}
			widget->pickModel(0, 0); // löst Ray cast im widget aus
			motionParameters.setTarget(MotionParameters::MotionTarget::TARGET_OBJECT);
			newState = OBJECT_MANIPULATE;
			break;
		case GestureRecognition::Gesture::FLY_GESTURE:
			motionParameters.setTarget(MotionParameters::MotionTarget::TARGET_CAMERA);
			newState = FLY;
			break;
		default: //Übergang zu IDLE, entspricht UNKNOWN
			motionParameters.setTarget(MotionParameters::MotionTarget::TARGET_CAMERA);
			newState = IDLE;//Zustand nicht wechseln
			motionParameters.resetMotion();
			break;
	}

	setState(newState);
	if (newState != currentState) motionParameters.resetMotion();
}


/**
* Glättet gepufferte Positionen
*
* @param buffer Puffer für Positionen
*/
CameraSpacePoint StateMachine::smoothSpeed(Buffer<CameraSpacePoint>* buffer) {
	CameraSpacePoint speedPoint = { 0,0,0 };
	// läuft atm vom ältesten zum jüngsten, deshalb schleife absteigend
	for (int i = buffer->end(); i >= 1; i--) {
		// von jung zu alt, stoppt eins später für ableitung
		// ich achte mal noch nich drauf was bei nicht vollem puffer genau passiert
		CameraSpacePoint *curPoint = buffer->get(i);
		CameraSpacePoint *nextPoint = buffer->get(i - 1);
		float smoothing = smoothingFactor[i - 1] / smoothingSum;

		speedPoint.X += (curPoint->X - nextPoint->X) * smoothing;
		speedPoint.Y += (curPoint->Y - nextPoint->Y) * smoothing;
		speedPoint.Z += (curPoint->Z - nextPoint->Z) * smoothing;
	}
	return speedPoint;
}

/**
* Glättet gepufferte Rotationen (in Form von Quaternions)
*
* @param buffer Puffer für Rotationen
*/
Eigen::Quaternionf StateMachine::smoothRotation(Buffer<Eigen::Quaternionf> *buffer) {
	Eigen::Quaternionf rotation = Eigen::Quaternionf::Identity();
	for (int i = buffer->end(); i >= 0; i--) {
		Eigen::Quaternionf *cur_rot = buffer->get(i);
		float smoothing = rotationSmoothingFactor[i] / rotationSmoothingSum;
		Eigen::Quaternionf downscaled_rot = Eigen::Quaternionf::Identity().slerp(smoothing, *cur_rot); // skaliert einen Puffereintrag
		rotation *= downscaled_rot;
	}
	return rotation;
}