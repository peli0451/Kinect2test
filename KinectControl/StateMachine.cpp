#include "stdafx.h"
#include <string>
#include <iostream>
#include "StateMachine.h"
/**
* Konstruktor für StateMachine
*/
StateMachine::StateMachine()
{
	setState(IDLE);
	motionParameters.resetMotion();
}

/**
* Wechselt den aktuellen Zustand
*
* @param newState neuer Zustand
*/
void StateMachine::setState(State newState) {
	state = newState;
}

/**
* Liest den aktuellen Zustand
*
* @return state aktueller Zustand
*/
StateMachine::State StateMachine::getState() {
	return state;
}

/**
* Setzt aktuelle MotionParameters
*
* @param newParameters neue Parameter
*/
void StateMachine::setMotionParameters(MotionParameters newParameters) {
	motionParameters = newParameters;
}

/**
* Liest die aktuellen MotionParameters
*
* @return parameters ausgelesene Parameter
*/
MotionParameters StateMachine::getMotionParameters() {
	return motionParameters;
}

/**
* Berechnet aus der Trackingrückgabe Konfidenzwerte für die verschiedenen Gesten
*/
void StateMachine::bufferGestureConfidence() {
	//aktueller Zustand
	State currentState = getState();

	//Ermittelter Konfidenzwert
	GestureRecognition::GestureConfidence newConfidence = { 0,0,0,0 };
	//Erinnerung Konfidenzschema: { unknown, translate, rotate, grab }


	//Hilfsvariablen für Kombinationen von von Handstates
	boolean leftHandOpen = (master.leftHandState == HandState_Open);
	boolean leftHandClosed = (master.leftHandState == HandState_Closed);
	boolean rightHandOpen = (master.rightHandState == HandState_Open);
	boolean rightHandClosed = (master.rightHandState == HandState_Closed);

	boolean bothHandsOpen = (leftHandOpen && rightHandOpen);
	boolean bothHandsClosed = (leftHandClosed && rightHandClosed);

	GestureRecognition::ControlHand risenHand;
	boolean handRisen;
	boolean risenHandOpen;
	boolean risenHandUnknown;


	if (master.leftHandCurrentPosition.Y - master.rightHandCurrentPosition.Y > .4) {
		risenHand = GestureRecognition::ControlHand::HAND_LEFT;
		handRisen = true;
		risenHandOpen = (master.leftHandState == HandState_Open);
		risenHandUnknown = (master.leftHandState == HandState_Unknown);
	}
	else if (master.rightHandCurrentPosition.Y - master.leftHandCurrentPosition.Y > .4) {
		risenHand = GestureRecognition::ControlHand::HAND_RIGHT;
		handRisen = true;
		risenHandOpen = (master.rightHandState == HandState_Open);
		risenHandUnknown = (master.rightHandState == HandState_Unknown);
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
		if (risenHandOpen) newConfidence = { .0f,.0f,.0f,1.f }; //Grabgeste
		else if (bothHandsClosed) newConfidence = { .0f,.0f,1.f,.0f }; //beide geschlossen
		else if (bothHandsOpen) newConfidence = { .0f,1.f,.0f,.0f }; //beide offen
		else newConfidence = { 1.f,.0f,.0f,.0f }; //Unbekannt
		break;

		//Zustand TRANSLATE -> neben eindeutigen Gesten sind doppeldeutige mit einer geöffneten Hand vermutlich ein Translate
	case State::CAMERA_TRANSLATE:
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
	case State::CAMERA_ROTATE:
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
	case State::OBJECT_MANIPULATE:
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
	gestureRecognition.getConfidenceBuffer()->push(newConfidence);

	//Puffer auswerten
	gestureRecognition.recognize();
}

/**
* Realisiert die Berechnungen der MotionParameters in den Zuständen der State Machine.
*/
void StateMachine::compute() {
	State currentState = getState();
	GestureRecognition::Gesture recognizedGesture = gestureRecognition.getRecognizedGesture();

	switch (currentState) {
	case State::IDLE:
		//TODO
		break;
	case State::CAMERA_TRANSLATE:
		switch (recognizedGesture) {
		case GestureRecognition::Gesture::ROTATE_GESTURE:
			getMotionParameters().resetTranslation();
			break;
		case GestureRecognition::Gesture::TRANSLATE_GESTURE: {
			CameraSpacePoint smoothenedLeftHandPosition = *smoothSpeed(leftHandPositionBuffer);
			CameraSpacePoint smoothenedRightHandPosition = *smoothSpeed(rightHandPositionBuffer);
			float translateX = (smoothenedLeftHandPosition.X + smoothenedRightHandPosition.X) / 2;
			float translateY = (smoothenedLeftHandPosition.Y + smoothenedRightHandPosition.Y) / 2;
			float translateZ = (smoothenedLeftHandPosition.Z + smoothenedRightHandPosition.Z) / 2;
			getMotionParameters().setTranslation(translateX, translateY, translateZ);
			break;
		}
		default:
			getMotionParameters().resetMotion();
			break;
		}
		break;
	case State::CAMERA_ROTATE:
		switch (recognizedGesture) {
		case GestureRecognition::Gesture::ROTATE_GESTURE: {

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
			getMotionParameters().setRotation(smoothRotation(rotationBuffer));

			/* // ursprünglicher Ansatz
			Eigen::Matrix3f rot1 = getRotationMatrix(origin_axis).inverse(); // rot1 ist die Rotationsmatrix, die origin_axis auf den 1. Einheitsvektor dreht
			Eigen::Matrix3f rot2 = getRotationMatrix(target_axis); // rot2 ist die Rotationsmatrix, die den 1. Einheitsvektor auf target_axis dreht
			Eigen::Matrix3f rot3 = rot2 * rot1.inverse(); // rot3 ist die Gesamtrotation (wenn es genau falsch herum rotiert, rot3 invertieren ;) )
			// Quelle für den Lösungsansatz: http://matheplanet.com/default3.html?call=viewtopic.php?topic=64323 Antwort 1
			setRotation(Eigen::Quaternionf(rot3));
			*/
			break;
		}
		case GestureRecognition::Gesture::TRANSLATE_GESTURE:
			getMotionParameters().resetRotation();
			break;
		default:
			getMotionParameters().resetMotion();
			break;
		}
		break;

	case State::OBJECT_MANIPULATE:
	{
		// Bewegung
		CameraSpacePoint smoothenedHandPosition;
		if (controlHand == GestureRecognition::ControlHand::HAND_LEFT) { // welche Hand wird zum Bewegen verwendet?
			smoothenedHandPosition = *smoothSpeed(leftHandPositionBuffer);
		}
		else {
			smoothenedHandPosition = *smoothSpeed(rightHandPositionBuffer);
		}
		getMotionParameters().setTranslation(smoothenedHandPosition.X, smoothenedHandPosition.Y, smoothenedHandPosition.Z);

		// Rotation
		JointOrientation joint_orient[JointType::JointType_Count];
		trackedBodies[master.id]->GetJointOrientations(JointType::JointType_Count, joint_orient); //Ausrichtung der Gelenke holen
		Eigen::Quaternionf currentHandOrientation = Eigen::Quaternionf::Identity();
		Vector4 handOrientation;
		if (controlHand == GestureRecognition::ControlHand::HAND_LEFT) { // welche Hand wird zum Rotieren verwendet?
			handOrientation = joint_orient[JointType::JointType_HandLeft].Orientation; // Ausrichtung der linken Hand
		}
		else {
			handOrientation = joint_orient[JointType::JointType_HandRight].Orientation; // Ausrichtung der rechten Hand
		}
		// übertrage Werte von Kinect Vector4 in Eigen Quaternion
		currentHandOrientation = Eigen::Quaternionf(handOrientation.w, handOrientation.x, handOrientation.y, handOrientation.z);
		if (lastHandOrientationInitialized) { // nur rotieren, wenn lastHandOrientation initialisiert ist
			rotationBuffer->push(currentHandOrientation * lastHandOrientation.inverse()); // Quaternion-Mult. ist Rotation von last auf current
			getMotionParameters().setRotation(smoothRotation(rotationBuffer));
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
void StateMachine::switchState() {
	State currentState = getState();
	GestureRecognition::Gesture recognizedGesture = gestureRecognition.getRecognizedGesture();

	switch (currentState) {
	case IDLE:		//Fallthrough
	case CAMERA_TRANSLATE:	//Fallthrough
	case OBJECT_MANIPULATE: // Fallthrough (TODO Fix)
	case CAMERA_ROTATE:
		switch (recognizedGesture) {
		case GestureRecognition::Gesture::ROTATE_GESTURE:
			// Initialisiere den Rotationsbuffer mit (0,0,0,1)-Werten
			for (int i = 0; i < ROT_BUFFER_SIZE; i++) {
				rotationBuffer->push(Eigen::Quaternionf::Identity());
			}
			getMotionParameters().setTarget(MotionParameters::MotionTarget::TARGET_CAMERA);
			setState(CAMERA_ROTATE);
			break;
		case GestureRecognition::Gesture::TRANSLATE_GESTURE:
			getMotionParameters().setTarget(MotionParameters::MotionTarget::TARGET_CAMERA);
			setState(CAMERA_TRANSLATE);
			break;
		case GestureRecognition::Gesture::GRAB_GESTURE:
			if (currentState != OBJECT_MANIPULATE) lastHandOrientationInitialized = false;
			controlHand = risenHand;
			// Initialisiere den Rotationsbuffer mit (0,0,0,1)-Werten
			for (int i = 0; i < ROT_BUFFER_SIZE; i++) {
				rotationBuffer->push(Eigen::Quaternionf::Identity());
			}
			widget->pickModel(0, 0); // löst Ray cast im widget aus
			getMotionParameters().setTarget(MotionParameters::MotionTarget::TARGET_OBJECT);
			setState(OBJECT_MANIPULATE);
			break;
		default:
			getMotionParameters().setTarget(MotionParameters::MotionTarget::TARGET_CAMERA);
			setState(IDLE);//Zustand nicht wechseln
			getMotionParameters().resetMotion();
			break;
		}
		break;
	default:
		break;
	}
}
