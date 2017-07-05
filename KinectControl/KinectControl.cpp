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
	bodyFrameSource->get_BodyCount(&numberOfTrackedBodies); //max. mgl. Anzahl parallel trackbarer Personen
	bodyFrameSource->OpenReader(&bodyFrameReader);

	for (int i = 0; i < BODY_COUNT; i++) {
		deviationBuffer[i] = new Buffer<float>(NUMBER_OF_COLLECTED_FRAMES);
	}

	for (int i = 0; i < BODY_COUNT; i++) {
		collectDeviation[i] = false;
	}
}

/**
* Bestimmt ein Mittel aus allen Abweichungen einer Person
* @param deviationBuffer Zeiger auf einen Buffer
* @return arrithmetisches Mittel aller Abweichungen
*/
float KinectControl::evaluateDeviationBuffer(Buffer<float> *deviationBuffer) {
	float arrithmetic_mean = 0;
	for (int i = 0; i < NUMBER_OF_COLLECTED_FRAMES; i++) {
		arrithmetic_mean += *deviationBuffer->get(i);
	}
	return arrithmetic_mean / NUMBER_OF_COLLECTED_FRAMES;
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
* Prüft, ob eine Person die vordefinierte Konfigurationspose eingenommen hat
*
* @return Angabe (boolesch), ob in Pose (true) oder nicht (false)
*/

bool KinectControl::isInConfigurationPose(Joint* joints)
{
	Eigen::Vector3f originAxis = StateMachine::convToVec3(joints[JointType_ShoulderRight].Position) - StateMachine::convToVec3(joints[JointType_ShoulderLeft].Position);
	originAxis.normalize();
	Eigen::Vector3f targetAxis(1.0f, 0, 0);
	Eigen::AngleAxisf bodyRotation = StateMachine::getRotationAngleAxis(originAxis, targetAxis);
	
	Eigen::Vector3f rotatedLeftHandPosition = 
		bodyRotation * StateMachine::convToVec3(joints[JointType_HandLeft].Position);

	Eigen::Vector3f rotatedRightHandPosition = 
		bodyRotation * StateMachine::convToVec3(joints[JointType_HandRight].Position);;

	Eigen::Vector3f rotatedLeftHipPosition = 
		bodyRotation * StateMachine::convToVec3(joints[JointType_HipLeft].Position);

	Eigen::Vector3f rotatedRightHipPosition =
		bodyRotation *  StateMachine::convToVec3(joints[JointType_HipRight].Position);

	//OutputDebugStringA(std::to_string(originAxis.x()).c_str()); OutputDebugStringA("\t"); OutputDebugStringA(std::to_string(originAxis.y()).c_str()); OutputDebugStringA("\t");  OutputDebugStringA(std::to_string(originAxis.z()).c_str());
	//OutputDebugStringA("\n");

	/* linke Hand vor und nach Rota + Winkel
	OutputDebugStringA(std::to_string(joints[JointType_HandLeft].Position.X).c_str());
	OutputDebugStringA("\t"); OutputDebugStringA(std::to_string(joints[JointType_HandLeft].Position.Y).c_str());
	OutputDebugStringA("\t"); OutputDebugStringA(std::to_string(joints[JointType_HandLeft].Position.Z).c_str());
	OutputDebugStringA("\n");
	OutputDebugStringA(std::to_string(rotatedLeftHandPosition.x()).c_str());
	OutputDebugStringA("\t"); OutputDebugStringA(std::to_string(rotatedLeftHandPosition.y()).c_str());
	OutputDebugStringA("\t"); OutputDebugStringA(std::to_string(rotatedLeftHandPosition.z()).c_str());
	OutputDebugStringA("\n");
	OutputDebugStringA(std::to_string(bodyRotation.angle()*180/3.142f).c_str());
	OutputDebugStringA("\n");
	*/

	/* linke und rechte Hand (z-Koordinate) vor und nach Rota + Winkel
	OutputDebugStringA(std::to_string(joints[JointType_HandLeft].Position.Z).c_str());
	OutputDebugStringA("\t"); OutputDebugStringA(std::to_string(joints[JointType_HandRight].Position.Z).c_str());
	OutputDebugStringA("\n");
	OutputDebugStringA(std::to_string(rotatedLeftHandPosition.z()).c_str());
	OutputDebugStringA("\t"); OutputDebugStringA(std::to_string(rotatedRightHandPosition.z()).c_str());
	OutputDebugStringA("\n");
	OutputDebugStringA(std::to_string(bodyRotation.angle()*180/3.142f).c_str());
	OutputDebugStringA("\n");
	*/

	/* Debugging der Constraints 
	OutputDebugStringA(std::to_string(abs(rotatedLeftHandPosition.y() - rotatedRightHandPosition.y())).c_str());
	OutputDebugStringA("\n"); OutputDebugStringA(std::to_string(abs(rotatedLeftHandPosition.z() - rotatedRightHandPosition.z())).c_str());
	OutputDebugStringA("\n"); OutputDebugStringA(std::to_string(abs(rotatedLeftHipPosition.z() - rotatedRightHipPosition.z())).c_str());

	OutputDebugStringA("\n"); OutputDebugStringA(std::to_string(abs(rotatedLeftHandPosition.y() - rotatedLeftHipPosition.y())).c_str());
	OutputDebugStringA("\n"); OutputDebugStringA(std::to_string(abs(rotatedRightHandPosition.y() - rotatedRightHipPosition.y())).c_str());
	OutputDebugStringA("\n"); OutputDebugStringA(std::to_string(abs(rotatedLeftHandPosition.z() - rotatedLeftHipPosition.z())).c_str());
	OutputDebugStringA("\n"); OutputDebugStringA(std::to_string(abs(rotatedRightHandPosition.z() - rotatedRightHipPosition.z())).c_str());

	OutputDebugStringA("\n"); OutputDebugStringA(std::to_string(abs(rotatedLeftHandPosition.x() - rotatedLeftHipPosition.x())).c_str());
	OutputDebugStringA("\n"); OutputDebugStringA(std::to_string(abs(rotatedRightHandPosition.x() - rotatedRightHipPosition.x())).c_str());

	OutputDebugStringA("\n"); OutputDebugStringA(std::to_string(joints[JointType_HandLeft].Position.X).c_str());
	OutputDebugStringA("\n"); OutputDebugStringA(std::to_string(joints[JointType_HandRight].Position.X).c_str());

	OutputDebugStringA("\n"); OutputDebugStringA("\n");
	OutputDebugStringA(std::to_string(bodyRotation.angle() * 180 / 3.142f).c_str());
	OutputDebugStringA("\n");
	*/

	return (abs(rotatedLeftHandPosition.y() - rotatedRightHandPosition.y()) < .1f) // Hände etwa auf gleicher Höhe
		&& (abs(rotatedLeftHandPosition.z() - rotatedRightHandPosition.z()) < .1f) // Hände etwa in gleicher Entfernung
		&& (abs(rotatedLeftHipPosition.z() - rotatedRightHipPosition.z()) < .1f) // Hüfte nicht verdreht (gegenüber Schultern)
		&& (abs(rotatedLeftHandPosition.y() - rotatedLeftHipPosition.y()) < .1f)
		&& (abs(rotatedRightHandPosition.y() - rotatedRightHipPosition.y()) < .1f) // Hände ungefähr auf Hüfthöhe
		&& (abs(rotatedLeftHandPosition.z() - rotatedLeftHipPosition.z()) < .1f)
		&& (abs(rotatedRightHandPosition.z() - rotatedRightHipPosition.z()) < .1f) // Hände ungefähr auf Hüftentfernung
		&& (abs(rotatedLeftHandPosition.x() - rotatedLeftHipPosition.x()) < .25f)
		&& (abs(rotatedRightHandPosition.x() - rotatedRightHipPosition.x()) < .25f)
		//&& (abs(rotatedLeftHandPosition.x() - rotatedRightHandPosition.x())
		//	< 5.0f * abs(rotatedLeftHipPosition.x() - rotatedRightHipPosition.x())) // Handabstand < 5*Hüftabstand (x-Werte)
		//&& (abs(rotatedLeftHandPosition.x() - rotatedRightHandPosition.x())
		//	> 2.0f * abs(rotatedLeftHipPosition.x() - rotatedRightHipPosition.x())) // Handabstand > 2*Hüftabstand (x-Werte)
		&& (joints[JointType::JointType_HandLeft].Position.X < joints[JointType::JointType_HandRight].Position.X)
		&& (joints[JointType::JointType_HipLeft].Position.X < joints[JointType::JointType_HipRight].Position.X); // richtig herum vor der Kamera

	/*
	return (abs(joints[JointType::JointType_HandLeft].Position.Y - joints[JointType::JointType_HandRight].Position.Y) < .1f) // Hände etwa auf gleicher Höhe
		&& (abs(joints[JointType::JointType_HandLeft].Position.Z - joints[JointType::JointType_HandRight].Position.Z) < .1f) // Hände etwa in gleicher Entfernung
		&& (abs(joints[JointType::JointType_HipLeft].Position.Z - joints[JointType::JointType_HipRight].Position.Z) < .1f) // Hüfte nicht verdreht (gerade vor der Kamera)
		&& (abs(joints[JointType::JointType_HandLeft].Position.Y - joints[JointType::JointType_HipLeft].Position.Y) < .2f)
		&& (abs(joints[JointType::JointType_HandRight].Position.Y - joints[JointType::JointType_HipRight].Position.Y) < .2f) // Hände ungefähr auf Hüfthöhe
		&& (abs(joints[JointType::JointType_HandLeft].Position.Z - joints[JointType::JointType_HipLeft].Position.Z) < .15f)
		&& (abs(joints[JointType::JointType_HandRight].Position.Z - joints[JointType::JointType_HipRight].Position.Z) < .15f) // Hände ungefähr auf Hüftentfernung
		&& (abs(joints[JointType::JointType_HandLeft].Position.X - joints[JointType::JointType_HandRight].Position.X)
			< 5.0f * abs(joints[JointType::JointType_HipLeft].Position.X - joints[JointType::JointType_HipRight].Position.X)) // Handabstand < 4*Hüftabstand (x-Werte)
		&& (abs(joints[JointType::JointType_HandLeft].Position.X - joints[JointType::JointType_HandRight].Position.X)
			> 2.0f * abs(joints[JointType::JointType_HipLeft].Position.X - joints[JointType::JointType_HipRight].Position.X)) // Handabstand > 2*Hüftabstand (x-Werte)
		&& (joints[JointType::JointType_HandLeft].Position.X < joints[JointType::JointType_HandRight].Position.X)
		&& (joints[JointType::JointType_HipLeft].Position.X < joints[JointType::JointType_HipRight].Position.X); // richtig herum vor der Kamera
	*/
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

	//[deprecated]
	//Plan: Iterieren über Köpfe, den niedrigsten z-Wert als Master wählen, Mastervariable
	if (!masterDetermined) {
		master.setId(-1);
		master.setZ(FLT_MAX);
	}
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

	bool searchForMaster;
	bool masterTrackedDuringCollection = false;
	BOOLEAN isTracked;
	UINT64 currentTrackingId;

	if (masterDetermined && !collectFrames) {
		trackedBodies[master.getId()]->get_IsTracked(&isTracked);
		trackedBodies[master.getId()]->get_TrackingId(&currentTrackingId);
	}

	if (masterDetermined && isTracked && master.getTrackingId() == currentTrackingId)
		searchForMaster = false;
	else
		searchForMaster = true;

	//TODO hier noch alte, primitive Mastererkennung
		for (int i = 0; i < numberOfTrackedBodies; i++)
		{
			trackedBodies[i]->get_IsTracked(&isTracked); //ist i-te potentielle Person getrackt
			trackedBodies[i]->get_TrackingId(&currentTrackingId); //Tracking ID der i-ten getrackten Person

			//Tracking erkannter Personen, Identifikation des Masters
			if (isTracked == TRUE) {
				//Wenn getrackt, lege Gelenkmodell an (Gelenkarray)
				Joint joints[JointType_Count];
				result = trackedBodies[i]->GetJoints(JointType_Count, joints);
				if (SUCCEEDED(result)) {
					//Falls Gelenke erfolgreich geholt
					_CameraSpacePoint headPosition = joints[JointType::JointType_Head].Position;
					//----------------------
					// Mastersuche
					

					if (masterDetermined && !collectFrames) {
						
						//master.compareBodyProperties(joints);
						/*
						OutputDebugStringA("isInConfigurationPose: ");
						OutputDebugStringA(std::to_string(isInConfigurationPose(joints)).c_str());
						OutputDebugStringA("\t");
						*/

						//OutputDebugStringA(std::to_string(searchForMaster).c_str());
						if (searchForMaster && isInConfigurationPose (joints)) {
							if (collectDeviation[i] == false) {
								collectDeviation[i] = true;
								trackingId[i] = currentTrackingId;
							}
							else if (collectDeviation[i] == true && currentTrackingId == trackingId[i]) {
								if (deviationBuffer[i]->isFull()) {
									float deviation = evaluateDeviationBuffer(deviationBuffer[i]);
									if (deviation < 50.0f) {
										searchForMaster = false;
										master.setId(i);
										master.setTrackingId(currentTrackingId);
										for (int j = 0; j < numberOfTrackedBodies; j++) {
											collectDeviation[j] = false;
											deviationBuffer[j]->empty();
										}
										OutputDebugStringA("Master gefunden.\t");
										OutputDebugStringA(std::to_string(deviation).c_str());
										OutputDebugStringA("\n");
									} else { 
										deviationBuffer[i]->empty();
										OutputDebugStringA("----------------\t");
										OutputDebugStringA(std::to_string(deviation).c_str());
										OutputDebugStringA("\n");
									}
									//OutputDebugStringA(std::to_string(blub).c_str());
									//OutputDebugStringA("\n");
								}
								else {
									OutputDebugStringA("Collecting [");
									OutputDebugStringA(std::to_string(i).c_str());
									OutputDebugStringA("]\n");
									float deviation = master.compareBodyProperties(joints);
									if (deviation != FLT_MAX) {
										deviationBuffer[i]->push(deviation);
									}
									else {
										deviationBuffer[i]->empty();
									}
								}
							}
							else {
								collectDeviation[i] = false;
							}
						}
						/*												
						OutputDebugStringA("Abweichung:\t");
						OutputDebugStringA(std::to_string(master.compareBodyProperties(joints)).c_str());
						OutputDebugStringA("\n");
						*/
					}
					//----------------------
					// Masterfestlegung
					else if (masterDetermined && collectFrames){
						
						if (master.getId() == -1) {
							if (isInConfigurationPose(joints)) {
								master.setId(i);
								master.setTrackingId(currentTrackingId);
							}
						}
						else if (master.getTrackingId() == currentTrackingId) {
							masterTrackedDuringCollection = true;
							//OutputDebugStringA(std::to_string(collectedFrames).c_str()); OutputDebugStringA("\n");
							if (collectedFrames < 20) {

								master.setJoints(joints); //@TODO Fragwürdige Lösung? Nochmal z-TEst oder so

								if (isInConfigurationPose(joints) == false || master.collectBodyProperties() == false) {
									master.deleteCollectedBodyProperties();
									OutputDebugStringA("RESET\n");
									collectedFrames = 0;
									master.setId(-1);
								}
								collectedFrames++;
							}
							else {
								master.calculateBodyProperties();
								collectedFrames = 0;
								collectFrames = false;
							}
						}
					}
					//----------------------
					// Masterfestlegung primitiv
					else {
						if (headPosition.Z < master.getZ()) {
							master.setJoints(joints);
							master.setId(i);
							master.setZ(headPosition.Z);
							master.setTrackingId(currentTrackingId);
						}
					}

				}
			}
		}

		stateMachine.setMaster(master);

		if (masterDetermined && collectFrames && !masterTrackedDuringCollection) {
			master.deleteCollectedBodyProperties();
			OutputDebugStringA("RESET\n");
			collectedFrames = 0;
			master.setId(-1);
		}

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

/** 
* Bereitet die Mastererfassung vor
*/
void KinectControl::assignMaster() {
	masterDetermined = true;
	collectFrames = true;
	collectedFrames = 0;
	Person master = stateMachine.getMaster();
	master.setId(-1);
	stateMachine.setMaster(master);
}

