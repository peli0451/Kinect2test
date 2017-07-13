/**
* KinectControl.cpp
* Managerklasse, Schnittstelle zur Kinect
*/

#include "stdafx.h"
#include <string>
#include <iostream>
#include "KinectControl.h"


/**********************************************************
* Debug-Schalter
**********************************************************/

//#define DEBUG_CONFIG_POSE				//Ausgabe, ob behandelter Körper in Iteration sich in Konfigpose befindet
#define DEBUG_DEVIATIONS				//Ausgabe der bestimmten Abweichungswerte bei der Mastersuche nach einem vollen Puffer
//#define DEBUG_DEVIATION_VERBOSE		//Ausgabe der Abweichungswerte bei Mastersuche immer(!)
#define DEBUG_COLLECTING_COMPARE		//Ausgabe über Sammlungsstatus bei der Mastersuche
//#define DEBUG_STATE_MACHINE_STATE		//Ausgabe des Zustands der State-Machine

/**********************************************************
* Konstruktoren
**********************************************************/

KinectControl::KinectControl() { 
	masterDetermined = false;
	collectFrames = false;
	framesLeftToCollect = MIN_FRAMES_TO_COLLECT;
	maxFramesToCollect = MIN_FRAMES_TO_COLLECT;
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
	bodyFrameReader->SubscribeFrameArrived(&frameArrivedHandle);

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
float KinectControl::getSaneValue(float old, float fresh) {

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
MotionParameters KinectControl::run() {
	if (collectFrames) {
		OutputDebugStringA(std::to_string(framesLeftToCollect).c_str());
		OutputDebugStringA(" von ");
		OutputDebugStringA(std::to_string(maxFramesToCollect).c_str());
		OutputDebugStringA(" noch zu sammeln.\n");
	}
	
	Person master = stateMachine.getMaster();
	MotionParameters motionParameters = stateMachine.getMotionParameters();
	IBodyFrame *bodyFrame;
	HRESULT result;

	//falls kein Master bestimmt ist / werden soll -> Defaultwerte
	if (!masterDetermined) {
		master.setId(-1);
		master.setZ(FLT_MAX);
	}
	
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

	bool lostMaster = false;
	bool searchForMaster = false;
	bool masterTrackedDuringCollection = false;
	BOOLEAN isTracked = false;
	UINT64 currentTrackingId;

	//Falls Master eingespeichert ist, frage Tracking-Status und ID seiner ehemaligen Arrayposition ab
	//Zweck: Prüfen, ob Master nicht mehr da ist
	if (masterDetermined && !collectFrames) {
		trackedBodies[master.getId()]->get_IsTracked(&isTracked);
		trackedBodies[master.getId()]->get_TrackingId(&currentTrackingId);
	}

	//Falls Master eingespeichert ist und mit gleicher ID weiterhin getrackt: nicht suchen (sonst Master suchen)
	if (masterDetermined && isTracked && master.getTrackingId() == currentTrackingId) {
		searchForMaster = false;
	} else {
		if (!searchForMaster) lostMaster = true; //wird nur bei 0-1-Flanke von searchForMaster gesetzt und ist sonst false (verwendet am Ende der Funktion)
		searchForMaster = true;
	}

	/**********************************************************
	* Masterbehandlung
	**********************************************************/
	//Iteriere über trackedBodies
	for (int i = 0; i < numberOfTrackedBodies; i++)
	{
		trackedBodies[i]->get_IsTracked(&isTracked); //ist i-te potentielle Person getrackt?
		trackedBodies[i]->get_TrackingId(&currentTrackingId); //Tracking ID der i-ten getrackten Person

		//Tracking erkannter Personen, Identifikation des Masters
		if (isTracked == TRUE) {
			//Wenn getrackt, lege Gelenkmodell an (Gelenkarray)
			Joint joints[JointType_Count];
			result = trackedBodies[i]->GetJoints(JointType_Count, joints);
			
			if (SUCCEEDED(result)) { //Falls Gelenke erfolgreich geholt
				/**********************************************************
				* Mastersuche
				**********************************************************/
				if (masterDetermined && !collectFrames) {
					//Debugging der Konfigurationspose
					#ifdef DEBUG_CONFIG_POSE
						OutputDebugStringA("isInConfigurationPose: ");
						OutputDebugStringA(std::to_string(isInConfigurationPose(joints)).c_str());
						OutputDebugStringA("\t");
					#endif

					if (searchForMaster && isInConfigurationPose(joints)) {
						//Master wird gesucht, aktueller Körper ist in Konfigpose
						//Beginne mit der Sammlung oder fahre mit der Sammlung fort
						if (collectDeviation[i] == false) { //Beginne mit der Sammlung
							collectDeviation[i] = true;
							trackingId[i] = currentTrackingId;
						}
						else if (collectDeviation[i] == true && currentTrackingId == trackingId[i]) { //Fahre mit der Sammlung fort, falls für diesen Körper schon gesammelt wurde

							//Voller Puffer
							if (deviationBuffer[i]->isFull()) {
								//bei vollem Puffer (Sammlung abgeschlossen) -> Pufferauswertung
								//dies liefert Abweichungswert (deviation)
								//falls Deviation klein genug -> Master gefunden (andernfalls lösche Sammlung)
								float deviation = evaluateDeviationBuffer(deviationBuffer[i]);
								if (deviation < MASTER_ALLOWED_DEVIATION) {
									searchForMaster = false;
									master.setId(i);
									master.setTrackingId(currentTrackingId);
									for (int j = 0; j < numberOfTrackedBodies; j++) {
										collectDeviation[j] = false;
										deviationBuffer[j]->empty();
									}
									#ifdef DEBUG_DEVIATIONS
										OutputDebugStringA("Master gefunden.    \tArray-ID = ");
										OutputDebugStringA(std::to_string(i).c_str());
										OutputDebugStringA("\t Deviation = ");
										OutputDebugStringA(std::to_string(deviation).c_str());
										OutputDebugStringA("\n");
									#endif
								} else { 
									deviationBuffer[i]->empty();
									#ifdef DEBUG_DEVIATIONS
										OutputDebugStringA("Abweichung zu groß. \tArray-ID = ");
										OutputDebugStringA(std::to_string(i).c_str());
										OutputDebugStringA("\t Deviation = ");
										OutputDebugStringA(std::to_string(deviation).c_str());
										OutputDebugStringA("\n");
									#endif
								}
							}
							//Puffer noch nicht voll
							else {
								float deviation = master.compareBodyProperties(joints);
								#ifdef DEBUG_COLLECTING_COMPARE
									OutputDebugStringA("Collecting [");
									OutputDebugStringA(std::to_string(i).c_str());
									OutputDebugStringA("]\t Gefundene Deviation = ");
									OutputDebugStringA(std::to_string(deviation).c_str());
								#endif
								if (deviation != FLT_MAX) {
									deviationBuffer[i]->push(deviation);
									#ifdef DEBUG_COLLECTING_COMPARE
										OutputDebugStringA("\t ... Push.\n");
									#endif
								}
								else {
									deviationBuffer[i]->empty();
									#ifdef DEBUG_COLLECTING_COMPARE
										OutputDebugStringA("\t ... Abbruch.\n");
									#endif
								}
							}
						}
						else { //collectDeviation[i] ist true, aber Tracking-ID an Arraystelle hat sich geändert.
							collectDeviation[i] = false;
						}
					}

					#ifdef DEBUG_DEVIATIONS_VERBOSE									
						OutputDebugStringA("Abweichung:\t");
						OutputDebugStringA(std::to_string(master.compareBodyProperties(joints)).c_str());
						OutputDebugStringA("\n");
					#endif
				}
				/**********************************************************
				* Masterfestlegung
				**********************************************************/
				else if (masterDetermined && collectFrames){
					if (master.getId() == -1) { //Falls Master-ID noch nicht festgelegt, verwende erste Person, die in Pose steht und speichere diese ein
						if (isInConfigurationPose(joints)) {
							master.setId(i);
							master.setTrackingId(currentTrackingId);
						}
					}
					else if (master.getTrackingId() == currentTrackingId) { //Vorgesehener Master ist weiterhin getrackt -> sammle weiter
						masterTrackedDuringCollection = true;
						if (framesLeftToCollect > 0) {
							master.setJoints(joints);
							boolean collectingResult = master.collectBodyProperties();
							if (isInConfigurationPose(joints) == false || collectingResult == false) { //Falls Master bei Sammlung Pose verlässt oder schlechter (ungetrackter) Wert dabei war
								//Beginne Sammlung von vorne
								master.deleteCollectedBodyProperties();
								OutputDebugStringA("RESET\n");
								framesLeftToCollect = maxFramesToCollect;
								//Falls Master aus Pose geriet oder in Teilen inferred war, wird wieder der Erstbeste als nächstes genommen, um einen Master festzulegen
								//Könnte man alternativ auch ohne ID-Rücksetzen machen, würde dann wieder dieselbe Person versuchen, einzuspeichern
								master.setId(-1);
							} else {
								//Frames nur weiterzählen, falls der Frame gut war
								framesLeftToCollect--;
							}
						} else { //Falls fertig gesammelt, berechne BodyProperties und Setze das Sammeln zurück.
							master.calculateBodyProperties();
							collectFrames = false;
						}
					}
				}
				/**********************************************************
				* Primitive Masterfestlegung als Default-Methode
				**********************************************************/
				else {
					//Primitive Masterfestlegung: niedrigster z-Wert des Kopfes
					_CameraSpacePoint headPosition = joints[JointType::JointType_Head].Position;
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

	//Verwerfen der Collection, falls Masterframes gesammelt werden, aber der Master die vorgesehene Master nicht mehr da ist
	if (masterDetermined && collectFrames && !masterTrackedDuringCollection) {
		master.deleteCollectedBodyProperties();
		OutputDebugStringA("RESET\n");
		framesLeftToCollect = maxFramesToCollect;
		master.setId(-1);
	}

	/**********************************************************
	* Masterauslesung (falls einer existiert)
	**********************************************************/
	if (master.getId() != -1 && !lostMaster) {

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

		/**********************************************************
		* State-Machine-Berechnungsschritt
		**********************************************************/
		stateMachine.bufferGestureConfidence();
		stateMachine.compute();
		stateMachine.switchState();
		
		#ifdef DEBUG_STATE_MACHINE_STATE
			switch (stateMachine.getState()) {
			case StateMachine::State::IDLE: OutputDebugStringA("IDLE\n"); break;
			case StateMachine::State::CAMERA_TRANSLATE: OutputDebugStringA("CAMERA_TRANSLATE\n"); break;
			case StateMachine::State::CAMERA_ROTATE: OutputDebugStringA("CAMERA_ROTATE\n"); break;
			case StateMachine::State::OBJECT_MANIPULATE: OutputDebugStringA("OBJECT_MANIPULATE\n"); break;
			case StateMachine::State::FLY: OutputDebugStringA("FLY\n"); break;
			default: break;
			}
		#endif
	}

	//falls Master verloren ging, stoppe jede Bewegung.
	if (lostMaster) {
		stateMachine.stopMotion();
		lostMaster = false;
	}

	// Frame-Speicher freigeben
	bodyFrame->Release();

	return stateMachine.getMotionParameters();
}

/** 
* Bereitet die Mastererfassung vor
*/
void KinectControl::assignMaster() {
	if (!masterDetermined) {
		Person master = stateMachine.getMaster();
		master.setId(-1);
		stateMachine.setMaster(master);

		masterDetermined = true;
		collectFrames = true;
		framesLeftToCollect = MIN_FRAMES_TO_COLLECT;
		maxFramesToCollect = MIN_FRAMES_TO_COLLECT;
	} else {
		switch (WaitForSingleObject(&frameArrivedHandle, 0)) { //Frage frameArrivedHandle ab, Timeout 0 Millisekunden
		case WAIT_TIMEOUT: return;	//kein Frame da
		case WAIT_FAILED: return;	//Fehler
		case WAIT_OBJECT_0: {		//Frame da
			framesLeftToCollect++;
			maxFramesToCollect++; }
		}
	}


}

