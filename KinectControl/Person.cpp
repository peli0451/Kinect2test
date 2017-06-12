#include "stdafx.h"
#include "Person.h"
#include <cmath>

//#define DEBUG_BODY_PROPERTIES //Debugausgaben Körpermerkmale
//#define DEBUG_ARM_DIFFERENCE //Debugmeldung über Oberarmlängendifferenz
//#define DEBUG_LEG_DIFFERENCE //Debugmeldung über Schenkellängendifferenz
#define DEBUG_ACCUMULATED_ERROR //Debugmeldung über errechnete Abweichung
//#define DEBUG_NOTIFY_BAD_PROPERTY //Debugbenachrichtigungen für schlechte Werte
#define DEBUG_COLLECTING //Debugmeldung über Standardabweichung bei Masterfestlegung
//#define DEBUG_VERBOSE //Debugmeldungen über Funktionsaufrufe und berechnungen

/**********************************************************
* Konstruktoren
**********************************************************/

Person::Person()
{
	id = -1;

	leftHandCurrentPosition = { 0,0,0 };
	rightHandCurrentPosition = { 0,0,0 };

	leftHandLastPosition = { 0,0,0 };
	rightHandLastPosition = { 0,0,0 };

	leftHandState = HandState_Unknown;
	rightHandState = HandState_Unknown;

	z = FLT_MAX;

	// Handpositionenbuffer
	leftHandPositionBuffer = new Buffer<CameraSpacePoint>(POS_BUFFER_SIZE);
	rightHandPositionBuffer = new Buffer<CameraSpacePoint>(POS_BUFFER_SIZE);

	// Rotationenbuffer
	rotationBuffer = new Buffer<Eigen::Quaternionf>(ROT_BUFFER_SIZE);

	// Grenzen (Minimum und Maximum) für jede einzelne Körperproportion festlegen,
	// innerhalb derer sich die "vernünftigen" Werte für eine Proportion befinden sollten,
	// falls der Wert nicht durch einen Messfehler auf einen unglaubwürdig zu kleinen oder zu
	// grossen Wert springt
	bodyPropertiesLimits[LEFT_UPPER_ARM_LENGTH].min = 0.1f;
	bodyPropertiesLimits[LEFT_UPPER_ARM_LENGTH].max = 0.6f;
	bodyPropertiesLimits[RIGHT_UPPER_ARM_LENGTH].min = bodyPropertiesLimits[LEFT_UPPER_ARM_LENGTH].min;
	bodyPropertiesLimits[RIGHT_UPPER_ARM_LENGTH].max = bodyPropertiesLimits[LEFT_UPPER_ARM_LENGTH].max;
	bodyPropertiesLimits[LEFT_LOWER_ARM_LENGTH].min = 0.1f;
	bodyPropertiesLimits[LEFT_LOWER_ARM_LENGTH].max = 0.6f;
	bodyPropertiesLimits[RIGHT_LOWER_ARM_LENGTH].min = bodyPropertiesLimits[LEFT_UPPER_ARM_LENGTH].min;
	bodyPropertiesLimits[RIGHT_LOWER_ARM_LENGTH].max = bodyPropertiesLimits[LEFT_UPPER_ARM_LENGTH].max;
	bodyPropertiesLimits[LEFT_UPPER_LEG_LENGTH].min = 0.2f;
	bodyPropertiesLimits[LEFT_UPPER_LEG_LENGTH].max = 0.8f;
	bodyPropertiesLimits[RIGHT_UPPER_LEG_LENGTH].min = bodyPropertiesLimits[LEFT_UPPER_LEG_LENGTH].min;
	bodyPropertiesLimits[RIGHT_UPPER_LEG_LENGTH].max = bodyPropertiesLimits[LEFT_UPPER_LEG_LENGTH].max;
	bodyPropertiesLimits[SHOULDER_WIDTH].min = 0.2f;
	bodyPropertiesLimits[SHOULDER_WIDTH].max = 0.7f;
	bodyPropertiesLimits[HIP_WIDTH].min = 0.2f;
	bodyPropertiesLimits[HIP_WIDTH].max = 0.7f;
	bodyPropertiesLimits[TORSO_LENGTH].min = 0.2f;
	bodyPropertiesLimits[TORSO_LENGTH].max = 1.0f;
	bodyPropertiesLimits[RIGHT_HAND_TO_WRIST].min = 0.01f;
	bodyPropertiesLimits[RIGHT_HAND_TO_WRIST].max = 0.5f;
	bodyPropertiesLimits[LEFT_HAND_TO_WRIST].min = 0.01f;
	bodyPropertiesLimits[LEFT_HAND_TO_WRIST].max = 0.5f;
	bodyPropertiesLimits[NECK_TO_HEAD].min = 0.01f;
	bodyPropertiesLimits[NECK_TO_HEAD].max = 0.5f;

	numberOfWeights = sizeof(bodyPropertiesWeights) / sizeof(float);
}



/**********************************************************
* Getter und Setter
**********************************************************/

void Person::setId(int newId) {
	id = newId;
}

int Person::getId() {
	return id;
}



void Person::setZ(float newZ) {
	z = newZ;
}

float Person::getZ() {
	return z;
}



void Person::setJoints(Joint* newJoints) {
	for (int i = 0; i<JointType_Count; i++)
		joints[i] = newJoints[i];
}

Joint* Person::getJoints() {
	return joints;
}



void Person::setJointOrientations(JointOrientation* newOrientations) {
	for (int i = 0; i<JointType_Count; i++)
		jointOrientations[i] = newOrientations[i];
}

JointOrientation* Person::getJointOrientations() {
	return jointOrientations;
}



void Person::setLeftHandState(HandState newState) {
	leftHandState = newState;
};

HandState Person::getLeftHandState() {
	return leftHandState;
};



void Person::setRightHandState(HandState newState) {
	rightHandState = newState;
};

HandState Person::getRightHandState() {
	return rightHandState;
};



Buffer<_CameraSpacePoint>* Person::getLeftHandPosBuffer() {
	return leftHandPositionBuffer;
}



Buffer<_CameraSpacePoint>* Person::getRightHandPosBuffer() {
	return rightHandPositionBuffer;
}



Buffer<Eigen::Quaternionf>* Person::getRotationBuffer() {
	return rotationBuffer;
}



void Person::setLastHandOrientation(Eigen::Quaternionf orientation) {
	lastHandOrientation = orientation;
	lastHandOrientationInitialized = true;
}

Eigen::Quaternionf Person::getLastHandOrientation() {
	return lastHandOrientation;
}



void Person::setLastHandOrientationInitialized(bool val) {
	lastHandOrientationInitialized = val;
}

bool Person::isLastHandOrientationInitialized() {
	return lastHandOrientationInitialized;
}



void Person::setControlHand(GestureRecognition::ControlHand hand) {
	controlHand = hand;
}

GestureRecognition::ControlHand Person::getControlHand() {
	return controlHand;
}



void Person::setRisenHand(GestureRecognition::ControlHand hand) {
	risenHand = hand;
}

GestureRecognition::ControlHand Person::getRisenHand() {
	return risenHand;
}
/**********************************************************
* Funktionen
**********************************************************/

/** Berechnet aus den übergebenen Gelenkpunkten durch Ermittlung der euklidische Distanz
* zwischen zwei Punkten die Körperproportionen gemäß dem BODY_PROPERTIES enum und speichert
* diese in einem Array (jeder Eintrag im Array entspricht einer Konstante aus dem genannten enum)
*
* Die errechnete Distanz wird hierbei auf 0.0f gesetzt, falls einer der beiden Endpunkte
* der Distanz nicht den TrackingState "Tracked" haben (also "NotTracked" oder "Inferred" sind)
* und somit für unsere Zwecke nicht von der Kinect mit genügend großer Konfidenz erkannt werden
* bsplsweise: Berechnung der linken Oberarmlänge aus der Distanz zwischen den Gelenkpunkten
* von linker Schulter und linkem Elbogen
*
* @param extractedBodyProperties Zeiger auf den Array in dem die Körperproportionen gespeichert werden
* @param inputJoints Zeiger auf den Array mit den Gelenkpunkten
*/
void Person::extractBodyProperties(float* extractedBodyProperties, Joint* inputJoints)
{
#ifdef DEBUG_VERBOSE
	OutputDebugStringA("DEBUG: extractBodyProperties aufgerufen.\n");
#endif
	_CameraSpacePoint shoulderLeft = inputJoints[JointType::JointType_ShoulderLeft].Position;
	_CameraSpacePoint shoulderRight = inputJoints[JointType::JointType_ShoulderRight].Position;
	_CameraSpacePoint handLeft = inputJoints[JointType::JointType_HandLeft].Position;
	_CameraSpacePoint handRight = inputJoints[JointType::JointType_HandRight].Position;
	_CameraSpacePoint wristLeft = inputJoints[JointType::JointType_WristLeft].Position;
	_CameraSpacePoint wristRight = inputJoints[JointType::JointType_WristRight].Position;
	_CameraSpacePoint spineShoulder = inputJoints[JointType::JointType_SpineShoulder].Position;
	_CameraSpacePoint elbowLeft = inputJoints[JointType::JointType_ElbowLeft].Position;
	_CameraSpacePoint elbowRight = inputJoints[JointType::JointType_ElbowRight].Position;
	_CameraSpacePoint neck = inputJoints[JointType::JointType_Neck].Position;
	_CameraSpacePoint spineBase = inputJoints[JointType::JointType_SpineBase].Position;
	_CameraSpacePoint hipLeft = inputJoints[JointType::JointType_HipLeft].Position;
	_CameraSpacePoint hipRight = inputJoints[JointType::JointType_HipRight].Position;
	_CameraSpacePoint kneeLeft = inputJoints[JointType::JointType_KneeLeft].Position;
	_CameraSpacePoint kneeRight = inputJoints[JointType::JointType_KneeRight].Position;
	_CameraSpacePoint head = inputJoints[JointType::JointType_Head].Position;

	TrackingState shoulderLeftState = inputJoints[JointType::JointType_ShoulderLeft].TrackingState;
	TrackingState shoulderRightState = inputJoints[JointType::JointType_ShoulderRight].TrackingState;
	TrackingState handLeftState = inputJoints[JointType::JointType_HandLeft].TrackingState;
	TrackingState handRightState = inputJoints[JointType::JointType_HandRight].TrackingState;
	TrackingState wristLeftState = inputJoints[JointType::JointType_WristLeft].TrackingState;
	TrackingState wristRightState = inputJoints[JointType::JointType_WristRight].TrackingState;
	TrackingState spineShoulderState = inputJoints[JointType::JointType_SpineShoulder].TrackingState;
	TrackingState elbowLeftState = inputJoints[JointType::JointType_ElbowLeft].TrackingState;
	TrackingState elbowRightState = inputJoints[JointType::JointType_ElbowRight].TrackingState;
	TrackingState neckState = inputJoints[JointType::JointType_Neck].TrackingState;
	TrackingState spineBaseState = inputJoints[JointType::JointType_SpineBase].TrackingState;
	TrackingState hipLeftState = inputJoints[JointType::JointType_HipLeft].TrackingState;
	TrackingState hipRightState = inputJoints[JointType::JointType_HipRight].TrackingState;
	TrackingState kneeLeftState = inputJoints[JointType::JointType_KneeLeft].TrackingState;
	TrackingState kneeRightState = inputJoints[JointType::JointType_KneeRight].TrackingState;
	TrackingState headState = inputJoints[JointType::JointType_Head].TrackingState;


	if (shoulderLeftState == TrackingState::TrackingState_Tracked && elbowLeftState == TrackingState::TrackingState_Tracked) {
		extractedBodyProperties[LEFT_UPPER_ARM_LENGTH] = sqrt(pow(shoulderLeft.X - elbowLeft.X, 2.0f) +
			pow(shoulderLeft.Y - elbowLeft.Y, 2.0f) + pow(shoulderLeft.Z - elbowLeft.Z, 2.0f));
	}
	else {
		extractedBodyProperties[LEFT_UPPER_ARM_LENGTH] = 0.0f;
	}

	if (shoulderRightState == TrackingState::TrackingState_Tracked && elbowRightState == TrackingState::TrackingState_Tracked) {
		extractedBodyProperties[RIGHT_UPPER_ARM_LENGTH] = sqrt(pow(shoulderRight.X - elbowRight.X, 2.0f) +
			pow(shoulderRight.Y - elbowRight.Y, 2.0f) + pow(shoulderRight.Z - elbowRight.Z, 2.0f));
	}
	else {
		extractedBodyProperties[RIGHT_UPPER_ARM_LENGTH] = 0.0f;
	}

	if (elbowLeftState == TrackingState::TrackingState_Tracked && handLeftState == TrackingState::TrackingState_Tracked) {
		extractedBodyProperties[LEFT_LOWER_ARM_LENGTH] = sqrt(pow(handLeft.X - elbowLeft.X, 2.0f) +
			pow(handLeft.Y - elbowLeft.Y, 2.0f) + pow(handLeft.Z - elbowLeft.Z, 2.0f));
	}
	else {
		extractedBodyProperties[LEFT_LOWER_ARM_LENGTH] = 0.0f;
	}

	if (elbowRightState == TrackingState::TrackingState_Tracked && handRightState == TrackingState::TrackingState_Tracked) {
		extractedBodyProperties[RIGHT_LOWER_ARM_LENGTH] = sqrt(pow(handRight.X - elbowRight.X, 2.0f) +
			pow(handRight.Y - elbowRight.Y, 2.0f) + pow(handRight.Z - elbowRight.Z, 2.0f));
	}
	else {
		extractedBodyProperties[RIGHT_LOWER_ARM_LENGTH] = 0.0f;
	}

	if (hipLeftState == TrackingState::TrackingState_Tracked && kneeLeftState == TrackingState::TrackingState_Tracked) {
		extractedBodyProperties[LEFT_UPPER_LEG_LENGTH] = sqrt(pow(hipLeft.X - kneeLeft.X, 2.0f) +
			pow(hipLeft.Y - kneeLeft.Y, 2.0f) + pow(hipLeft.Z - kneeLeft.Z, 2.0f));
	}
	else {
		extractedBodyProperties[LEFT_UPPER_LEG_LENGTH] = 0.0f;
	}

	if (hipRightState == TrackingState::TrackingState_Tracked && kneeRightState == TrackingState::TrackingState_Tracked) {
		extractedBodyProperties[RIGHT_UPPER_LEG_LENGTH] = sqrt(pow(hipRight.X - kneeRight.X, 2.0f) +
			pow(hipRight.Y - kneeRight.Y, 2.0f) + pow(hipRight.Z - kneeRight.Z, 2.0f));
	}
	else {
		extractedBodyProperties[RIGHT_UPPER_LEG_LENGTH] = 0.0f;
	}

	if (shoulderLeftState == TrackingState::TrackingState_Tracked && shoulderRightState == TrackingState::TrackingState_Tracked) {
		extractedBodyProperties[SHOULDER_WIDTH] = sqrt(pow(shoulderLeft.X - shoulderRight.X, 2.0f) +
			pow(shoulderLeft.Y - shoulderRight.Y, 2.0f) + pow(shoulderLeft.Z - shoulderRight.Z, 2.0f));
	}
	else {
		extractedBodyProperties[SHOULDER_WIDTH] = 0.0f;
	}

	if (hipLeftState == TrackingState::TrackingState_Tracked && hipRightState == TrackingState::TrackingState_Tracked) {
		extractedBodyProperties[HIP_WIDTH] = sqrt(pow(hipLeft.X - hipRight.X, 2.0f) +
			pow(hipLeft.Y - hipRight.Y, 2.0f) + pow(hipLeft.Z - hipRight.Z, 2.0f));
	}
	else {
		extractedBodyProperties[HIP_WIDTH] = 0.0f;
	}

	if (spineShoulderState == TrackingState::TrackingState_Tracked && spineBaseState == TrackingState::TrackingState_Tracked) {
		extractedBodyProperties[TORSO_LENGTH] = sqrt(pow(spineShoulder.X - spineBase.X, 2.0f) +
			pow(spineShoulder.Y - spineBase.Y, 2.0f) + pow(spineShoulder.Z - spineBase.Z, 2.0f));
	}
	else {
		extractedBodyProperties[TORSO_LENGTH] = 0.0f;
	}

	if (neckState == TrackingState::TrackingState_Tracked && headState == TrackingState::TrackingState_Tracked) {
		extractedBodyProperties[NECK_TO_HEAD] = sqrt(pow(neck.X - head.X, 2.0f) +
			pow(neck.Y - head.Y, 2.0f) + pow(neck.Z - head.Z, 2.0f));
	}
	else {
		extractedBodyProperties[NECK_TO_HEAD] = 0.0f;
	}

	if (wristRightState == TrackingState::TrackingState_Tracked && handRightState == TrackingState::TrackingState_Tracked) {
		extractedBodyProperties[RIGHT_HAND_TO_WRIST] = sqrt(pow(wristRight.X - handRight.X, 2.0f) +
			pow(wristRight.Y - handRight.Y, 2.0f) + pow(wristRight.Z - handRight.Z, 2.0f));
	}
	else {
		extractedBodyProperties[RIGHT_HAND_TO_WRIST] = 0.0f;
	}

	if (wristLeftState == TrackingState::TrackingState_Tracked && handLeftState == TrackingState::TrackingState_Tracked) {
		extractedBodyProperties[LEFT_HAND_TO_WRIST] = sqrt(pow(wristLeft.X - handLeft.X, 2.0f) +
			pow(wristLeft.Y - handLeft.Y, 2.0f) + pow(wristLeft.Z - handLeft.Z, 2.0f));
	}
	else {
		extractedBodyProperties[LEFT_HAND_TO_WRIST] = 0.0f;
	}


#ifdef DEBUG_NOTIFY_BAD_PROPERTY
	for (int i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {
		if (extractedBodyProperties[i] == 0.0f) {
			switch (i)
			{
			case LEFT_UPPER_ARM_LENGTH:  OutputDebugStringA("SCHLECHTER WERT (MASTERFESTLEGUNG)   Linker Oberarm\n"); break;
			case RIGHT_UPPER_ARM_LENGTH: OutputDebugStringA("SCHLECHTER WERT (MASTERFESTLEGUNG)   Rechter Oberarm\n"); break;
			case LEFT_LOWER_ARM_LENGTH:  OutputDebugStringA("SCHLECHTER WERT (MASTERFESTLEGUNG)   Linker Unterarm\n"); break;
			case RIGHT_LOWER_ARM_LENGTH: OutputDebugStringA("SCHLECHTER WERT (MASTERFESTLEGUNG)   Rechter Unterarm\n"); break;
			//case LEFT_UPPER_LEG_LENGTH:  OutputDebugStringA("SCHLECHTER WERT (MASTERFESTLEGUNG)   Linker Oberschenkel\n"); break;
			//case RIGHT_UPPER_LEG_LENGTH: OutputDebugStringA("SCHLECHTER WERT (MASTERFESTLEGUNG)   Rechter Oberschenkel\n"); break;
			case SHOULDER_WIDTH:         OutputDebugStringA("SCHLECHTER WERT (MASTERFESTLEGUNG)   Schulterbreite\n"); break;
			case HIP_WIDTH:              OutputDebugStringA("SCHLECHTER WERT (MASTERFESTLEGUNG)   Hüftbreite\n"); break;
			case TORSO_LENGTH:           OutputDebugStringA("SCHLECHTER WERT (MASTERFESTLEGUNG)   Torsolänge\n"); break;
			case NECK_TO_HEAD:           OutputDebugStringA("SCHLECHTER WERT (MASTERFESTLEGUNG)   Hals zu Kopf\n"); break;
			case RIGHT_HAND_TO_WRIST:    OutputDebugStringA("SCHLECHTER WERT (MASTERFESTLEGUNG)   rechte Handlänge\n"); break;
			case LEFT_HAND_TO_WRIST:     OutputDebugStringA("SCHLECHTER WERT (MASTERFESTLEGUNG)   linke Handlänge\n"); break;

			default: break;
			}
		}
	}
#endif
}

/** extrahiert die Körperproportionen aus der der Person-Klasse zu eigenen Gelenkpunkten
* und speichert diese wiederum in einem Attribut der Person-Klasse
*/
void Person::saveBodyProperties()
{
#ifdef DEBUG_VERBOSE
	OutputDebugStringA("DEBUG: saveBodyProperties aufgerufen.\n");
#endif
	extractBodyProperties(bodyProperties, joints);
#ifdef DEBUG_VERBOSE
	OutputDebugStringA("DEBUG: Neue bodyProperties in aus joints gelesen.\n");
#endif
}

/**
* extrahiert die Körperproportionen aus den Person-eigenen Gelenken in ein neu angelegtes Array und 
* speichert einen Zeiger auf dieses Array in einem Buffer ab 
*/
void Person::collectBodyProperties()
{
#ifdef DEBUG_VERBOSE
	OutputDebugStringA("DEBUG: collectBodyProperties aufgerufen.\n");
#endif

	float* bodyPropertiesTemp = new float[NUMBER_OF_BODY_PROPERTIES];
	extractBodyProperties(bodyPropertiesTemp, joints);

	bodyPropertiesBuffer.push_back(bodyPropertiesTemp);
#ifdef DEBUG_VERBOSE
	OutputDebugStringA("DEBUG: Temporäre bodyProperties aus joints gelesen und auf Puffer gelegt.\n");
#endif

}

/*
* Berechnet aus allen im Buffer für die Körperproportionen gespeicherten Werten 
* jeweils für eine Proportion den Durchschnitt aus allen Werten für diese Proportion
* und speichert diese im eigenen bodyProperties-Array der Person-Klasse 
*/
void Person::calculateBodyProperties()
{
#ifdef DEBUG_VERBOSE
	OutputDebugStringA("DEBUG: calculateBodyProperties aufgerufen.\n");
#endif

	// Iterator um durch den Buffer (verkettete Liste aus der STL) zu iterieren
	std::list<float*>::iterator liter;
	// temporärer Zeiger auf ein Buffer-Element, welches ein Array für einen Frame alle Proportionen enthält
	float* bodyPropertiesTemp;
	// Anzahl der gültigen Samples pro Proportion (Messungen bei denen für eine Proportion einer beiden Endpunkte
	// nicht "Tracked" war, wurden auf 0.0f gesetzt und zählen somit nicht hinein)
	int numberOfSamples[NUMBER_OF_BODY_PROPERTIES];
	int i;

	float smallest_val[NUMBER_OF_BODY_PROPERTIES];
	float greatest_val[NUMBER_OF_BODY_PROPERTIES];

	// Fertig, falls Buffer leer
	if (bodyPropertiesBuffer.empty())
		return;

	for (i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {
		bodyProperties[i] = 0.0f;
		numberOfSamples[i] = 0;
		smallest_val[i] = FLT_MAX;
		greatest_val[i] = -FLT_MAX;
	}

	// Bestimmung des kleinsten und größten Werts pro Property über alle Frames, werden später gestrichen
	for (i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {
		std::list<float*>::iterator frame;
		for (frame = bodyPropertiesBuffer.begin(); frame != bodyPropertiesBuffer.end(); frame++) {
			bodyPropertiesTemp = *frame;
			if (bodyPropertiesTemp[i] < smallest_val[i]) { smallest_val[i] = bodyPropertiesTemp[i]; }
			if (bodyPropertiesTemp[i] > greatest_val[i]) { greatest_val[i] = bodyPropertiesTemp[i]; }
		}
	}

	// Gehe durch den Buffer und...
	for (liter = bodyPropertiesBuffer.begin(); liter != bodyPropertiesBuffer.end(); liter++) {
		bodyPropertiesTemp = *liter;

		// ...gehe für jedes Element einzeln durch die Proportionen und...
		for (i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {
			//... beziehe, falls der Wert ungleich 0.0f und damit gültig war, in den Durchschnitt für 
			// diese Proportion mit ein, streiche auch kleinsten und größten Wert
			
			//OutputDebugStringA(std::to_string(smallest_val[i]).c_str());
			//OutputDebugStringA("\n");
			if (bodyPropertiesTemp[i] != 0.0f && bodyPropertiesTemp[i] != smallest_val[i] && bodyPropertiesTemp[i] != greatest_val[i]) {
				bodyProperties[i] += bodyPropertiesTemp[i];
				numberOfSamples[i]++;
			}
		}
		//OutputDebugStringA("nächste Bufferposition---\n");

		// Wenn fertig mit diesem Buffer-Element lösche dieses
		//delete[] bodyPropertiesTemp;
	}

	// Berechne Durchschnitt für jede Proportion einzeln mittels Quotient aus Summe der
	// der gültigen Samples durch die Anzahl der gültigen Samples für diese Proportion
	for (i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {
		if (numberOfSamples[i] != 0 && bodyProperties[i] != 0.0f) {
			bodyProperties[i] /= numberOfSamples[i];
		}
		// Standardabweichung für jede Property berechnen
		float sum = 0; //Summe der quadratischen Abstände für Merkmal i
		for (liter = bodyPropertiesBuffer.begin(); liter != bodyPropertiesBuffer.end(); liter++) {
			bodyPropertiesTemp = *liter;
			if (bodyPropertiesTemp[i] != 0.0f && bodyPropertiesTemp[i] != smallest_val[i] && bodyPropertiesTemp[i] != greatest_val[i]) {
				sum += pow(bodyPropertiesTemp[i] - bodyProperties[i], 2);
			}
		}
		standardDeviations[i] = .0001f;
		if (numberOfSamples[i] > 1) {
			standardDeviations[i] = max(sqrt(1.0f / (numberOfSamples[i] - 1) * sum), standardDeviations[i]);
		}
#ifdef DEBUG_COLLECTING
		OutputDebugStringA("Standardabweichung, ");
		switch (i)
		{
		case LEFT_UPPER_ARM_LENGTH:  OutputDebugStringA("Linker Oberarm:        "); break;
		case RIGHT_UPPER_ARM_LENGTH: OutputDebugStringA("Rechter Oberarm:       "); break;
		case LEFT_LOWER_ARM_LENGTH:  OutputDebugStringA("Linker Unterarm:       "); break;
		case RIGHT_LOWER_ARM_LENGTH: OutputDebugStringA("Rechter Unterarm:      "); break;
		//case LEFT_UPPER_LEG_LENGTH:  OutputDebugStringA("Linker Oberschenkel:   "); break;
		//case RIGHT_UPPER_LEG_LENGTH: OutputDebugStringA("Rechter Oberschenkel:  "); break;
		case SHOULDER_WIDTH:         OutputDebugStringA("Schulterbreite:        "); break;
		case HIP_WIDTH:              OutputDebugStringA("Hüftbreite:            "); break;
		case TORSO_LENGTH:           OutputDebugStringA("Torsolänge:            "); break;
		case NECK_TO_HEAD:           OutputDebugStringA("Hals zu Kopf:            "); break;
		case RIGHT_HAND_TO_WRIST:    OutputDebugStringA("rechte Handlänge:            "); break;
		case LEFT_HAND_TO_WRIST:     OutputDebugStringA("linke Handlänge:            "); break;
		
		default: break;
		}
		OutputDebugStringA(std::to_string(standardDeviations[i]).c_str());
		OutputDebugStringA("\t");
		if (numberOfSamples[i] != 0 && bodyProperties[i] != 0.0f) {
			OutputDebugStringA("Mittel ");
			OutputDebugStringA(std::to_string(bodyProperties[i]).c_str());
			OutputDebugStringA(" über ");
			OutputDebugStringA(std::to_string(numberOfSamples[i]).c_str());
			OutputDebugStringA(" Samples.\n");
		}

#endif
	}
	bodyPropertiesBuffer.clear();

#ifdef DEBUG_VERBOSE
	OutputDebugStringA("DEBUG: Mittel der Samples und Standardabweichung bestimmt.\n");
#endif
}


/** Extrahiert aus den übergebenen Gelenkpunkten (einer zu vergleichenden Person)
* die Körperproportionen und vergleicht diese mit den eigenen (der der Person Klasse)
*
* Hierbei wird zunächst für jede einzelne Proportion ein Wert (die Konfidenz) zwischen 0.0f und 1.0f
* berechnet der umso grösser sein soll je ähnlicher der aus den übergegebenen 
* Gelenken berechnete Wert dem gespeicherten Wert für diese Proportion ist
* Über alle Proportionen wird schließlich ein gewichteter Durchschnitt der Konfidenzen erstellt und zurück-
* gegeben, wobei die Gewichtung einer Körperproportion davon abhängt, wie weit 
* die aus den übergebenen Gelenken berechneten Proportionen von denen im Konstruktor
* festgelegten Min-Max-Grenzen abweicht (falls der Wert innerhalb der Grenzen liegt ist die Gewichtung 1.0f;
* je weiter ausserhalb er liegt desto geringer ist die Gewichtung)
*
*@param inputJoints Gelenke der zu vergleichenden Person
*@return float gibt einen Konfidenzwert zwischen 0.0f und 1.0f der umso größer ist
* je ähnlicher sich die Körperproportionen der übergebenen Gelenkpunkte und die eigenen
* Proportionen sind
*/

float Person::compareBodyProperties(Joint* inputJoints) {
	float propertiesForComparison[NUMBER_OF_BODY_PROPERTIES];
	extractBodyProperties(propertiesForComparison, inputJoints);
	float deviation;
	int weightIndex;

	float normalizationFactor = 0.0f;
	double accumulatedError = 0.0; 

	for (int i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {

		// schaue nach inwie weit der gemesse Wert für diese Proportion von den im Konstruktor
		// festgelegten "sinnvollen" Min-Max-Grenzen liegt (falls dieser ausserhalb liegt, ist von
		// einem Messfehler auszugehen) und speichere die Abweichung von den Grenzen in der deviation Variable
		if (propertiesForComparison[i] < bodyPropertiesLimits[i].min) {
			deviation = bodyPropertiesLimits[i].min - propertiesForComparison[i];
		}
		else if (propertiesForComparison[i] <= bodyPropertiesLimits[i].max) {
			deviation = 0.0f;
		}
		else {
			deviation = propertiesForComparison[i] - bodyPropertiesLimits[i].max;
		}

		// Entscheide welches Gewicht für diese Proportion zu verwenden ist
		// (Es ist davon auszugehen das am Index 0 das Gewicht 1.0f steht und je höher der Index desto kleiner
		// das Gewicht; Die Abweichung wird mit 10.0f multipliziert, um den Index zu erhalten, wodurch
		// das Gewicht 1.0f ist, falls der Wert innerhalb der Grenzen lag und pro 10cm Abweichung
		// von den Grenzen das Gewicht um eine Stufeim Gewichte-Array abnimmt)
		weightIndex = static_cast<int>(deviation*10.0f);
		//OutputDebugStringA(std::to_string(weightIndex).c_str());

		// Falls der berechnete Index für den Gewichte-Array die Größe des Arrays übersteigt
		// verwende höchsten Index an dessen Stelle für gewöhnlich das Gewicht 0.0f steht
		if (weightIndex >= numberOfWeights) {
			weightIndex = numberOfWeights - 1;
		}

		// Prüfe, ob entweder die Person-eigene Proportion oder die übergebene ungültig war
		// Falls ja setze das Gewicht auf das kleinstmögliche (für gewöhnlich 0.0f)
		if (bodyProperties[i] == 0.0f || propertiesForComparison[i] == 0.0f) {
			weightIndex = numberOfWeights - 1;
		}

		normalizationFactor += bodyPropertiesWeights[weightIndex] * bodyPropertiesFactors[i] / standardDeviations[i];

		// Quadratische Abweichung, normiert, gewichtet mit Glaubwürdigkeitsfaktor und 1/Standardabweichung
		if (bodyProperties[i] != 0.0f) {
			accumulatedError += pow((double) bodyProperties[i] - (double) propertiesForComparison[i], 2) / (double) bodyProperties[i]
				* (double) bodyPropertiesWeights[weightIndex] * (double) bodyPropertiesFactors[i] / (double) standardDeviations[i];
		}

		//Debugausgaben Körpermerkmale
		#ifdef DEBUG_BODY_PROPERTIES
			switch (i)
			{
			case LEFT_UPPER_ARM_LENGTH:  OutputDebugStringA("Linker Oberarm:        "); break;
			case RIGHT_UPPER_ARM_LENGTH: OutputDebugStringA("Rechter Oberarm:       "); break;
			case LEFT_LOWER_ARM_LENGTH:  OutputDebugStringA("Linker Unterarm:       "); break;
			case RIGHT_LOWER_ARM_LENGTH: OutputDebugStringA("Rechter Unterarm:      "); break;
			//case LEFT_UPPER_LEG_LENGTH:  OutputDebugStringA("Linker Oberschenkel:   "); break;
			//case RIGHT_UPPER_LEG_LENGTH: OutputDebugStringA("Rechter Oberschenkel:  "); break;
			case SHOULDER_WIDTH:         OutputDebugStringA("Schulterbreite:        "); break;
			case HIP_WIDTH:              OutputDebugStringA("Hüftbreite:            "); break;
			case TORSO_LENGTH:           OutputDebugStringA("Torsolänge:            "); break;
			case NECK_TO_HEAD:           OutputDebugStringA("Hals zu Kopf:            "); break;
			case RIGHT_HAND_TO_WRIST:    OutputDebugStringA("rechte Handlänge:            "); break;
			case LEFT_HAND_TO_WRIST:     OutputDebugStringA("linke Handlänge:            "); break;

			}
			
			if (i != LEFT_UPPER_LEG_LENGTH && i != RIGHT_UPPER_LEG_LENGTH) {
				OutputDebugStringA("eingespeichert = ");
				OutputDebugStringA(std::to_string(bodyProperties[i]).c_str());
				OutputDebugStringA("\t\t");

				OutputDebugStringA("neu = ");
				OutputDebugStringA(std::to_string(propertiesForComparison[i]).c_str());
				OutputDebugStringA("\t\t");

				OutputDebugStringA("Konfidenz = ");
				OutputDebugStringA(std::to_string(min(bodyProperties[i] / propertiesForComparison[i], propertiesForComparison[i] / bodyProperties[i])).c_str());
				OutputDebugStringA("\n");
			}
		#endif

		//Debugbenachrichtigungen für schlechte Werte
		#ifdef DEBUG_NOTIFY_BAD_PROPERTY
			if(bodyProperties[i] == 0.0f) {
				switch (i)
				{
				case LEFT_UPPER_ARM_LENGTH:  OutputDebugStringA("SCHLECHTER WERT  Linker Oberarm\n"); break;
				case RIGHT_UPPER_ARM_LENGTH: OutputDebugStringA("SCHLECHTER WERT  Rechter Oberarm\n"); break;
				case LEFT_LOWER_ARM_LENGTH:  OutputDebugStringA("SCHLECHTER WERT  Linker Unterarm\n"); break;
				case RIGHT_LOWER_ARM_LENGTH: OutputDebugStringA("SCHLECHTER WERT  Rechter Unterarm\n"); break;
				//case LEFT_UPPER_LEG_LENGTH:  OutputDebugStringA("SCHLECHTER WERT  Linker Oberschenkel\n"); break;
				//case RIGHT_UPPER_LEG_LENGTH: OutputDebugStringA("SCHLECHTER WERT  Rechter Oberschenkel\n"); break;
				case SHOULDER_WIDTH:         OutputDebugStringA("SCHLECHTER WERT  Schulterbreite\n"); break;
				case HIP_WIDTH:              OutputDebugStringA("SCHLECHTER WERT  Hüftbreite\n"); break;
				case TORSO_LENGTH:           OutputDebugStringA("SCHLECHTER WERT  Torsolänge\n"); break;
				case NECK_TO_HEAD:         OutputDebugStringA("SCHLECHTER WERT  Hals zu Kopf\n"); break;
				case RIGHT_HAND_TO_WRIST:         OutputDebugStringA("SCHLECHTER WERT  rechte Handlänge\n"); break;
				case LEFT_HAND_TO_WRIST:         OutputDebugStringA("SCHLECHTER WERT  linke Handlänge\n"); break;
				default: break;
				}
			}
		#endif
	}
	if (normalizationFactor != 0.0f)
		accumulatedError = 100000.0f * accumulatedError / normalizationFactor;
	else
		accumulatedError = 0.0f;

//Debugmeldung über Oberarmlängendifferenz
#ifdef DEBUG_ARM_DIFFERENCE
	OutputDebugStringA("Messunterschied Oberarmlänge:      ");
	OutputDebugStringA(std::to_string(abs(bodyProperties[LEFT_UPPER_ARM_LENGTH]-bodyProperties[RIGHT_UPPER_ARM_LENGTH])).c_str());
	OutputDebugStringA("\t");
	OutputDebugStringA("Mittel: ");
	OutputDebugStringA(std::to_string((bodyProperties[LEFT_UPPER_ARM_LENGTH] + bodyProperties[RIGHT_UPPER_ARM_LENGTH])/2.0f).c_str());
	OutputDebugStringA("\n");
#endif

//Debugmeldung über Schenkellängendifferenz
#ifdef DEBUG_LEG_DIFFERENCE
	OutputDebugStringA("Messunterschied Oberschenkellänge: ");
	OutputDebugStringA(std::to_string(abs(bodyProperties[LEFT_UPPER_LEG_LENGTH] - bodyProperties[RIGHT_UPPER_LEG_LENGTH])).c_str());
	OutputDebugStringA("\n");
	OutputDebugStringA("Mittel: ");
	OutputDebugStringA(std::to_string((bodyProperties[LEFT_UPPER_LEG_LENGTH] + bodyProperties[RIGHT_UPPER_LEG_LENGTH]) / 2.0f).c_str());
	OutputDebugStringA("\n");
#endif

//Debugmeldung über errechnete Abweichung
#ifdef DEBUG_ACCUMULATED_ERROR
	OutputDebugStringA("Berechnete Abweichung: ");
	OutputDebugStringA(std::to_string(accumulatedError).c_str());
	OutputDebugStringA("\n\n");
#endif

	return accumulatedError;
}

/**
* Prüft, ob die Person die vordefinierte Konfigurationspose eingenommen hat
*/

bool Person::isInConfigurationPose()
{
	return (abs(joints[JointType::JointType_HandLeft].Position.Y - joints[JointType::JointType_HandRight].Position.Y) < .1f) // Hände etwa auf gleicher Höhe
		&& (abs(joints[JointType::JointType_HandLeft].Position.Z - joints[JointType::JointType_HandRight].Position.Z) < .1f) // Hände etwa in gleicher Entfernung
		&& (abs(joints[JointType::JointType_HipLeft].Position.Z - joints[JointType::JointType_HipRight].Position.Z) < .1f) // Hüfte nicht verdreht (gerade vor der Kamera)
		&& (abs(joints[JointType::JointType_HandLeft].Position.Y - joints[JointType::JointType_HipLeft].Position.Y) < .15f)
		&& (abs(joints[JointType::JointType_HandRight].Position.Y - joints[JointType::JointType_HipRight].Position.Y) < .15f) // Hände ungefähr auf Hüfthöhe
		&& (abs(joints[JointType::JointType_HandLeft].Position.Z - joints[JointType::JointType_HipLeft].Position.Z) < .1f)
		&& (abs(joints[JointType::JointType_HandRight].Position.Z - joints[JointType::JointType_HipRight].Position.Z) < .1f) // Hände ungefähr auf Hüftentfernung
		&& (abs(joints[JointType::JointType_HandLeft].Position.X - joints[JointType::JointType_HandRight].Position.X)
			< 4.0f * abs(joints[JointType::JointType_HipLeft].Position.X - joints[JointType::JointType_HipRight].Position.X)) // Handabstand < 4*Hüftabstand (x-Werte)
		&& (abs(joints[JointType::JointType_HandLeft].Position.X - joints[JointType::JointType_HandRight].Position.X)
			> 2.0f * abs(joints[JointType::JointType_HipLeft].Position.X - joints[JointType::JointType_HipRight].Position.X)) // Handabstand > 2*Hüftabstand (x-Werte)
		&& (joints[JointType::JointType_HandLeft].Position.X < joints[JointType::JointType_HandRight].Position.X)
		&& (joints[JointType::JointType_HipLeft].Position.X < joints[JointType::JointType_HipRight].Position.X); // richtig herum vor der Kamera
}
