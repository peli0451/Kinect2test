#include "stdafx.h"
#include "Person.h"
#include <cmath>

/**********************************************************
* Debug-Schalter
**********************************************************/

//#define DEBUG_BODY_PROPERTIES			//Debugausgaben K�rpermerkmale
//#define DEBUG_ARM_DIFFERENCE			//Debugmeldung �ber Oberarml�ngendifferenz
//#define DEBUG_LEG_DIFFERENCE			//Debugmeldung �ber Schenkell�ngendifferenz
//#define DEBUG_ACCUMULATED_ERROR		//Debugmeldung �ber errechnete Abweichung
//#define DEBUG_NOTIFY_BAD_PROPERTY		//Debugbenachrichtigungen f�r schlechte Werte
#define DEBUG_COLLECTING				//Debugmeldung �ber Standardabweichung bei Masterfestlegung
//#define DEBUG_VERBOSE					//Debugmeldungen �ber Funktionsaufrufe und berechnungen
//#define DEBUG_MASTER					//Debugmeldung, ob Master erkannt

/**********************************************************
* Konstruktoren
**********************************************************/

Person::Person()
{
	id = -1;
	trackingId = -1;

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

	// Grenzen (Minimum und Maximum) f�r jede einzelne K�rperproportion festlegen,
	// innerhalb derer sich die "vern�nftigen" Werte f�r eine Proportion befinden sollten,
	// falls der Wert nicht durch einen Messfehler auf einen unglaubw�rdig zu kleinen oder zu
	// grossen Wert springt
	bodyPropertiesLimits[LEFT_UPPER_ARM_LENGTH].min = 0.1f;
	bodyPropertiesLimits[LEFT_UPPER_ARM_LENGTH].max = 0.6f;
	bodyPropertiesLimits[RIGHT_UPPER_ARM_LENGTH].min = bodyPropertiesLimits[LEFT_UPPER_ARM_LENGTH].min;
	bodyPropertiesLimits[RIGHT_UPPER_ARM_LENGTH].max = bodyPropertiesLimits[LEFT_UPPER_ARM_LENGTH].max;
	bodyPropertiesLimits[LEFT_LOWER_ARM_LENGTH].min = 0.1f;
	bodyPropertiesLimits[LEFT_LOWER_ARM_LENGTH].max = 0.6f;
	bodyPropertiesLimits[RIGHT_LOWER_ARM_LENGTH].min = bodyPropertiesLimits[LEFT_UPPER_ARM_LENGTH].min;
	bodyPropertiesLimits[RIGHT_LOWER_ARM_LENGTH].max = bodyPropertiesLimits[LEFT_UPPER_ARM_LENGTH].max;
	bodyPropertiesLimits[SHOULDER_WIDTH].min = 0.2f;
	bodyPropertiesLimits[SHOULDER_WIDTH].max = 0.7f;
	bodyPropertiesLimits[HIP_WIDTH].min = 0.2f;
	bodyPropertiesLimits[HIP_WIDTH].max = 0.7f;
	bodyPropertiesLimits[TORSO_LENGTH].min = 0.2f;
	bodyPropertiesLimits[TORSO_LENGTH].max = 1.0f;
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


void Person::setTrackingId(UINT64 newTId) {
	trackingId = newTId;
}

UINT64 Person::getTrackingId() {
	return trackingId;
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

bool Person::isTracked(float* testBodyProperties)
{
	for (int i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {
		if (testBodyProperties[i] == 0.0f) {
			return false;
		}
	}

	return true;
}

/** Berechnet aus den �bergebenen Gelenkpunkten durch Ermittlung der euklidische Distanz
* zwischen zwei Punkten die K�rperproportionen gem�� dem BODY_PROPERTIES enum und speichert
* diese in einem Array (jeder Eintrag im Array entspricht einer Konstante aus dem genannten enum)
*
* Die errechnete Distanz wird hierbei auf 0.0f gesetzt, falls einer der beiden Endpunkte
* der Distanz nicht den TrackingState "Tracked" haben (also "NotTracked" oder "Inferred" sind)
* und somit f�r unsere Zwecke nicht von der Kinect mit gen�gend gro�er Konfidenz erkannt werden
* bsplsweise: Berechnung der linken Oberarml�nge aus der Distanz zwischen den Gelenkpunkten
* von linker Schulter und linkem Elbogen
*
* @param extractedBodyProperties Zeiger auf den Array in dem die K�rperproportionen gespeichert werden
* @param inputJoints Zeiger auf den Array mit den Gelenkpunkten
* @return gibt false zur�ck falls einer der relevanten Gelenkpunkte nicht getrackt war
*/
bool Person::extractBodyProperties(float* extractedBodyProperties, Joint* inputJoints)
{
#ifdef DEBUG_VERBOSE
	OutputDebugStringA("DEBUG: extractBodyProperties aufgerufen.\n");
#endif
	_CameraSpacePoint shoulderLeft = inputJoints[JointType::JointType_ShoulderLeft].Position;
	_CameraSpacePoint shoulderRight = inputJoints[JointType::JointType_ShoulderRight].Position;
	_CameraSpacePoint handLeft = inputJoints[JointType::JointType_HandLeft].Position;
	_CameraSpacePoint handRight = inputJoints[JointType::JointType_HandRight].Position;
	_CameraSpacePoint spineShoulder = inputJoints[JointType::JointType_SpineShoulder].Position;
	_CameraSpacePoint elbowLeft = inputJoints[JointType::JointType_ElbowLeft].Position;
	_CameraSpacePoint elbowRight = inputJoints[JointType::JointType_ElbowRight].Position;
	_CameraSpacePoint neck = inputJoints[JointType::JointType_Neck].Position;
	_CameraSpacePoint spineBase = inputJoints[JointType::JointType_SpineBase].Position;
	_CameraSpacePoint hipLeft = inputJoints[JointType::JointType_HipLeft].Position;
	_CameraSpacePoint hipRight = inputJoints[JointType::JointType_HipRight].Position;
	_CameraSpacePoint head = inputJoints[JointType::JointType_Head].Position;

	TrackingState shoulderLeftState = inputJoints[JointType::JointType_ShoulderLeft].TrackingState;
	TrackingState shoulderRightState = inputJoints[JointType::JointType_ShoulderRight].TrackingState;
	TrackingState handLeftState = inputJoints[JointType::JointType_HandLeft].TrackingState;
	TrackingState handRightState = inputJoints[JointType::JointType_HandRight].TrackingState;
	TrackingState spineShoulderState = inputJoints[JointType::JointType_SpineShoulder].TrackingState;
	TrackingState elbowLeftState = inputJoints[JointType::JointType_ElbowLeft].TrackingState;
	TrackingState elbowRightState = inputJoints[JointType::JointType_ElbowRight].TrackingState;
	TrackingState neckState = inputJoints[JointType::JointType_Neck].TrackingState;
	TrackingState spineBaseState = inputJoints[JointType::JointType_SpineBase].TrackingState;
	TrackingState hipLeftState = inputJoints[JointType::JointType_HipLeft].TrackingState;
	TrackingState hipRightState = inputJoints[JointType::JointType_HipRight].TrackingState;
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
			case HIP_WIDTH:              OutputDebugStringA("SCHLECHTER WERT (MASTERFESTLEGUNG)   H�ftbreite\n"); break;
			case TORSO_LENGTH:           OutputDebugStringA("SCHLECHTER WERT (MASTERFESTLEGUNG)   Torsol�nge\n"); break;
			case NECK_TO_HEAD:           OutputDebugStringA("SCHLECHTER WERT (MASTERFESTLEGUNG)   Hals zu Kopf\n"); break;

			default: break;
			}
		}
	}
#endif

	return isTracked (extractedBodyProperties);
}

/** extrahiert die K�rperproportionen aus der der Person-Klasse zu eigenen Gelenkpunkten
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
* extrahiert die K�rperproportionen aus den Person-eigenen Gelenken in ein neu angelegtes Array und 
* speichert einen Zeiger auf dieses Array in einem Buffer ab 
*/
bool Person::collectBodyProperties()
{
#ifdef DEBUG_VERBOSE
	OutputDebugStringA("DEBUG: collectBodyProperties aufgerufen.\n");
#endif
	float* bodyPropertiesTemp = new float[NUMBER_OF_BODY_PROPERTIES];
	bool isTracked = extractBodyProperties(bodyPropertiesTemp, joints);

	if (isTracked) {
		bodyPropertiesBuffer.push_back(bodyPropertiesTemp);
	}
	else {
		delete[] bodyPropertiesTemp;
	}
#ifdef DEBUG_VERBOSE
	OutputDebugStringA("DEBUG: Tempor�re bodyProperties aus joints gelesen und auf Puffer gelegt.\n");
#endif

	return isTracked;
}

/**
* L�scht den Buffer f�r die K�rperproportionen, der durch collectBodyProperties gef�llt wird
*/
void Person::deleteCollectedBodyProperties()
{
	std::list<float*>::iterator liter;
	float* bodyPropertiesTemp;

	if (bodyPropertiesBuffer.empty())
		return;

	for (liter = bodyPropertiesBuffer.begin(); liter != bodyPropertiesBuffer.end(); liter++) {
		bodyPropertiesTemp = *liter;
		delete[] bodyPropertiesTemp;
	}

	bodyPropertiesBuffer.clear();
}

/*
* Berechnet aus allen im Buffer f�r die K�rperproportionen gespeicherten Werten 
* jeweils f�r eine Proportion den Durchschnitt aus allen Werten f�r diese Proportion
* und speichert diese im eigenen bodyProperties-Array der Person-Klasse 
* return gibt false zur�ck, falls einer der berechneten K�rperproportionen durchgehend inferred war (oder Puffer leer)
*/
bool Person::calculateBodyProperties()
{
#ifdef DEBUG_VERBOSE
	OutputDebugStringA("DEBUG: calculateBodyProperties aufgerufen.\n");
#endif

	// Iterator um durch den Buffer (verkettete Liste aus der STL) zu iterieren
	std::list<float*>::iterator liter;
	// tempor�rer Zeiger auf ein Buffer-Element, welches ein Array f�r einen Frame alle Proportionen enth�lt
	float* bodyPropertiesTemp;
	// Anzahl der g�ltigen Samples pro Proportion (Messungen bei denen f�r eine Proportion einer beiden Endpunkte
	// nicht "Tracked" war, wurden auf 0.0f gesetzt und z�hlen somit nicht hinein)
	int numberOfSamples[NUMBER_OF_BODY_PROPERTIES];
	int i;

	// Fertig, falls Buffer leer
	if (bodyPropertiesBuffer.empty())
		return false;

	for (i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {
		bodyProperties[i] = 0.0f;
		numberOfSamples[i] = 0;
	}

	// Gehe durch den Buffer und...
	for (liter = bodyPropertiesBuffer.begin(); liter != bodyPropertiesBuffer.end(); liter++) {
		bodyPropertiesTemp = *liter;

		// ...gehe f�r jedes Element einzeln durch die Proportionen und...
		for (i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {
			//... beziehe, falls der Wert ungleich 0.0f und damit g�ltig war, in den Durchschnitt f�r 
			// diese Proportion mit ein, streiche auch kleinsten und gr��ten Wert
			
			if (bodyPropertiesTemp[i] != 0.0f) {
				bodyProperties[i] += bodyPropertiesTemp[i];
				numberOfSamples[i]++;
			}
		}
		//OutputDebugStringA("n�chste Bufferposition---\n");

		// Wenn fertig mit diesem Buffer-Element l�sche dieses
		//delete[] bodyPropertiesTemp;
	}

	// Berechne Durchschnitt f�r jede Proportion einzeln mittels Quotient aus Summe der
	// der g�ltigen Samples durch die Anzahl der g�ltigen Samples f�r diese Proportion
	for (i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {
		if (numberOfSamples[i] != 0 && bodyProperties[i] != 0.0f) {
			bodyProperties[i] /= numberOfSamples[i];
		}
		// Standardabweichung f�r jede Property berechnen
		float sum = 0; //Summe der quadratischen Abst�nde f�r Merkmal i
		for (liter = bodyPropertiesBuffer.begin(); liter != bodyPropertiesBuffer.end(); liter++) {
			bodyPropertiesTemp = *liter;
			if (bodyPropertiesTemp[i] != 0.0f) {
				sum += pow(bodyPropertiesTemp[i] - bodyProperties[i], 2);
			}
		}
		standardDeviations[i] = .0001f;
		if (numberOfSamples[i] > 1) {
			standardDeviations[i] = max(sqrt(1.0f / (numberOfSamples[i] - 1) * sum), standardDeviations[i]);
			//standardDeviations[i] += 0.02f;
		}

	}
	// filtere alle Werte aus den Puffern, die au�erhalb der PERMITTED_QUANTIL-ten Standardabweichung liegen
	for (i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {
		numberOfSamples[i] = 0;
		float bodyPropertyMiddle = bodyProperties[i];
		bodyProperties[i] = 0;
		for (liter = bodyPropertiesBuffer.begin(); liter != bodyPropertiesBuffer.end(); liter++) {
			bodyPropertiesTemp = *liter;
			
			if (bodyPropertiesTemp[i] != 0.0f 
				&& bodyPropertiesTemp[i] < bodyPropertyMiddle + standardDeviations[i] * PERMITTED_QUANTIL
				&& bodyPropertiesTemp[i] > bodyPropertyMiddle - standardDeviations[i] * PERMITTED_QUANTIL) {

				bodyProperties[i] += bodyPropertiesTemp[i];
				numberOfSamples[i]++;
			}
		}

		OutputDebugStringA(std::to_string(numberOfSamples[i]).c_str());
		OutputDebugStringA("\t");
		// bestimme wieder den Mittelwert �ber die gesammelten Werte
		if (numberOfSamples[i] != 0 && bodyProperties[i] != 0.0f) {
			bodyProperties[i] /= numberOfSamples[i];
		}
	}
	OutputDebugStringA("\n");
#ifdef DEBUG_COLLECTING
	for (i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {
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
		case HIP_WIDTH:              OutputDebugStringA("H�ftbreite:            "); break;
		case TORSO_LENGTH:           OutputDebugStringA("Torsol�nge:            "); break;
		case NECK_TO_HEAD:           OutputDebugStringA("Hals zu Kopf:          "); break;

		default: break;
		}
		OutputDebugStringA(std::to_string(standardDeviations[i]).c_str());
		OutputDebugStringA("\t");
		if (numberOfSamples[i] != 0 && bodyProperties[i] != 0.0f) {
			OutputDebugStringA("Mittel ");
			OutputDebugStringA(std::to_string(bodyProperties[i]).c_str());
			OutputDebugStringA(" �ber ");
			OutputDebugStringA(std::to_string(numberOfSamples[i]).c_str());
			OutputDebugStringA(" Samples.\n");
		}
	}
#endif

	bodyPropertiesBuffer.clear();

#ifdef DEBUG_VERBOSE
	OutputDebugStringA("DEBUG: Mittel der Samples und Standardabweichung bestimmt.\n");
#endif

	return isTracked (bodyProperties);
}


/** Extrahiert aus den �bergebenen Gelenkpunkten (einer zu vergleichenden Person)
* die K�rperproportionen und vergleicht diese mit den eigenen (der der Person Klasse)
*
* Hierbei wird zun�chst f�r jede einzelne Proportion ein Wert (die Konfidenz) zwischen 0.0f und 1.0f
* berechnet der umso gr�sser sein soll je �hnlicher der aus den �bergegebenen 
* Gelenken berechnete Wert dem gespeicherten Wert f�r diese Proportion ist
* �ber alle Proportionen wird schlie�lich ein gewichteter Durchschnitt der Konfidenzen erstellt und zur�ck-
* gegeben, wobei die Gewichtung einer K�rperproportion davon abh�ngt, wie weit 
* die aus den �bergebenen Gelenken berechneten Proportionen von denen im Konstruktor
* festgelegten Min-Max-Grenzen abweicht (falls der Wert innerhalb der Grenzen liegt ist die Gewichtung 1.0f;
* je weiter ausserhalb er liegt desto geringer ist die Gewichtung)
*
*@param inputJoints Gelenke der zu vergleichenden Person
*@return float gibt durchschnittliche Abweichung der K�rperproportionen an
*/

float Person::compareBodyProperties(Joint* inputJoints) {
	float propertiesForComparison[NUMBER_OF_BODY_PROPERTIES];
	float deviation;
	int weightIndex;

	float normalizationFactor = 0.0f;
	double accumulatedError = 0.0; 

	if (extractBodyProperties(propertiesForComparison, inputJoints) == false)
		return FLT_MAX;

	for (int i = 0; i < NUMBER_OF_BODY_PROPERTIES; i++) {

		// schaue nach inwie weit der gemesse Wert f�r diese Proportion von den im Konstruktor
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

		// Entscheide welches Gewicht f�r diese Proportion zu verwenden ist
		// (Es ist davon auszugehen das am Index 0 das Gewicht 1.0f steht und je h�her der Index desto kleiner
		// das Gewicht; Die Abweichung wird mit 10.0f multipliziert, um den Index zu erhalten, wodurch
		// das Gewicht 1.0f ist, falls der Wert innerhalb der Grenzen lag und pro 10cm Abweichung
		// von den Grenzen das Gewicht um eine Stufeim Gewichte-Array abnimmt)
		weightIndex = static_cast<int>(deviation*10.0f);
		//OutputDebugStringA(std::to_string(weightIndex).c_str());

		// Falls der berechnete Index f�r den Gewichte-Array die Gr��e des Arrays �bersteigt
		// verwende h�chsten Index an dessen Stelle f�r gew�hnlich das Gewicht 0.0f steht
		if (weightIndex >= numberOfWeights) {
			weightIndex = numberOfWeights - 1;
		}

		// Pr�fe, ob entweder die Person-eigene Proportion oder die �bergebene ung�ltig war
		// Falls ja setze das Gewicht auf das kleinstm�gliche (f�r gew�hnlich 0.0f)
		if (bodyProperties[i] == 0.0f || propertiesForComparison[i] == 0.0f) {
			weightIndex = numberOfWeights - 1;
		}

		normalizationFactor += bodyPropertiesWeights[weightIndex] * bodyPropertiesFactors[i];

		// Quadratische Abweichung, normiert, gewichtet mit Glaubw�rdigkeitsfaktor und 1/Standardabweichung
		if (bodyProperties[i] != 0.0f) {
			accumulatedError += pow((double) bodyProperties[i] - (double) propertiesForComparison[i], 2) / (double) bodyProperties[i]
				* (double) bodyPropertiesWeights[weightIndex] * (double) bodyPropertiesFactors[i];
		}

		//Debugausgaben K�rpermerkmale
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
			case HIP_WIDTH:              OutputDebugStringA("H�ftbreite:            "); break;
			case TORSO_LENGTH:           OutputDebugStringA("Torsol�nge:            "); break;
			case NECK_TO_HEAD:           OutputDebugStringA("Hals zu Kopf:          "); break;

			}
			
			
				OutputDebugStringA("eingespeichert = ");
				OutputDebugStringA(std::to_string(bodyProperties[i]).c_str());
				OutputDebugStringA("\t\t");

				OutputDebugStringA("neu = ");
				OutputDebugStringA(std::to_string(propertiesForComparison[i]).c_str());
				OutputDebugStringA("\t\t");

				OutputDebugStringA("Konfidenz = ");
				OutputDebugStringA(std::to_string(min(bodyProperties[i] / propertiesForComparison[i], propertiesForComparison[i] / bodyProperties[i])).c_str());
				OutputDebugStringA("\n");
			
		#endif

		//Debugbenachrichtigungen f�r schlechte Werte
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
				case HIP_WIDTH:              OutputDebugStringA("SCHLECHTER WERT  H�ftbreite\n"); break;
				case TORSO_LENGTH:           OutputDebugStringA("SCHLECHTER WERT  Torsol�nge\n"); break;
				case NECK_TO_HEAD:         OutputDebugStringA("SCHLECHTER WERT  Hals zu Kopf\n"); break;
				default: break;
				}
			}
		#endif
	}
	if (normalizationFactor != 0.0f)
		accumulatedError = 100000.0f * accumulatedError / normalizationFactor;
	else
		accumulatedError = 0.0f;

//Debugmeldung �ber Oberarml�ngendifferenz
#ifdef DEBUG_ARM_DIFFERENCE
	OutputDebugStringA("Messunterschied Oberarml�nge:      ");
	OutputDebugStringA(std::to_string(abs(bodyProperties[LEFT_UPPER_ARM_LENGTH]-bodyProperties[RIGHT_UPPER_ARM_LENGTH])).c_str());
	OutputDebugStringA("\t");
	OutputDebugStringA("Mittel: ");
	OutputDebugStringA(std::to_string((bodyProperties[LEFT_UPPER_ARM_LENGTH] + bodyProperties[RIGHT_UPPER_ARM_LENGTH])/2.0f).c_str());
	OutputDebugStringA("\n");
#endif

//Debugmeldung �ber Schenkell�ngendifferenz
#ifdef DEBUG_LEG_DIFFERENCE
	OutputDebugStringA("Messunterschied Oberschenkell�nge: ");
	OutputDebugStringA(std::to_string(abs(bodyProperties[LEFT_UPPER_LEG_LENGTH] - bodyProperties[RIGHT_UPPER_LEG_LENGTH])).c_str());
	OutputDebugStringA("\n");
	OutputDebugStringA("Mittel: ");
	OutputDebugStringA(std::to_string((bodyProperties[LEFT_UPPER_LEG_LENGTH] + bodyProperties[RIGHT_UPPER_LEG_LENGTH]) / 2.0f).c_str());
	OutputDebugStringA("\n");
#endif

//Debugmeldung �ber errechnete Abweichung
#ifdef DEBUG_ACCUMULATED_ERROR
	OutputDebugStringA("Berechnete Abweichung: ");
	OutputDebugStringA(std::to_string(accumulatedError).c_str());
	OutputDebugStringA("\n\n");
#endif
#ifdef DEBUG_MASTER
	if (accumulatedError < 50.0f) {
		OutputDebugStringA("MASTER ERKANNT\n");
	}
	else {
		OutputDebugStringA("--------------\n");
	}
#endif

	return (float) accumulatedError;
}


/**
* Leert alle wichtigen Puffer
*/
void Person::resetMotionBuffers() {
	// Handpositionenbuffer
	leftHandPositionBuffer->empty();
	rightHandPositionBuffer->empty();

	// Rotationenbuffer
	rotationBuffer->empty();
}