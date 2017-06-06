#include "stdafx.h"
#include "Person.h"
#include <cmath>

//#define DEBUG_BODY_PROPERTIES

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
	bodyPropertiesLimits[HEIGHT_OF_HEAD].min = 1.0f;
	bodyPropertiesLimits[HEIGHT_OF_HEAD].max = 2.5f;

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
	_CameraSpacePoint kneeLeft = inputJoints[JointType::JointType_KneeLeft].Position;
	_CameraSpacePoint kneeRight = inputJoints[JointType::JointType_KneeRight].Position;
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

	if (headState == TrackingState::TrackingState_Tracked) {
		extractedBodyProperties[HEIGHT_OF_HEAD] = head.Y;
	}
	else {
		extractedBodyProperties[HEIGHT_OF_HEAD] = 0.0f;
	}
}

/** extrahiert die Körperproportionen aus der der Person-Klasse zu eigenen Gelenkpunkten
* und speichert diese wiederum in einem Attribut der Person-Klasse
*/
void Person::saveBodyProperties()
{
	extractBodyProperties(bodyProperties, joints);
}

/**
* extrahiert die Körperproportionen aus den Person-eigenen Gelenken in ein neu angelegtes Array und 
* speichert einen Zeiger auf dieses Array in einem Buffer ab 
*/
void Person::collectBodyProperties()
{
	float* bodyPropertiesTemp = new float[NUMBER_OF_BODY_PROPERTIES];
	extractBodyProperties(bodyPropertiesTemp, joints);

	bodyPropertiesBuffer.push_back(bodyPropertiesTemp);
}

/*
* Berechnet aus allen im Buffer für die Körperproportionen gespeicherten Werten 
* jeweils für eine Proportion den Durchschnitt aus allen Werten für diese Proportion
* und speichert diese im eigenen bodyProperties-Array der Person-Klasse 
*/
void Person::calculateBodyProperties()
{
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
		float sum = 0;
		for (liter = bodyPropertiesBuffer.begin(); liter != bodyPropertiesBuffer.end(); liter++) {
			bodyPropertiesTemp = *liter;
			if (bodyPropertiesTemp[i] != 0.0f && bodyPropertiesTemp[i] != smallest_val[i] && bodyPropertiesTemp[i] != greatest_val[i]) {
				sum += pow(bodyPropertiesTemp[i] - bodyProperties[i], 2);
			}
		}
		standardDeviations[i] = sqrt(1 / (numberOfSamples[i] - 1) * sum);
		standardDeviations[i] = max(standardDeviations[i], 0.001f);
	}
	bodyPropertiesBuffer.clear();
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

	float sumOfWeights = 0.0f;
	double accumulatedError = 0.0;

	float sumOfAccuracy = 0.0f;
	float sumOfFactors = 0.0f;

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

		// Summen für die Normierung der Wichtung berechnen
		sumOfWeights += bodyPropertiesWeights[weightIndex];
		//OutputDebugStringA(std::to_string(standardDeviations[i]).c_str());
		sumOfAccuracy += 1 / standardDeviations[i];
		sumOfFactors += bodyPropertiesFactors[i];

#ifdef DEBUG_BODY_PROPERTIES
		OutputDebugStringA("Confidence for BodyProperty No ");
		OutputDebugStringA(std::to_string(i).c_str());
		OutputDebugStringA(": ");
		OutputDebugStringA(std::to_string(min(bodyProperties[i] / propertiesForComparison[i], propertiesForComparison[i] / bodyProperties[i])).c_str());
		OutputDebugStringA("\n");
#endif
		// Quadratische Abweichung, normiert, gewichtet mit Glaubwürdigkeitsfaktor und 1/Standardabweichung
		
		if (bodyProperties[i] != 0.0f) {
			accumulatedError += pow((double) bodyProperties[i] - (double) propertiesForComparison[i], 2) / (double) bodyProperties[i]
				* (double) bodyPropertiesWeights[weightIndex] * (double) bodyPropertiesFactors[i] / (double) standardDeviations[i];
		}
	}
	if (sumOfWeights * sumOfAccuracy * sumOfFactors != 0.0f)
		return 1000000.0 * accumulatedError / (sumOfWeights * sumOfAccuracy * sumOfFactors);
	else
		return 0.0f;
}