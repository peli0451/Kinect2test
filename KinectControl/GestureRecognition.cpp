/**
* GestureRecognition.cpp
* Stellt Strukturen zur Gestenerkennung bereit, v. A. Konfidenzstruktur, Gestenpuffer und Pufferauswertung
*/
#include "stdafx.h"
#include "GestureRecognition.h"



/**********************************************************
* Konstruktoren
**********************************************************/

GestureRecognition::GestureRecognition()
{
	recognizedGesture = UNKNOWN;
	confidenceBuffer = new Buffer<GestureConfidence>(GESTURE_BUFFER_SIZE);
}



/**********************************************************
* Getter und Setter
**********************************************************/

GestureRecognition::Gesture GestureRecognition::getRecognizedGesture() {
	return recognizedGesture;
}

void GestureRecognition::setRecognizedGesture(Gesture gesture) {
	recognizedGesture = gesture;
}



Buffer<GestureRecognition::GestureConfidence>* GestureRecognition::getConfidenceBuffer()
{
	return confidenceBuffer;
}



/**********************************************************
* Funktionen
**********************************************************/

/**
* Wertet den Buffer mit den erkannten Confidence-Werten aus
* Die Werte werden mittels einer Exponentialfunktion geglättet, dann wird der Maximalwert herausgesucht
*
* Setzt die private Variable recognizedGesture
*/
void GestureRecognition::evaluateBuffer() {

	//Konfidenz aus dem Puffer
	GestureRecognition::GestureConfidence currentConfidence;

	//Ergebniskonfidenz der Auswertung
	GestureRecognition::GestureConfidence finalConfidence = { 0,0,0,0 };

	//Gehe durch den Puffer und glätte alle Komponenten
	for (int i = 0; i < GESTURE_BUFFER_SIZE; i++) {
		float iSmoothFactor = gestureSmooth[i] / gestureSmoothSum;
		currentConfidence = *confidenceBuffer->get(i);
		finalConfidence.flyConfidence += currentConfidence.flyConfidence * iSmoothFactor;
		finalConfidence.grabConfidence += currentConfidence.grabConfidence * iSmoothFactor;
		finalConfidence.translateCameraConfidence += currentConfidence.translateCameraConfidence * iSmoothFactor;
		finalConfidence.rotateCameraConfidence += currentConfidence.rotateCameraConfidence * iSmoothFactor;
		finalConfidence.unknownConfidence += currentConfidence.unknownConfidence * iSmoothFactor;
	}

	//Werte aus, welche Komponente den höchsten Konfidenzwert hat
	float maxConfidence = finalConfidence.unknownConfidence; Gesture maxConfidenceGesture = UNKNOWN;
	if (maxConfidence < finalConfidence.flyConfidence) { maxConfidence = finalConfidence.flyConfidence; maxConfidenceGesture = FLY_GESTURE; }
	if (maxConfidence < finalConfidence.grabConfidence) { maxConfidence = finalConfidence.grabConfidence; maxConfidenceGesture = GRAB_GESTURE; }
	if (maxConfidence < finalConfidence.translateCameraConfidence) { maxConfidence = finalConfidence.translateCameraConfidence; maxConfidenceGesture = TRANSLATE_GESTURE; }
	if (maxConfidence < finalConfidence.rotateCameraConfidence) { maxConfidence = finalConfidence.rotateCameraConfidence;  maxConfidenceGesture = ROTATE_GESTURE; }

	setRecognizedGesture(maxConfidenceGesture);
}