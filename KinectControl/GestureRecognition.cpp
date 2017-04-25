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
void GestureRecognition::recognize() {

	//Konfidenz aus dem Puffer
	GestureRecognition::GestureConfidence currentConfidence;

	//Ergebniskonfidenz der Auswertung
	GestureRecognition::GestureConfidence finalConfidence = { 0,0,0,0 };

	//Gehe durch den Puffer und glätte alle Komponenten
	for (int i = 0; i < GESTURE_BUFFER_SIZE; i++) {
		currentConfidence = *confidenceBuffer->get(i);
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

	setRecognizedGesture(maxConfidenceGesture);
}