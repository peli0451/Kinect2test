#include "stdafx.h"
#include "MotionParameters.h"



/**********************************************************
* Konstruktoren
**********************************************************/

MotionParameters::MotionParameters() {
	translateX = .0f;
	translateY = .0f;
	translateZ = .0f;
	rotate = Eigen::Quaternionf::Identity();
	target = TARGET_CAMERA;
}



/**********************************************************
* Getter und Setter
**********************************************************/

/**
* Setzt neue motionParameters
*
* @param translateX neuer Translationswert in x-Richtung
* @param translateY neuer Translationswert in y-Richtung
* @param translateZ neuer Translationswert in z-Richtung
* @param r Quaternion für Rotation
* @param target Objekt oder Kamera bewegen?
*/
void MotionParameters::setMotion(float x, float y, float z, Eigen::Quaternionf r, MotionTarget t) {
	setTranslation(x, y, z);
	setRotation(r);
	setTarget(t);
}

void MotionParameters::setTranslation(float x, float y, float z) {
	translateX = x;
	translateY = y;
	translateZ = z;
}

void MotionParameters::setRotation(Eigen::Quaternionf r) {
	rotate = r;
}

void MotionParameters::setTarget(MotionTarget t) {
	target = t;
}

/**
* Nullt die motionParameters
*/
void MotionParameters::resetMotion() {
	setMotion(.0f, .0f, .0f, Eigen::Quaternionf::Identity(), TARGET_CAMERA);
}

/**
* Nullt die motionParameters, nur Translation
*/
void MotionParameters::resetTranslation() {
	setTranslation(.0f, .0f, .0f);
}

/**
* Nullt die motionParameters, nur Rotation
*/
void MotionParameters::resetRotation() {
	setRotation(Eigen::Quaternionf::Identity());
}

float MotionParameters::getTranslateX() {
	return translateX;
}

float MotionParameters::getTranslateY() {
	return translateY;
}

float MotionParameters::getTranslateZ() {
	return translateZ;
}

Eigen::Quaternionf MotionParameters::getRotation() {
	return rotate;
}

MotionParameters::MotionTarget MotionParameters::getTarget() {
	return target;
}