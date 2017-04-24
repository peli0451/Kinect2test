#include "MotionParameters.h"

/**
* Setzt neue motionParameters
*
* @param translateX neuer Translationswert in x-Richtung
* @param translateY neuer Translationswert in y-Richtung
* @param translateZ neuer Translationswert in z-Richtung
* @param rotateX neuer Rotationswinkel um x-Achse
* @param rotateY neuer Rotationswinkel um y-Achse
* @param rotateZ neuer Rotationswinkel um z-Achse
* @param target Objekt oder Kamera bewegen?
*/
void MotionParameters::setMotion(float x, float y, float z, Eigen::Quaternionf r, MotionTarget t) {
	setTranslation(x, y, z);
	setRotation(r);
	setTarget(t);
}

/**
* Setzt neue motionParameters, nur Translation
*
* @param translateX neuer Translationswert in x-Richtung
* @param translateY neuer Translationswert in y-Richtung
* @param translateZ neuer Translationswert in z-Richtung
*/
void MotionParameters::setTranslation(float x, float y, float z) {
	translateX = x;
	translateY = y;
	translateZ = z;
}

/**
* Setzt neue motionParameters, nur Rotation
*
* @param rotateX neuer Rotationswinkel um x-Achse
* @param rotateY neuer Rotationswinkel um y-Achse
* @param rotateZ neuer Rotationswinkel um z-Achse
*/
void MotionParameters::setRotation(Eigen::Quaternionf r) {
	rotate = r;
}

/**
* Setzt neue motionParameters, nur Objekt/Kamera-Modus
*
* @param target neuer Modus (Objekt oder Kamera)
*/
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