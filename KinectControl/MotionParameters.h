#pragma once
#include "Eigen/Dense"

class MotionParameters {
public:
	enum MotionTarget {
		TARGET_OBJECT = false,
		TARGET_CAMERA = true
	};

	void setMotion(float translateX, float translateY, float translateZ, Eigen::Quaternionf rotate, MotionTarget target);
	void setTranslation(float translateX, float translateY, float translateZ);
	void setRotation(Eigen::Quaternionf rotate);
	void setTarget(MotionTarget target);
	void resetMotion();
	void resetTranslation();
	void resetRotation();

private:
	float translateX;
	float translateY;
	float translateZ;
	Eigen::Quaternionf rotate;
	MotionTarget target; //0-verändere Model, 1-verändere Kamera
};