// ConsoleApplication1.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>

#include "KinectControl.h"


/*
* fancy doku
*/
KinectControl::KinectControl() {
	master = {
		-1,
		{ 0,0,0 },
		{ 0,0,0 },
		{ 0,0,0 },
		{ 0,0,0 },
		HandState_Unknown,
		HandState_Unknown,
		100.0
	};
	currentControlMode = DEFAULT_MODE;
}

void KinectControl::init() {
	GetDefaultKinectSensor(&kinectSensor);
	kinectSensor->Open();
	kinectSensor->get_BodyFrameSource(&bodyFrameSource);
	bodyFrameSource->get_BodyCount(&numberOfTrackedBodies); //Anzahl Personen?
	bodyFrameSource->OpenReader(&bodyFrameReader);
}

void KinectControl::run(MotionParameters *motionParameters) {
	//Initialisierung der motionParameters
	motionParameters->translateX = .0f;
	motionParameters->translateY = .0f;
	motionParameters->translateZ = .0f;
	motionParameters->rotateX = .0f;
	motionParameters->rotateY = .0f;
	motionParameters->rotateZ = .0f;

	//Plan: Iterieren �ber K�pfe, den niedrigsten z-Wert als Master w�hlen, Mastervariable
	master.id = -1;
	master.z = 100.0;

	IBodyFrame *bodyFrame;
	result = bodyFrameReader->AcquireLatestFrame(&bodyFrame);

	if (result != S_OK) {
		return;
	}

	result = bodyFrame->GetAndRefreshBodyData(numberOfTrackedBodies, trackedBodies); //Update f�r bodies-Array

	if (result != S_OK) {
		bodyFrame->Release();
		return;
	}

	for (int i = 0; i < numberOfTrackedBodies; ++i)
	{
		BOOLEAN isTracked;
		trackedBodies[i]->get_IsTracked(&isTracked); //ist i-te potentielle Person getrackt
		if (isTracked == TRUE) //Wenn getrackt, lege Gelenkmodell an (Gelenkarray)
		{
			result = trackedBodies[i]->GetJoints(JointType_Count, joints);

			if (SUCCEEDED(result)) //Falls Gelenke erfolgreich geholt
			{
				auto position(joints[JointType::JointType_Head].Position);  //Wei�t position den Position-struct vom joint zu (referenziell)
				if (position.Z < master.z) {
					master.id = i;
					master.z = position.Z;
				}

			}
			}
		}

		if (master.id != -1) {
			//std::cout << "Master hat ID " << masterId << "\n";

			trackedBodies[master.id]->GetJoints(JointType_Count, joints);
			trackedBodies[master.id]->get_HandLeftState(&master.leftHandState);
			trackedBodies[master.id]->get_HandRightState(&master.rightHandState);

			master.leftHandCurrentPosition = joints[JointType::JointType_HandLeft].Position;
			master.rightHandCurrentPosition = joints[JointType::JointType_HandRight].Position;

			//std::cout << master.leftHandCurrentPosition.Z << " -- " << master.rightHandCurrentPosition.Z << "\t";


			if (master.leftHandState == HandState_Closed && master.rightHandState == HandState_Closed) {
				//Startpunkt f�r Puffer
				currentControlMode = CAMERA_MODE;

				//OutputDebugStringA("Rotiere Kamera\n");
				//std::cout << "Rotiere Kamera";
			} else if (master.leftHandState == HandState_Open && master.rightHandState == HandState_Open) {
				float currentPositionX = (master.leftHandCurrentPosition.X + master.rightHandCurrentPosition.X) / 2;
				float currentPositionY = (master.leftHandCurrentPosition.Y + master.rightHandCurrentPosition.Y) / 2;
				float currentPositionZ = (master.leftHandCurrentPosition.Z + master.rightHandCurrentPosition.Z) / 2;

				float lastPositionX = (master.leftHandLastPosition.X + master.rightHandLastPosition.X)/2;
				float lastPositionY = (master.leftHandLastPosition.Y + master.rightHandLastPosition.Y)/2;
				float lastPositionZ = (master.leftHandLastPosition.Z + master.rightHandLastPosition.Z)/2;

				float translateX = currentPositionX - lastPositionX;
				float translateY = currentPositionY - lastPositionY;
				float translateZ = currentPositionZ - lastPositionZ;
				/*
				float threshold = .002f;

				if (std::abs(translateX) < threshold) translateX = .0f;
				if (std::abs(translateY) < threshold) translateY = .0f;
				if (std::abs(translateZ) < threshold) translateZ = .0f;
				*/
				motionParameters->translateX = translateX;
				motionParameters->translateY = translateY;
				motionParameters->translateZ = translateZ;

				//On-Screen-Debug-Gehacke
				//OutputDebugStringA("Verschiebe Kamera\n");
				//std::cout << "Verschiebe Kamera ( " << (int)(motionParameters->translateX * 100) << " "
				//	<< (int)(motionParameters->translateY * 100) << " " << (int)(motionParameters->translateZ * 100) << " )";
			} else {
				//OutputDebugStringA("Master AFK\n");
			}

			std::cout << std::endl;
			master.leftHandLastPosition = master.leftHandCurrentPosition;
			master.rightHandLastPosition = master.rightHandCurrentPosition;

			//auto master_position(joints[JointType::JointType_HandLeft].Position);

			/*
			Marios wildes Rumgehacke
			switch (masterLeftHandState)
			{
			case HandState::HandState_Closed:
			std::cout << "Left Hand: X: " << master_position.X - last_point.X
			<< " Y: " << master_position.Y - last_point.Y
			<< " Z: " << master_position.Z - last_point.Z << "\n";
			break;
			case HandState::HandState_Open:
			std::cout << "0\n";
			break;
			case HandState::HandState_Lasso:
			auto head_position(joints[JointType::JointType_Head].Position);
			float x_dif = head_position.X - master_position.X;
			float y_dif = head_position.Y - master_position.Y;
			std::cout << "Rotieren - X:" << x_dif <<
			" Y: " << y_dif << "\n";
			break;
			}
			*/
		
		}
		bodyFrame->Release();
}
