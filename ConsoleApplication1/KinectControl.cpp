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

void KinectControl::run() {
	while (true) {
		//Plan: Iterieren ¸ber Kˆpfe, den niedrigsten z-Wert als Master w‰hlen, Mastervariable
		master.id = -1;
		master.z = 100.0;

		IBodyFrame *bodyFrame;
		result = bodyFrameReader->AcquireLatestFrame(&bodyFrame);

		if (FAILED(result)) {
			continue;
		}

		result = bodyFrame->GetAndRefreshBodyData(numberOfTrackedBodies, trackedBodies); //Update f¸r bodies-Array

		if (FAILED(result)) {
			bodyFrame->Release();
			continue;
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
					auto position(joints[JointType::JointType_Head].Position);  //Weiﬂt position den Position-struct vom joint zu (referenziell)
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

			std::cout << master.leftHandCurrentPosition.Z << " -- " << master.rightHandCurrentPosition.Z << "\t";


			if (master.leftHandState == HandState_Closed && master.rightHandState == HandState_Closed) {
				//Startpunkt f¸r Puffer
				currentControlMode = CAMERA_MODE;

				std::cout << "Rotiere Kamera";
			}

			if (master.leftHandState == HandState_Open && master.rightHandState == HandState_Open) {
				motionParameters.translateX = master.leftHandCurrentPosition.X - master.leftHandLastPosition.X;
				motionParameters.translateY = master.leftHandCurrentPosition.Y - master.leftHandLastPosition.Y;
				motionParameters.translateZ = master.leftHandCurrentPosition.Z - master.leftHandLastPosition.Z;

				//On-Screen-Debug-Gehacke
				std::cout << "Verschiebe Kamera ( " << (int)(motionParameters.translateX * 100) << " "
					<< (int)(motionParameters.translateY * 100) << " " << (int)(motionParameters.translateZ * 100) << " )";


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
}

/*
int main()
{
	init();
	while (true) run();
	//bodyFrameReader->Release();
	//kinectSensor->Close();
    //return 0;
}

*/