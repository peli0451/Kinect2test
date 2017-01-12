// ConsoleApplication1.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>

#include <Kinect.h>
#include <Kinect.VisualGestureBuilder.h>

#define TRACKING 1
#define VGBTRACKING 0

int main()
{
	IKinectSensor *sensor;
	auto res = GetDefaultKinectSensor(&sensor);
	sensor->Open();

	IBodyFrameSource *bodyFrameSource;
	sensor->get_BodyFrameSource(&bodyFrameSource);

	INT32 count;
	bodyFrameSource->get_BodyCount(&count); //Anzahl Personen?

	//alte Initialisierung (hässlich)
	//IBody **bodies = new IBody*[count];
	//ZeroMemory(bodies, sizeof(IBody*) * count);

	IBody *bodies[BODY_COUNT] = { nullptr };

	IBodyFrameReader *bodyFrameReader;
	bodyFrameSource->OpenReader(&bodyFrameReader);

	_CameraSpacePoint last_point = { 0,0,0 };

	enum ControlMode {
		DEFAULT_MODE,
		CAMERA_MODE,
		OBJECT_MODE
	};

	ControlMode currentControlMode = DEFAULT_MODE;

	const int handPositionBufferSize = 10;
	_CameraSpacePoint HandPositionBuffer[handPositionBufferSize];


#if TRACKING
	while (true)
	{
		//Plan: Iterieren über Köpfe, den niedrigsten z-Wert als Master wählen, Mastervariable
		INT32 masterId = -1;
		float master_z=100.0;

		IBodyFrame *bodyFrame;
		auto res = bodyFrameReader->AcquireLatestFrame(&bodyFrame);
		
		Joint joints[JointType_Count];
		if (SUCCEEDED(res)) //Falls es gelungen ist, aktuellen Frame zu holen
		{

			res = bodyFrame->GetAndRefreshBodyData(count, bodies); //Update für bodies-Array
			if (SUCCEEDED(res)) //Falls Update gelungen
			{
				for (int i = 0; i < count; ++i)
				{
					BOOLEAN is_tracked;
					bodies[i]->get_IsTracked(&is_tracked); //ist i-te potentielle Person getrackt
					if (is_tracked == TRUE) //Wenn getrackt, lege Gelenkmodell an (Gelenkarray)
					{
						
						res = bodies[i]->GetJoints(JointType_Count, joints);

						if (SUCCEEDED(res)) //Falls Gelenke erfolgreich geholt
						{
							auto position(joints[JointType::JointType_Head].Position);  //Weißt position den Position-struct vom joint zu (referenziell)
							if (position.Z < master_z) {
								masterId = i;
								master_z = position.Z;
							}
	
						}
					}
				}
				if (masterId != -1) {
					HandState masterLeftHandState;
					HandState masterRightHandState;

					//std::cout << "Master hat ID " << masterId << "\n";

					bodies[masterId]->GetJoints(JointType_Count, joints);
					bodies[masterId]->get_HandLeftState(&masterLeftHandState);
					bodies[masterId]->get_HandRightState(&masterRightHandState);

					auto masterLeftHandPosition(joints[JointType::JointType_HandLeft].Position);
					auto masterRightHandPosition(joints[JointType::JointType_HandRight].Position);

					std::cout << masterLeftHandPosition.Z << " -- " << masterRightHandPosition.Z << "\t";

					if (masterLeftHandState == HandState_Closed && masterRightHandState == HandState_Closed) {
						//Startpunkt für Puffer
						currentControlMode = CAMERA_MODE;
						
						std::cout << "Rotiere Kamera";
					}
					std::cout << std::endl;

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
					

					std::cout << "Left Hand: X: " << master_position.X << " Y: " << master_position.Y << " Z:" << master_position.Z << "\n";
					std::cout << "Left Hand State: ";

					
					switch (masterLeftHandState)
						{
						case HandState::HandState_Closed:
							std::cout << "Closed\n";
							break;
						case HandState::HandState_Open:
							std::cout << "Open\n";
							break;
						case HandState::HandState_NotTracked:
							std::cout << "Not tracked\n";
							break;
						case HandState::HandState_Lasso:
							std::cout << "Lasso\n";
							break;
						case HandState::HandState_Unknown:
							std::cout << "Unknown\n";
							break;
						}

					std::cout << "---------------\n";
					*/
					//last_point = master_position;
				}
			}
			bodyFrame->Release();
		}
	}
	bodyFrameReader->Release();
#endif

#if VGBTRACKING
	IVisualGestureBuilderDatabase *database = nullptr;
	HRESULT hr = CreateVisualGestureBuilderDatabaseInstanceFromFile(L"C:\\Users\\Peter\\Documents\\Kinect Studio\\Repository\\WinkeWinke.gbd", &database);

	UINT numGestures = 0;
	database->get_AvailableGesturesCount(&numGestures); //Hole #Gesten
	IGesture **gestures = new IGesture*[numGestures]; //Zeiger auf Array von Gesten
	database->get_AvailableGestures(numGestures, gestures); //Hole Gesten, befülle Array

	for (UINT i = 0; i < numGestures; i++) {
		WCHAR name[2048];
		ZeroMemory(name, sizeof(name));
		gestures[i]->get_Name(2048, name);
		GestureType type;
		gestures[i]->get_GestureType(&type);
		std::cout << name << " (Type " << type <<")\n";
	}
	
	std::cout.flush();

	IVisualGestureBuilderFrameSource *vgbFrameSource;
	IVisualGestureBuilderFrameReader *vgbFrameReader;

	CreateVisualGestureBuilderFrameSource(sensor, 0, &vgbFrameSource);
	vgbFrameSource->AddGestures(numGestures, gestures); //Alle Gesten hinzufügen
	vgbFrameSource->SetIsEnabled(gestures[0], TRUE); //Schaltet erste Geste an

	res = vgbFrameSource->OpenReader(&vgbFrameReader);
	WAITABLE_HANDLE arrivedHandle;
	vgbFrameReader->SubscribeFrameArrived(&arrivedHandle);

	res = sensor->get_BodyFrameSource(&bodyFrameSource);
	res = bodyFrameSource->OpenReader(&bodyFrameReader);
	WAITABLE_HANDLE bodyHandle;
	bodyFrameReader->SubscribeFrameArrived(&bodyHandle);

	while (true) {
		bool run = true;

		//Nachrichten Empfangen?
		IBodyFrameArrivedEventArgs *bodyEventData;
		res = bodyFrameReader->GetFrameArrivedEventData(bodyHandle, &bodyEventData);
		//if (!SUCCEEDED(res)) continue;
		if (SUCCEEDED(res)) {
			//??
			IBodyFrameReference *bodyFrameReference;
			res = bodyEventData->get_FrameReference(&bodyFrameReference);
			if (SUCCEEDED(res)) {
				IBodyFrame *bodyFrame;
				res = bodyFrameReference->AcquireFrame(&bodyFrame);
				if (SUCCEEDED(res)) {
					res = bodyFrame->GetAndRefreshBodyData(BODY_COUNT, bodies);
					std::cout << "Bodies refreshed ----------\n";
					if (SUCCEEDED(res)) {
						run = false;
						for (UINT i = 0; i < BODY_COUNT; ++i) {
							UINT64 trackingId;
							bodies[i]->get_TrackingId(&trackingId);
							if (trackingId > 0) {
								BOOLEAN valid;
								vgbFrameSource->put_TrackingId(trackingId);
								auto res = vgbFrameSource->get_IsTrackingIdValid(&valid);
								//std::cout << (valid > 0 ? "Valid ID" : "Invalid Id") << "\n";
								run = true;
								//break; //Zwecklos, bricht FOR ab, da gehen einem höchstens ids durch die Lappen
							} else {
								//std::cout << "Nothing Tracked\n";
							}

						}
					}
					bodyFrame->Release();
				}
				bodyFrameReference->Release();
			}
			bodyEventData->Release();
		}
		vgbFrameReader->put_IsPaused(!run);

		IVisualGestureBuilderFrameArrivedEventArgs *vgbFrameEventArgs;
		res = vgbFrameReader->GetFrameArrivedEventData(arrivedHandle, &vgbFrameEventArgs);

		if (SUCCEEDED(res)) {
			IVisualGestureBuilderFrameReference *vgbFrameReference;
			res = vgbFrameEventArgs->get_FrameReference(&vgbFrameReference);

			if (SUCCEEDED(res)) {
				IVisualGestureBuilderFrame *vgbFrame;
				res = vgbFrameReference->AcquireFrame(&vgbFrame);
				if (SUCCEEDED(res)) {
					IDiscreteGestureResult *gestureResult = nullptr;
					res = vgbFrame->get_DiscreteGestureResult(gestures[0], &gestureResult);

					if (gestureResult != NULL) {
						float confidence;
						gestureResult->get_Confidence(&confidence);
						BOOLEAN detected;
						gestureResult->get_Detected(&detected);
						std::cout << (detected ? "Detected,     " : "Not Detected, ")  << "Confidence: " << confidence << "\n";
						std::cout.flush();
						gestureResult->Release();
					}
					vgbFrame->Release();
				}
				vgbFrameReference->Release();
			}
			vgbFrameEventArgs->Release();
		}
	}

	vgbFrameReader->Release();
	vgbFrameSource->Release();
	bodyFrameReader->Release();
	bodyFrameSource->Release();
#endif
	sensor->Close();

	while (true) {}

    return 0;
}

