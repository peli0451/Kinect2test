// ConsoleApplication1.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>

#include <Kinect.h>
//#include <Kinect.VisualGestureBuilder.h>



int main()
{
	IKinectSensor *sensor;
	auto res = GetDefaultKinectSensor(&sensor);
	sensor->Open();

	IBodyFrameSource *source;
	sensor->get_BodyFrameSource(&source);

	INT32 count;
	source->get_BodyCount(&count); //Anzahl Personen?

	IBody **bodies = new IBody*[count];
	ZeroMemory(bodies, sizeof(IBody*) * count); // mit 0 initialisieren?

	IBodyFrameReader *reader;
	source->OpenReader(&reader);

	while (true)
	{
		//Plan: Iterieren über Köpfe, den niedrigsten z-Wert als Master wählen, Mastervariable
		INT32 masterId = -1;
		int master_z=1000;
		IBodyFrame *frame;
		auto res = reader->AcquireLatestFrame(&frame);
		Joint joints[JointType_Count];
		if (SUCCEEDED(res)) //Falls es gelungen ist, aktuellen Frame zu holen
		{

			res = frame->GetAndRefreshBodyData(count, bodies); //Update für bodies-Array
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
					HandState masterHandState;
					std::cout << "Master: Body " << masterId << "\n";
					bodies[masterId]->GetJoints(JointType_Count, joints);
					bodies[masterId]->get_HandLeftState(&masterHandState);
					auto master_position(joints[JointType::JointType_HandLeft].Position);
					std::cout << "Left Hand: X: " << master_position.X << " Y: " << master_position.Y << " Z:" << master_position.Z << "\n";
					std::cout << "Left Hand State: ";
					switch (masterHandState)
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
				}
			}
			frame->Release();
		}
	}
	reader->Release();

	//IVisualGestureBuilderDatabase *database = nullptr;
	//HRESULT hr = CreateVisualGestureBuilderDatabaseInstanceFromFile(L"C:\Users\Peter\Documents\Kinect Studio\Repository\WinkeWinke.gbd", &database);

	sensor->Close();

    return 0;
}

