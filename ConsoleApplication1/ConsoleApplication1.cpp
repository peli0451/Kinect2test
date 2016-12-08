// ConsoleApplication1.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <iostream>

#include <Kinect.h>



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
		IBodyFrame *frame;
		auto res = reader->AcquireLatestFrame(&frame);

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
						Joint joints[JointType_Count];
						res = bodies[i]->GetJoints(JointType_Count, joints);

						if (SUCCEEDED(res)) //Falls Gelenke erfolgreich geholt
						{
							auto position(joints[JointType::JointType_HandLeft].Position);  //Weißt position den Position-struct vom joint zu (referenziell)
							std::cout << "Body " << i << "\t";
							std::cout << "Left Hand: X: " << position.X << " Y: " << position.Y << " Z:" << position.Z << "\n";
						}
					}
				}
				std::cout << "---------------\n";
			}
			frame->Release();
		}
	}
	reader->Release();

	sensor->Close();

    return 0;
}

