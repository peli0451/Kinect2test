#include "stdafx.h"
#include <iostream>
#include "KinectControl.h"

int main()
{
	KinectControl kinectControl;
	kinectControl.init();
	kinectControl.run();
}