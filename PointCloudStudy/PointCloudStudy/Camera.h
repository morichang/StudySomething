#pragma once

#include "DxLib.h"

class Camera{
private:
	VECTOR cameraAngle;
	VECTOR cameraPosition;
	int prevMouseX, prevMouseY;
public:
	Camera();

	void init();

	void update(int dt, char* keys);
};