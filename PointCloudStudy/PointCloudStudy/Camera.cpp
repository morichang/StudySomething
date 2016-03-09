#include "Camera.h"
#include <math.h>

Camera::Camera(){
	init();
}

void Camera::init(){
	cameraAngle.x = 0.00;
	cameraAngle.y = 0.00;
	cameraAngle.z = 0.00;
	cameraPosition = VGet(0, 0, 0);

	GetMousePoint(&prevMouseX, &prevMouseY);
}

void Camera::update(int dt, char* keys){
	//É}ÉEÉXÇÃà íuéÊìæ
	int mouseX, mouseY;
	GetMousePoint(&mouseX, &mouseY);
	int mx, my;

	mx = mouseX - prevMouseX;
	prevMouseX = mouseX;
	my = mouseY - prevMouseY;
	prevMouseY = mouseY;

	//ÉJÉÅÉâêUÇËå¸Ç´ë¨ìxê›íË
	double mouseSensitive = 0.000135 * dt;
	if (mouseSensitive > 0.001) mouseSensitive = 0.001;	//ÉCÉLÉXÉMñhé~

	cameraAngle.x += my * mouseSensitive;
	cameraAngle.y += mx * mouseSensitive;

	//ÉJÉÅÉâà⁄ìÆë¨ìxåàíË
	double speed = 1.8 * dt;
	if (keys[KEY_INPUT_LSHIFT]) {
		speed *= 2.0;
	}
	if (speed > 1000) speed = 1000;						//ÉCÉLÉXÉMñhé~

	double rollSpeed = 0.000625 * dt;
	if (keys[KEY_INPUT_Q]) cameraAngle.z += 0.01f;
	if (keys[KEY_INPUT_E]) cameraAngle.z -= 0.01f;

	if (keys[KEY_INPUT_W]) {
		cameraPosition.x += sin(cameraAngle.y) * speed;
		cameraPosition.y += -sin(cameraAngle.x) * speed;
		cameraPosition.z += cos(cameraAngle.y) * speed;
	}
	if (keys[KEY_INPUT_S]) {
		cameraPosition.x -= sin(cameraAngle.y) * speed;
		cameraPosition.y -= -sin(cameraAngle.x) * speed;
		cameraPosition.z -= cos(cameraAngle.y) * speed;
	}
	if (keys[KEY_INPUT_A]) {
		cameraPosition.x += sin(cameraAngle.y - DX_PI / 2) * speed;
		cameraPosition.y += -sin(cameraAngle.z) * speed;
		cameraPosition.z += cos(cameraAngle.y - DX_PI / 2) * speed;
	}
	if (keys[KEY_INPUT_D]) {
		cameraPosition.x += sin(cameraAngle.y + DX_PI / 2) * speed;
		cameraPosition.y += -sin(cameraAngle.z) * speed;
		cameraPosition.z += cos(cameraAngle.y + DX_PI / 2) * speed;
	}
	if (keys[KEY_INPUT_SPACE]) {
		cameraPosition.y += speed;
	}
	if (keys[KEY_INPUT_LCONTROL]) {
		cameraPosition.y -= speed;
	}

	SetCameraPositionAndAngle(cameraPosition, cameraAngle.x, cameraAngle.y, cameraAngle.z);
}