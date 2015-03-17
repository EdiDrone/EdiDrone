//Author: Simon Tran

#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

//General includes:
#include <iostream>
#include <stdio.h>
#include <string>

//Network related includes:
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>

class JoystickSkin;

using namespace cocos2d;
class Joystick: public cocos2d::Layer {
protected:
	float joystickRadiusSq;
	float thumbRadiusSq;
	float deadRadiusSq;

	 CC_SYNTHESIZE(bool, enabled, Enabled)
	;CC_SYNTHESIZE_READONLY(cocos2d::Point, stickPosition, StickPosition)
	;CC_SYNTHESIZE_READONLY(float, degrees, Degrees)
	;CC_SYNTHESIZE_READONLY(cocos2d::Point, velocity, Velocity)
	;CC_SYNTHESIZE(bool, autoCenter, AutoCenter)
	;CC_SYNTHESIZE_READONLY(bool, isDPad, IsDPad)
	;CC_SYNTHESIZE(bool, hasDeadzone, HasDeadzone)
	;CC_SYNTHESIZE(int, numberOfDirections, NumberOfDirections)
	;CC_SYNTHESIZE(JoystickSkin*, baseStick, BaseStick)
	;CC_SYNTHESIZE_READONLY(float, joystickRadius, JoystickRadius)
	;CC_SYNTHESIZE_READONLY(float, thumbRadius, ThumbRadius)
	;CC_SYNTHESIZE_READONLY(float, deadRadius, DeadRadius)
	;

	static Joystick* create(float s);
	Joystick(float s);
	bool initWithRect(cocos2d::Rect rect);
	virtual void onEnterTransitionDidFinish();
	
	void setIsDPad(bool b);
	void setJoystickRadius(float r);
	void setThumbRadius(float r);
	void setDeadRadius(float r);
	
	bool onTouchBegan(Touch *touch, Event *event);
	void onTouchMoved(Touch *touch, Event *event);
	void onTouchEnded(Touch *touch, Event *event);
	void onTouchCancelled(Touch *touch, Event *event);

	void sendData(float dx, float dy);
	void getData();

	void start();

private:
	cocos2d::Point _beganPoint;
	int side;
	Size screen;

	int sd, ret;
	struct sockaddr_in server;
	struct in_addr ipv4addr;
	struct hostent *hp;
	
	virtual bool init();
	void updateVelocity(cocos2d::Point point);
	void setTouchRadius();

};
#endif
