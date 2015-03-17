//Author: Simon Tran

#ifndef __MAIN_SCENE_H__
#define __MAIN_SCENE_H__

#include "cocos2d.h"
#include "Joystick.h"
#include "JoystickSkin.h"

USING_NS_CC;

class MainScene: public Layer {
public:
	// there's no 'id' in cpp, so we recommend returning the class instance pointer
	static cocos2d::Scene* createScene();

	// Here's a difference. Method 'init' in cocos2d-x returns bool, instead of returning 'id' in cocos2d-iphone
	virtual bool init();
	void menuCloseCallback(Ref* pSender);
	
	// implement the "static create()" method manually
	CREATE_FUNC(MainScene)
	;

	Joystick* ljoypad;
	Joystick* rjoypad;

private:
	Label *m_labelStatusCode;
	JoystickSkin *lbaseStick;
	cocos2d::Sprite *ljoypadBG;
	cocos2d::Sprite *ljoypadThumb;
	JoystickSkin *rbaseStick;
	cocos2d::Sprite *rjoypadBG;
	cocos2d::Sprite *rjoypadThumb;
};

#endif // __MAIN_SCENE_H__
