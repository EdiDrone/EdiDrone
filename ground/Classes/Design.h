//Author: Simon Tran

#ifndef __DESIGN_H__
#define __DESIGN_H__

#include "cocos2d.h"

USING_NS_CC;
class Design: public Layer {

public:
	static Design* create();

	int top(Layer *o1);
	int left(Layer *o1, Layer *o2);
	int right(Layer *o1);
	int bottom(Layer *o1, Layer *o2);
	int centerx(Layer *o1, Layer *o2);
	int centery(Layer *o1, Layer *o2);
	int cosx(Layer *o1, Layer *o2);
	int siny(Layer *o1, Layer *o2);
	int insideTop(Layer *o1, Layer *o2);
	int insideRight(Layer *o1, Layer *o2);

	int sinsideTop(Layer *o1);
	int sinsideRight(Layer *o1);
	int scenterx(Layer *o1);
	int slcenterx(Layer *o1, float s);
	int srcenterx(Layer *o1, float s);
	int scentery(Layer *o1);
	
	void setScaleX(Layer *o1);
	void setScaleY(Layer *o1);

private:
	Size screen;
	
	Design();
	virtual bool init();

};
#endif