//Author: Simon Tran

#include "Design.h"

Design::Design() {
}

Design* Design::create() {
	Design* object = new Design();

	if (object->init()) {
		object->autorelease();
	} else {
		CC_SAFE_RELEASE_NULL(object);
	}
	return object;
}

bool Design::init() {
	if (!Layer::init()) {
		return false;
	}

	screen = Director::getInstance()->getVisibleSize();

	return true;
}

// ---------- General function for all to use ---------- //
// Place the second sprite ontop of the first sprite
int Design::top(Layer *o1) {
	return o1->getPosition().y + (o1->getContentSize().height * o1->getScaleY());
}

// Place the second sprite to the left of the first sprite
int Design::left(Layer *o1, Layer *o2) {
	return o1->getPosition().x - (o2->getContentSize().width * o2->getScaleX());
}

// Place the second sprite to the right of the first sprite
int Design::right(Layer *o1) {
	return o1->getPosition().x + (o1->getContentSize().width * o1->getScaleX());
}

// Place the second sprite at the bottom of the first sprite
int Design::bottom(Layer *o1, Layer *o2) {
	return o1->getPosition().y - (o2->getContentSize().height * o2->getScaleY());
}

// Place the second sprite horizontally in the center of the first sprite
int Design::centerx(Layer *o1, Layer *o2) {
	return o1->getPosition().x
			+ (((o1->getContentSize().width * o1->getScaleX())
					- (o2->getContentSize().width * o2->getScaleX())) / 2);
}

// Place the second sprite vertically in the center of the first sprite
int Design::centery(Layer *o1, Layer *o2) {
	return o1->getPosition().y
			+ (((o1->getContentSize().height * o1->getScaleY())
					- (o2->getContentSize().height * o2->getScaleY())) / 2);
}

// Uses width as a radius
int Design::cosx(Layer *o1, Layer *o2) {
	return o1->getPosition().x + (o2->getContentSize().width* cos(CC_DEGREES_TO_RADIANS(o1->getRotationX())));
}

// Uses width as a radius
int Design::siny(Layer *o1, Layer *o2) {
	return o1->getPosition().y + (o2->getContentSize().width * -sin(CC_DEGREES_TO_RADIANS(o1->getRotationY())));
}

// Top align the second sprite relative to the first sprite
int Design::insideTop(Layer *o1, Layer *o2) {
	return (o1->getPosition().y
			+ (o1->getContentSize().height * o1->getScaleY()))
			- (o2->getContentSize().height * o2->getScaleY());
}

// Right align the second sprite relative to the first sprite
int Design::insideRight(Layer *o1, Layer *o2) {
	return (o1->getPosition().x + (o1->getContentSize().width * o1->getScaleX()))
			- (o2->getContentSize().width * o2->getScaleX());
}

// ---------- Align the sprite relative to the screen ---------- //
// Place the sprite at the top of the screen
// There is no need for a function that aligns the sprite at the bottom of the screen 
// Since it's just point zero 
int Design::sinsideTop(Layer *o1) {
	return screen.height - (o1->getContentSize().height * o1->getScaleY());
}

// Place the sprite at the right of the screen
// There is no need for a function that aligns the sprite at the left of the screen 
// Since it's just point zero 
int Design::sinsideRight(Layer *o1) {
	return screen.width - (o1->getContentSize().width * o1->getScaleX());
}

int Design::scenterx(Layer *o1) {
	return (screen.width - (o1->getContentSize().width * o1->getScaleX())) / 2;
}

int Design::slcenterx(Layer *o1, float s) {
	return (screen.width / 2 - (o1->getContentSize().width * o1->getScaleX()))
			/ 2;
}

// Center horizontally relative to the screen
int Design::srcenterx(Layer *o1, float s) {
	return screen.width / 2
			+ ((screen.width / 2
					- (o1->getContentSize().width * o1->getScaleX())) / 2);
}

// Center vertically relative to the screen
int Design::scentery(Layer *o1) {
	return (screen.height - (o1->getContentSize().height * o1->getScaleY())) / 2;
}

// A void function instead of a get function
// Scales the sprite to fit the screen
void Design::setScaleX(Layer *o1) {
	o1->setScaleX(
			screen.width / (o1->getContentSize().width * o1->getScaleX()));
}

// A void function instead of a get function
// Scales the sprite to fit the screen
void Design::setScaleY(Layer *o1) {
	o1->setScaleY(
			screen.height / (o1->getContentSize().height * o1->getScaleY()));
}