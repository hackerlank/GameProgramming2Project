#include <assert.h>
#include "ObjectTypes.h"
#include "BoxObject.h"

BoxObject::BoxObject(float t, float b, float l, float r, float n, float f, char* leaving)
  : GameObject(NULL,1)
{

  m_className = "Box";
  m_type = ObjectTypes::BOX;
  top = t;
  bottom = b;
  left = l;
  right = r;
  back = n;
  front = f;

  if ( strstr(leaving,"left") != 0 ) {
	  this->l = LEFT;
  } else if ( strstr(leaving,"right") != 0 ) {
	  this->l = RIGHT;
  } else if ( strstr(leaving,"top") != 0 ) {
	  this->l = TOP;
  } else if ( strstr(leaving,"bottom") != 0 ) {
	  this->l = BOTTOM;
  } else if ( strstr(leaving,"far") != 0 ) {
	  this->l = BACK;
  } else if ( strstr(leaving,"near") != 0 ) {
	  this->l = FRONT;
  } else {
	  //default
	  this->l = BACK;
  }

}

void BoxObject::process(float dt)
{
}

void BoxObject::move(float dt)
{
  GameObject::move(dt);
}

void BoxObject::render()
{
  // do nothing
}

void BoxObject::computeBoundingBox()
{
  m_boundingBox.empty();

  Vector3 a(left,bottom,back);
  Vector3 b(right,top,front);

  m_boundingBox.add(a);
  m_boundingBox.add(b);
}