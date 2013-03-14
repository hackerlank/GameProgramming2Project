#ifndef __BOXOBJECT_H_INCLUDED__
#define __BOXOBJECT_H_INCLUDED__

#include "Objects/GameObject.h"

/// \brief Represents a silo object
class BoxObject : public GameObject
{
public:

  enum Leaving { LEFT=0,RIGHT,TOP,BOTTOM,BACK,FRONT };

  friend class Ned3DObjectManager;
  
  BoxObject(float t, float b, float l, float r, float n=-10000000.0f,float f=10000000.0f,char* leaving = "far");
  Leaving l;
  float top;
  float bottom;
  float right;
  float left;
  float back;
  float front;

  virtual void computeBoundingBox();  ///< Updates the object's bounding box.
  virtual void process(float dt);
  virtual void move(float dt);
  virtual void render();

protected:
};

#endif