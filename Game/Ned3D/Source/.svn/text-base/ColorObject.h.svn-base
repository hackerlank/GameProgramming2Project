#ifndef __COLOROBJECT_H_INCLUDED__
#define __COLOROBJECT_H_INCLUDED__

#include "Objects/GameObject.h"




//colors
enum Color
{
    RED=0, GREEN, BLUE
};

/// \brief Represents a enemy object.
class ColorObject : public GameObject {
public:
	friend class GameObjectManager;
	ColorObject(Model *m, int parts=1, int frames=1);  ///< Constructs a enemy using the given model.
	~ColorObject();         ///< Destroys the enemy.
	Color m_color;

	virtual void process(float dt);  ///< Performs internal logic updates.
	void changeColor(Color col);
	std::vector<std::string> m_allTextures; 
};


#endif