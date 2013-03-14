#include "ColorObject.h"


ColorObject::ColorObject(Model *m, int parts, int frames):
	GameObject(m,parts,frames),
	m_color(RED) {

  m_allTextures.resize(3);
  m_allTextures[0] = "red.tga"; // red
  m_allTextures[1] = "green2.tga"; // green
  m_allTextures[2] = "blue.tga"; // blue
  for (int a =0; a < (int)m_allTextures.size(); a++)
    gRenderer.cacheTextureDX(m_allTextures[a].c_str());
}

ColorObject::~ColorObject() {

}

void ColorObject::process(float dt)
{
  int numParts = m_pModel->getPartCount();
  for (int a = 0; a < numParts; a++)
    m_pModel->setPartTextureName(a,m_allTextures[m_color].c_str());
   m_pModel->cache();
}

void ColorObject::changeColor(Color col) {// set texture 
  m_color=col;
}
