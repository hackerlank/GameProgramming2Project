/*
----o0o=================================================================o0o----
* Copyright (c) 2006, Ian Parberry
* All rights reserved.
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of the University of North Texas nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS''
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE REGENTS AND CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
----o0o=================================================================o0o----
*/

/// \file Ned3DObjectManager.cpp
/// \brief Code for the Ned3DObjectManager class, which is responsible for all
/// objects specific to this demo.

/// Edited by the SAGE team, 2005
/// Last updated June 13, 2006
#define _USE_MATH_DEFINES


#include <math.h>
#include <algorithm>
#include "Common/MathUtil.h"
#include "Common/RotationMatrix.h"
#include "Graphics/ModelManager.h"
#include "Objects/GameObject.h"
#include "Particle/ParticleEngine.h"
#include "Sound/SoundManager.h"
#include "Terrain/Terrain.h"
#include "Water/Water.h"
#include "Ned3DObjectManager.h"
#include "Objects.h"
#include "ObjectTypes.h"
#include "BoxObject.h"
#include "DirectoryManager/DirectoryManager.h"
#include "PlaneObject.h"
#include "../../Bullet/src/btBulletDynamicsCommon.h"
#include "../../Bullet/src/BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.h"
#include "../../Bullet/src/BulletCollision/CollisionShapes/btConvexPolyhedron.h"





PlaneObject* gPlane;

Ned3DObjectManager::Ned3DObjectManager() :
  m_models(NULL),
  m_planeModel(NULL),
  m_enemyModel(NULL),
  m_bulletModel(NULL),
  m_siloModel(NULL),
  m_windmillModel(NULL),
  m_plane(NULL),
  m_terrain(NULL),
  m_water(NULL),
  m_startingEnemies(0)
{
}

Ned3DObjectManager::~Ned3DObjectManager() {
}

void Ned3DObjectManager::setModelManager(ModelManager &models)
{
  if(m_models == &models)
    return;
  m_models = &models;
  m_planeModel = NULL;
  m_enemyModel = NULL;
  m_bulletModel = NULL;
  m_siloModel = NULL;
  m_windmillModel = NULL;
}

void Ned3DObjectManager::clear()
{
  m_planeModel = NULL;
  m_enemyModel = NULL;
  m_bulletModel = NULL;
  m_siloModel = NULL;
  m_windmillModel = NULL;
  m_plane = NULL;
  m_terrain = NULL;
  m_water = NULL;
  m_boxes.clear();
  m_boxesToAdd.clear();
  m_enemys.clear();
  m_bullets.clear();
  m_furniture.clear();
  GameObjectManager::clear();
  m_startingEnemies = 0;
}

void Ned3DObjectManager::handleInteractions()
{

  for(ObjectSetIter fit = m_furniture.begin(); fit != m_furniture.end(); ++fit)
    interactPlaneFurniture(*m_plane, **fit);
  if(!m_boxes.empty()) {
	  for(ObjectSetIter bit = m_boxes.begin(); bit != m_boxes.end(); ++bit) {
		BoxObject &box = (BoxObject &)**bit;
		interactPlaneBox(*m_plane, box);
		break;
	  }
  }
  for(ObjectSetIter bit = m_bullets.begin(); bit != m_bullets.end(); ++bit)
  {
    BulletObject &bullet = (BulletObject &)**bit;
    if(!bullet.isAlive()) continue;
      
    // Check for bullets hitting stuff
    for(ObjectSetIter cit = m_enemys.begin(); cit != m_enemys.end(); ++cit)
    {
      EnemyObject &enemy = (EnemyObject &)**cit;
      if(!enemy.isAlive()) continue;
      interactEnemyBullet(enemy, bullet);
    }
    GameObject *victim = bullet.getVictim();
    if(victim != NULL)
    {
      // Bullet hit something
      switch(victim->getType())
      {
        case ObjectTypes::ENEMY :
        {
          shootEnemy((EnemyObject &)*victim);
        } break;
      }
    }
  }
  
  // Handle enemy-enemy interactions (slow....) and enemy-plane interactions
  
  for(ObjectSetIter cit1 = m_enemys.begin(); cit1 != m_enemys.end(); ++cit1)
  {
    EnemyObject &enemy1 = (EnemyObject &)**cit1;
    if(!enemy1.isAlive()) continue;
    
    interactPlaneEnemy(*m_plane,enemy1);
    interactEnemyTerrain(enemy1,*m_terrain);
    
    ObjectSetIter cit2 = cit1;
    for(++cit2; cit2 != m_enemys.end(); ++cit2)
    {
      EnemyObject &enemy2 = (EnemyObject &)**cit2;
      if(!enemy2.isAlive()) continue;
      interactEnemyEnemy(enemy1, enemy2);
    }
  }
  

  // Handle plane crashes
  
  interactPlaneTerrain(*m_plane, *m_terrain);
  interactPlaneWater(*m_plane, *m_water);
}

float* Ned3DObjectManager::getBounds()
{
	float* floats = (float*)malloc(6*sizeof(float));
	floats[0] = 1000000.0f; // init to whatever since they will be replaced unless game is not in playing state
	floats[1] = 1000000.0f;
	floats[2] = 1000000.0f;
	floats[3] = 1000000.0f;
	floats[4] = 1000000.0f;
	floats[5] = 1000000.0f;
  for(ObjectSetIter bit = m_boxes.begin(); bit != m_boxes.end(); ++bit) {
	BoxObject &box = (BoxObject &)**bit;
    if ( interactPlaneBox(*m_plane, box) ) {
		floats[0] = box.bottom;
		floats[1] = box.top;
		floats[2] = box.left;
		floats[3] = box.right;
		floats[4] = box.back;
		floats[5] = box.front;
	}
  }
  return floats;
}

bool Ned3DObjectManager::importXml(const std::string &fileName, bool defaultDirectory) {
	using namespace std;
  
	
  

  TiXmlDocument doc;
  if(defaultDirectory)
    gDirectoryManager.setDirectory(eDirectoryXML);
  if(!doc.LoadFile(fileName.c_str()))
    return false;

  

  TiXmlElement *root = doc.FirstChildElement("walls");
  
  gDirectoryManager.setDirectory(eDirectoryModels);

  // Utility variables
  
  Vector3 v;
//  int i;
//  double d;
  Model *m = NULL;
  const char *cs = NULL;
  
  // Read models
  
  for(TiXmlElement *model = root->FirstChildElement("wall"); model != NULL; model = model->NextSiblingElement("wall"))
  {
    // Get main model attributes
    cs = model->Attribute("left");
    if(cs == NULL) continue; // Broken model entry; must have name
    float left = (float)atof(cs);
    
	cs = model->Attribute("right");
    if(cs == NULL) continue; // Broken model entry; must have name
    float right = (float)atof(cs);

	cs = model->Attribute("top");
    if(cs == NULL) continue; // Broken model entry; must have name
    float top = (float)atof(cs);

	cs = model->Attribute("bottom");
    if(cs == NULL) continue; // Broken model entry; must have name
    float bottom = (float)atof(cs);
    
	cs = model->Attribute("near");
    if(cs == NULL) continue; // Broken model entry; must have name
    float front = (float)atof(cs);

	cs = model->Attribute("far");
    if(cs == NULL) continue; // Broken model entry; must have name
    float back = (float)atof(cs);

	cs = model->Attribute("leaving");  // what wall does it leave on?
    if(cs == NULL) continue; // Broken model entry; must have name
    char* change = (char*)cs;

    // Load wall/box


	this->spawnBox(top,bottom,left,right,back,front,change);
   
  }


  return true;
}

unsigned int Ned3DObjectManager::spawnPlane(const Vector3 &position, const EulerAngles &orientation)
{
  if(m_plane != NULL)
    return 0;  // Only one plane allowed
  if(m_planeModel == NULL)
    m_planeModel = m_models->getModelPointer("Plane"); // Cache plane model
  if(m_planeModel == NULL)
    return 0;  // Still NULL?  No such model
  
  m_plane = new PlaneObject(m_planeModel);
  m_plane->setPosition(position);
  m_plane->setOrientation(orientation);

  unsigned int id = addObject(m_plane, "Plane");
 
  addPhysics(m_plane,true);

  m_plane->setPPosition(position.x,position.y,position.z);

  return id;
}

unsigned int Ned3DObjectManager::spawnEnemy(const Vector3 &position, const EulerAngles &orientation, float speed)
{
  if(m_enemyModel == NULL)
    m_enemyModel = m_models->getModelPointer("Enemy"); // Cache enemy model
  if(m_enemyModel == NULL)
    return 0;  // Still NULL?  No such model
  EnemyObject *enemy = new EnemyObject(m_enemyModel);
  enemy->setSpeed(speed);
  enemy->setPosition(position);
  enemy->setOrientation(orientation);
  enemy->setMovementPattern(EnemyObject::MP_STRAIGHT);
  unsigned int id = addObject(enemy);

  addPhysics(enemy,true);
  enemy->setPPosition(position.x,position.y,position.z);

  m_enemys.insert(enemy);
  m_startingEnemies++;

  return id;
}

unsigned int Ned3DObjectManager::spawnEnemy(const Vector3 &position, const Vector3 &circleCenter, float speed, bool flyLeft)
{
  if(m_enemyModel == NULL)
    m_enemyModel = m_models->getModelPointer("Enemy"); // Cache enemy model
  if(m_enemyModel == NULL)
    return 0;  // Still NULL?  No such model
  EnemyObject *enemy = new EnemyObject(m_enemyModel);
  enemy->setSpeed(speed);
  enemy->setPosition(position);
  enemy->setCirclingParameters(circleCenter, flyLeft);
  enemy->setMovementPattern(EnemyObject::MP_CIRCLING);
  unsigned int id = addObject(enemy);

  addPhysics(enemy,true);
  enemy->setPPosition(position.x,position.y,position.z);

  m_enemys.insert(enemy);
  m_startingEnemies++;

  return id;
}

unsigned int Ned3DObjectManager::spawnBullet(const Vector3 &position, const EulerAngles &orientation,const Color color)
{
  if(m_bulletModel == NULL)
    m_bulletModel = m_models->getModelPointer("Bullet"); // Cache enemy model
  if(m_bulletModel == NULL)
    return 0;  // Still NULL?  No such model
  BulletObject *bullet = new BulletObject(m_bulletModel);
  bullet->changeColor(color);
  bullet->setPosition(position);
  bullet->setOrientation(orientation);
  unsigned int id = addObject(bullet);
  m_bullets.insert(bullet);

  addPhysics(bullet,true);
  bullet->setPPosition(position);
  return id;
}

unsigned int Ned3DObjectManager::spawnBox(float t, float b, float l, float r, float back, float f, char* leaving){
  BoxObject *box = new BoxObject(t,b,l,r,back,f,leaving);
  unsigned int id = addObject(box);
  if (m_boxes.empty()) {
	m_boxes.insert(box);/*
	char* tp = (char*)malloc(sizeof(char*)*500); // allocate char
	sprintf_s(tp,500,"Hello, new box is in the house!\n\0");
	OutputDebugString(tp);*/
  } else {
	m_boxesToAdd.insert(box);
  }

  addPhysics(box,false);
  box->setPPosition(r-l,t-b,f-back);

  return id;
}

unsigned int Ned3DObjectManager::spawnTerrain(Terrain *terrain)
{
  m_terrain = new TerrainObject(terrain);
  return addObject(m_terrain, false, false, false, "Terrain");
}

unsigned int Ned3DObjectManager::spawnWater(Water *water)
{
  m_water = new WaterObject(water);
  return addObject(m_water, false, false, false, "Water");
}

unsigned int Ned3DObjectManager::spawnSilo(const Vector3 &position, const EulerAngles &orientation)
{
  static const std::string silos[] = {"Silo1","Silo2","Silo3","Silo4"};
  static int whichSilo = 0;
  m_siloModel = m_models->getModelPointer(silos[whichSilo]); // Cache silo model
  if(m_siloModel == NULL)
    return 0;  // Still NULL?  No such model
  SiloObject *silo = new SiloObject(m_siloModel);
  silo->setPosition(position);
  silo->setOrientation(orientation);
  unsigned int id = addObject(silo);
  addPhysics(silo,false);
  silo->setPPosition(position.x,position.y,position.z);

  m_furniture.insert(silo);
  whichSilo = ++whichSilo % 4;
  return id;
}

unsigned int Ned3DObjectManager::spawnWindmill(const Vector3 &position, const EulerAngles &orientation)
{
  if(m_windmillModel == NULL)
    m_windmillModel = m_models->getModelPointer("Windmill"); // Cache windmill model
  if(m_windmillModel == NULL)
    return 0;  // Still NULL?  No such model
  WindmillObject *windmill = new WindmillObject(m_windmillModel);
  windmill->setPosition(position);
  windmill->setPosition(Vector3(0,27.0f,-0.5f),1);
  windmill->setRotationSpeedBank(kPiOver2,1);
  windmill->setOrientation(orientation);
  unsigned int id = addObject(windmill);
  addPhysics(windmill,false);
  windmill->setPPosition(position.x,position.y,position.z);

  m_furniture.insert(windmill);
  return id;
}

// Returns a handle to the first enemy in the list
unsigned int Ned3DObjectManager::getEnemy()
{
  ObjectSetIter cit = m_enemys.begin();
  if (cit == m_enemys.end())
    return - 1;

  return (*cit)->getID();
}

// Returns true if a enemy intersects the ray
bool Ned3DObjectManager::rayIntersectEnemy(const Vector3 &position, const Vector3 direction)
{
  
  // Check for bullets hitting enemys
  for(ObjectSetIter cit = m_enemys.begin(); cit != m_enemys.end(); ++cit)
  {
    EnemyObject &enemy = (EnemyObject &)**cit;
    if(!enemy.isAlive()) continue;
    
    float t = enemy.getBoundingBox().rayIntersect(position,direction);
    if(t <= 1.0f) return true;        
  }

  return false;
}

void Ned3DObjectManager::deleteObject(GameObject *object)
{
  if(object == m_plane)
    m_plane = NULL;
  else if(object == m_terrain)
    m_terrain = NULL;
  else if(object == m_water)
    m_water = NULL;
  m_enemys.erase(object);
  m_bullets.erase(object);
  m_furniture.erase(object);
  m_boxes.erase(object);
  GameObjectManager::deleteObject(object);
}

bool Ned3DObjectManager::interactPlaneEnemy(PlaneObject &plane, EnemyObject &enemy)
{
  bool collided = enforcePositions(plane, enemy);
  if(collided && !enemy.isDying())
  {
    shootEnemy(enemy);
    plane.damage(1);
  }
  return collided;
}

bool Ned3DObjectManager::interactPlaneBox(PlaneObject &plane, BoxObject &box)
{
	if (m_plane == NULL ) {
		return false;
	}

  bool collided = enforcePositions(plane, box);
  if(collided)
  {
    //plane.damage(1);
  }
  return collided;
}

bool Ned3DObjectManager::interactPlaneTerrain(PlaneObject &plane, TerrainObject &terrain)
{
  Terrain *terr = terrain.getTerrain();
  if(terr == NULL) return false;

  //test for plane collision with terrain
  Vector3 planePos = plane.getPosition();
  EulerAngles planeOrient = plane.getOrientation();
  Vector3 disp = planePos - disp;
  RotationMatrix planeMatrix;
  planeMatrix.setup(plane.getOrientation()); // get plane's orientation

  float planeBottom = plane.getBoundingBox().min.y;
  float terrainHeight = terr->getHeight(planePos.x,planePos.z);
  if(plane.isPlaneAlive() && planeBottom < terrainHeight)
  { //collision
    Vector3 viewVector = planeMatrix.objectToInertial(Vector3(0,0,1));
    if(viewVector * terr->getNormal(planePos.x,planePos.z) < -0.5f // dot product
      || plane.isCrashing())
    { 
      plane.killPlane();
      int partHndl = gParticle.createSystem("planeexplosion");
      gParticle.setSystemPos(partHndl, plane.getPosition());
      int boomHndl = gSoundManager.requestSoundHandle("Boom.wav");
      int boomInst = gSoundManager.requestInstance(boomHndl);
      if(boomInst != SoundManager::NOINSTANCE)
      {
        gSoundManager.setPosition(boomHndl,boomInst,plane.getPosition());
        gSoundManager.play(boomHndl,boomInst);
        gSoundManager.releaseInstance(boomHndl,boomInst);
      }
      plane.setSpeed(0.0f);
      planePos += 2.0f * viewVector;
      planeOrient.pitch = kPi / 4.0f;
      planeOrient.bank = kPi / 4.0f;
      plane.setOrientation(planeOrient);
    }
    else planePos.y = terrainHeight + planePos.y - planeBottom;
    //plane.setPPosition(planePos);
    return true;
  }
  return false;
}

bool Ned3DObjectManager::interactPlaneWater(PlaneObject &plane, WaterObject &water)
{
  Water *pWater = water.getWater();
  if(pWater == NULL) return false;
  
  // Test for plane collision with water
  
  Vector3 planePos = plane.getPosition();
  EulerAngles planeOrient = plane.getOrientation();
  Vector3 disp = planePos - disp;
  RotationMatrix planeMatrix;
  planeMatrix.setup(plane.getOrientation()); // get plane's orientation
  float planeBottom = plane.getBoundingBox().min.y;
  float waterHeight = pWater->getWaterHeight();
  
  if(plane.isPlaneAlive() && planeBottom < waterHeight)
  { //collision
    Vector3 viewVector = planeMatrix.objectToInertial(Vector3(0,0,1));
    plane.killPlane();
    plane.setSpeed(0.0f);
    planePos += 2.0f * viewVector;
    planeOrient.pitch = kPi / 4.0f;
    planeOrient.bank = kPi / 4.0f;
    plane.setOrientation(planeOrient);
    plane.setPPosition(planePos);
    
    int partHndl = gParticle.createSystem("planeexplosion");
    gParticle.setSystemPos(partHndl, plane.getPosition());
    int boomHndl = gSoundManager.requestSoundHandle("Boom.wav");
    int boomInst = gSoundManager.requestInstance(boomHndl);
    if(boomInst != SoundManager::NOINSTANCE)
    {
      gSoundManager.setPosition(boomHndl,boomInst,plane.getPosition());
      gSoundManager.play(boomHndl,boomInst);
      gSoundManager.releaseInstance(boomHndl,boomInst);
    }
    return true;
  }
  return false;
}

bool Ned3DObjectManager::interactPlaneFurniture(PlaneObject &plane, GameObject &silo)
{
  return enforcePosition(plane, silo);
}

bool Ned3DObjectManager::interactEnemyBullet(EnemyObject &enemy, BulletObject &bullet)
{
  return bullet.checkForBoundingBoxCollision(&enemy);
}

bool Ned3DObjectManager::interactEnemyEnemy(EnemyObject &enemy1, EnemyObject &enemy2)
{
  return enforcePositions(enemy1, enemy2);
}


bool Ned3DObjectManager::interactEnemyTerrain(EnemyObject &enemy, TerrainObject &terrain)
{
  Terrain *terr = terrain.getTerrain();
  if(terr == NULL) return false;

  //test for enemy collision with terrain
  Vector3 enemyPos = enemy.getPosition();
    
  float terrainHeight = terr->getHeight(enemyPos.x,enemyPos.z);
  if (enemyPos.y < terrainHeight)
  {
    enemyPos.y = terrainHeight;
    //enemy.setPPosition(enemyPos);       
    int tmpHndl = gParticle.createSystem("enemyfeatherssplat");
    gParticle.setSystemPos(tmpHndl, enemyPos);

    int thumpSound = gSoundManager.requestSoundHandle("Thump.wav");
    int instance = gSoundManager.requestInstance(thumpSound);
     if(thumpSound != SoundManager::NOINSTANCE)
  {
    gSoundManager.setPosition(thumpSound,instance,enemyPos);
    gSoundManager.play(thumpSound,instance);
    gSoundManager.releaseInstance(thumpSound,instance);
  }
    
    enemy.killObject();

    return true;
  }
  return false;
}

void Ned3DObjectManager::shootEnemy(EnemyObject &enemy)
{
  int tmpHndl = gParticle.createSystem("enemyfeathers");
  Vector3 enemyPos = enemy.getPosition();
  gParticle.setSystemPos(tmpHndl, enemyPos);
  int deathSound = gSoundManager.requestSoundHandle("enemydeath.wav");
  int deathInstance = gSoundManager.requestInstance(deathSound);
  if(deathInstance != SoundManager::NOINSTANCE)
  {
    gSoundManager.setPosition(deathSound,deathInstance,enemyPos);
    gSoundManager.play(deathSound,deathInstance);
    gSoundManager.releaseInstance(deathSound,deathInstance);
  }
  enemy.setDying();
}

bool Ned3DObjectManager::enforcePosition(GameObject &moving, GameObject &stationary)
{
	// Bullet stuff:
	//collisionWorld->contactPairTest(moving.colOb,stationary.colOb, *renderCallback);
	/*
	btVector3 v(0.0f,0.0f,0.0f);
	const btConvexPolyhedron* cP = NULL;
	const btConvexPolyhedron* cP2 = NULL;
	if ( moving.colOb->getCollisionShape()->isPolyhedral() ) {
		btPolyhedralConvexShape* pCS = ((btPolyhedralConvexShape*) moving.colOb->getCollisionShape());
		//pCS->initializePolyhedralFeatures();  // TODO wasn't doing this on startup, now I am
		cP = pCS->getConvexPolyhedron();
	}
	if (stationary.colOb->getCollisionShape()->isPolyhedral() ) {
		btPolyhedralConvexShape* pCS = ((btPolyhedralConvexShape*) stationary.colOb->getCollisionShape());
		//pCS->initializePolyhedralFeatures(); // TODO wasn't doing this on startup, now I am
		cP2 = pCS->getConvexPolyhedron();
	}

	bool collision = false;
	if (cP && cP2) {
	    collision = btPolyhedralContactClipping::findSeparatingAxis(*cP,*cP2,
		moving.colOb->getWorldTransform(),stationary.colOb->getWorldTransform(),
		v,renderCallback);
	}

	btVector3 v2 = moving.colOb->getWorldTransform().getOrigin();

	if (collision) {
		char* tp = (char*)malloc(sizeof(char*)*500); // allocate char
		sprintf_s(tp,500,"hello world %f %f %f\n\0",v2.getX(),v2.getY(),v2.getZ());
		OutputDebugString(tp);

		delete tp;
	}
	return collision;*/
  
  const AABB3 &box1 = moving.getBoundingBox(), &box2 = stationary.getBoundingBox();
  AABB3 intersectBox;
  if(AABB3::intersect(box1, box2, &intersectBox))
  {
    // Collision:  Knock back obj1
    //   - Kludge method:  Push back on smallest dimension
    Vector3 delta = intersectBox.size();
    Vector3 obj1Pos = moving.getPosition(), obj2Pos = stationary.getPosition();
    if(delta.x <= delta.y)
      if(delta.x <= delta.z)
      {
        // Push back on x
        obj1Pos.x += (box1.min.x < box2.min.x) ? -delta.x : delta.x;
      }
      else
      {
        // Push back on z
        obj1Pos.z += (box1.min.z < box2.min.z) ? -delta.z : delta.z;
      }
    else if(delta.y <= delta.z)
    {
      // Push back on y
      obj1Pos.y += (box1.min.y < box2.min.y) ? -delta.y : delta.y;
    }
    else
    {
      // Push back on z
      obj1Pos.z += (box1.min.z < box2.min.z) ? -delta.z : delta.z;
    }
    moving.setPPosition(obj1Pos);
    return true;
  }
  return false;
}


void Ned3DObjectManager::setNextBox(BoxObject* b, float wall) {

  

  if (m_boxesToAdd.empty()) {
	  deleteObject(b);
	  return;
  }
  for(ObjectSetIter bit = m_boxesToAdd.begin(); bit != m_boxesToAdd.end(); ++bit) {
	BoxObject &box = (BoxObject &)**bit;

	Vector3 pos = m_plane->getPosition();
	if (pos.x <= box.right && pos.x >= box.left &&
		pos.y <= box.top && pos.y >= box.bottom &&
		pos.z <= box.back && pos.z >= box.front) {

		;

		// hard coded wall conditions:
		if (abs(box.back - 1200.0f) + abs(box.right - 300.0f) < 0.01f && m_startingEnemies - m_enemys.size() == 0) {
			continue;
		}
		if (abs(box.left + 1000.0f) + abs(box.front - 500.0f) < 0.01f) {
			m_plane->m_bAllRange = true;
		}



		char* tp = (char*)malloc(sizeof(char*)*500); // allocate char
		sprintf_s(tp,500,"In with the new!\n\0");
		OutputDebugString(tp);
		// process box because it is suitable to be the next one

		// set plane
		switch (box.l) {
		case BoxObject::LEFT:
			m_plane->m_headingState = PlaneObject::HS_LEFT;
			m_plane->m_eaOrient->pitch = 0.0f;
			break;
		case BoxObject::RIGHT:
			m_plane->m_headingState = PlaneObject::HS_RIGHT;
			m_plane->m_eaOrient->pitch = 0.0f;
			break;
		case BoxObject::TOP:
			m_plane->m_eaOrient->heading = 0.0f;
			m_plane->m_eaOrient->pitch = (float)M_PI/2;
			break;
		case BoxObject::BOTTOM:
			m_plane->m_eaOrient->heading = 0.0f;
			m_plane->m_eaOrient->pitch = (float)-M_PI/2;
			break;
		case BoxObject::BACK:
			m_plane->m_headingState = PlaneObject::HS_STRAIGHT;
			m_plane->m_eaOrient->pitch = 0.0f;
			break;
		case BoxObject::FRONT:
			m_plane->m_eaOrient->heading = (float)M_PI;
			m_plane->m_eaOrient->pitch = 0.0f;
			break;
		default:
			break;
		}

		// set boxes

		deleteObject(*m_boxes.begin());
		m_boxes.clear();
		m_boxes.insert(*bit);
		m_boxesToAdd.erase(bit);

		break;  // only want one box at a time!
	}
  }  //Old physics code */
}

bool Ned3DObjectManager::enforcePositions(GameObject &obj1, GameObject &obj2) 
{	 // TODO: replace with http://code.google.com/p/bullet/source/browse/trunk/src/BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.cpp HullAgainstHull for OOBBs
	/*
	// Bullet stuff:
	
	btVector3 v(0.0f,0.0f,0.0f);
	const btConvexPolyhedron* cP = NULL;
	const btConvexPolyhedron* cP2 = NULL;
	if ( obj1.colOb->getCollisionShape()->isPolyhedral() ) {
		btPolyhedralConvexShape* pCS = ((btPolyhedralConvexShape*) obj1.colOb->getCollisionShape());
		//pCS->initializePolyhedralFeatures();
		cP = pCS->getConvexPolyhedron();
	}
	if (obj2.colOb->getCollisionShape()->isPolyhedral() ) {
		btPolyhedralConvexShape* pCS = ((btPolyhedralConvexShape*) obj2.colOb->getCollisionShape());
		//pCS->initializePolyhedralFeatures();
		cP2 = pCS->getConvexPolyhedron();
	}

	bool collision = false;

	if (cP && cP2) {
	    collision = btPolyhedralContactClipping::findSeparatingAxis(*cP,*cP2,
		obj1.colOb->getWorldTransform(),obj2.colOb->getWorldTransform(),
		v,renderCallback);
	}

	btVector3 v1 = obj1.colOb->getWorldTransform().getOrigin();
	btVector3 v2 = obj2.colOb->getWorldTransform().getOrigin();


	bool invert = false;
    if (obj1.getType() == ObjectTypes::BOX || obj2.getType() == ObjectTypes::BOX) {
	    invert = true;
    }
	if (collision) {
		if (!invert) {
			//normal collision stuff
			
		} else {

		}
	}
	return collision;*/

  bool invert = false;
  if (obj1.getType() == ObjectTypes::BOX || obj2.getType() == ObjectTypes::BOX) {
	  invert = true;
  }

  const AABB3 &box1 = obj1.getBoundingBox(), &box2 = obj2.getBoundingBox();
  AABB3 intersectBox;

  if(! invert && AABB3::intersect(box1, box2, &intersectBox))
  {
    // Collision:  Knock back both objects
    //   - Kludge method:  Push back on smallest dimension
    Vector3 delta = intersectBox.size();
    Vector3 obj1Pos = obj1.getPosition(), obj2Pos = obj2.getPosition();
	
	if(delta.x <= delta.y)
		if(delta.x <= delta.z)
		{
		// Push back on x
		float dx = (box1.min.x < box2.min.x) ? -delta.x : delta.x;
		obj1Pos.x += dx;
		obj2Pos.x -= dx;
		}
		else
		{
		// Push back on z
		float dz = (box1.min.z < box2.min.z) ? -delta.z : delta.z;
		obj1Pos.z += dz;
		obj2Pos.z -= dz;
		}
	else if(delta.y <= delta.z)
	{
		// Push back on y
		float dy = (box1.min.y < box2.min.y) ? -delta.y : delta.y;
		obj1Pos.y += dy;
		obj2Pos.y -= dy;
	}
	else
	{
		// Push back on z
		float dz = (box1.min.z < box2.min.z) ? -delta.z : delta.z;
		obj1Pos.z += dz;
		obj2Pos.z -= dz;
	}
	
    obj1.setPPosition(obj1Pos);
    obj2.setPPosition(obj2Pos);
    return true;
  } else if (AABB3::intersect(box1, box2, &intersectBox) ) { // inverted bounding box collision)

	char* tp = (char*)malloc(sizeof(char*)*500); // allocate char
	sprintf_s(tp,500,"hello world %f\n\0",box1.min.x);

	 

	Vector3 delta = intersectBox.size();
    Vector3 obj1Pos = obj1.getPosition(), obj2Pos = obj2.getPosition();
	float dx,dy,dz=0.0f;
	dx=dy=dz=0.0f;


	// check if plane should leave the box
	GameObject* p = &obj2;
	BoxObject* b = (BoxObject*)p;
	p = &obj1;
	PlaneObject* plane = (PlaneObject*)p;

	bool posChanged = false;

	// since one is much bigger than the other, we can use maxes to determine relative position:
	
	if (box1.min.x < box2.min.x + 1.0f) {
		// move obj2 to the left and move obj1 to the right
		dx = box2.min.x - box1.min.x;
		//obj1Pos.x += dx;

		posChanged = true;
		plane->m_stopState = plane->m_stopState | PlaneObject::STOP_LEFT;
		//OutputDebugString(tp);
		if (b->l == BoxObject::LEFT) {
			setNextBox(b,b->left);
			return true;
		}
	}
	
	if (box1.min.x < box2.min.x) {
		// move obj2 to the left and move obj1 to the right
		dx = box2.min.x - box1.min.x;
		obj1Pos.x += dx;

		posChanged = true;
		plane->m_stopState = plane->m_stopState | PlaneObject::STOP_LEFT;
		//OutputDebugString(tp);
		if (b->l == BoxObject::LEFT) {
			setNextBox(b,b->left);
			return true;
		}
	}

	if (box1.max.x + 1.0f > box2.max.x) {
		dx = box2.max.x - box1.max.x;
		//obj1Pos.x += dx;
		//OutputDebugString(tp);

		posChanged = true;
		plane->m_stopState = plane->m_stopState | PlaneObject::STOP_RIGHT;
		if (b->l == BoxObject::RIGHT) {
			setNextBox(b,b->right);
			return true;
		}
	}

	if (box1.max.x > box2.max.x) {
		dx = box2.max.x - box1.max.x;
		obj1Pos.x += dx;
		//OutputDebugString(tp);

		posChanged = true;
		plane->m_stopState = plane->m_stopState | PlaneObject::STOP_RIGHT;
		if (b->l == BoxObject::RIGHT) {
			setNextBox(b,b->right);
			return true;
		}
	}

	if (box1.min.y < box2.min.y + 1.0f) {
		// move obj2 to the left and move obj1 to the right
		dy = box2.min.y - box1.min.y;
		//obj1Pos.y += dy;
		//OutputDebugString(tp);

		posChanged = true;
		plane->m_stopState = plane->m_stopState | PlaneObject::STOP_DOWN;
		if (b->l == BoxObject::BOTTOM) {
			setNextBox(b,b->bottom);
			return true;
		}
	}

	if (box1.min.y < box2.min.y) { // TODO: Make plane not able to go slightly over
		// move obj2 to the left and move obj1 to the right
		dy = box2.min.y - box1.min.y;
		obj1Pos.y += dy;
		//OutputDebugString(tp);

		posChanged = true;
		plane->m_stopState = plane->m_stopState | PlaneObject::STOP_DOWN;
		if (b->l == BoxObject::BOTTOM) {
			setNextBox(b,b->bottom);
			return true;
		}
	}

	if (box1.max.y + 1.0f > box2.max.y) {
		dy = box2.max.y - box1.max.y;
		//obj1Pos.y += dy;
		//OutputDebugString(tp);

		posChanged = true;
		plane->m_stopState = plane->m_stopState | PlaneObject::STOP_UP;
		if (b->l == BoxObject::TOP) {
			setNextBox(b,b->top);
			return true;
		}
	}

	if (box1.max.y > box2.max.y) {
		dy = box2.max.y - box1.max.y;
		obj1Pos.y += dy;
		//OutputDebugString(tp);

		posChanged = true;
		plane->m_stopState = plane->m_stopState | PlaneObject::STOP_UP;
		if (b->l == BoxObject::TOP) {
			setNextBox(b,b->top);
			return true;
		}
	}


	if (box1.min.z < box2.min.z + 1.0f) {
		// move obj2 to the left and move obj1 to the right
		dz = box2.min.z - box1.min.z;
		//obj1Pos.z += dz;
		
		posChanged = true;
		plane->m_stopState = plane->m_stopState | PlaneObject::STOP_NEAR;
		if (b->l == BoxObject::FRONT) {
			setNextBox(b,b->back);
			return true;
		}
	}

	if (box1.min.z < box2.min.z) {
		// move obj2 to the left and move obj1 to the right
		dz = box2.min.z - box1.min.z;
		obj1Pos.z += dz;
		
		posChanged = true;
		plane->m_stopState = plane->m_stopState | PlaneObject::STOP_NEAR;
		if (b->l == BoxObject::FRONT) {
			setNextBox(b,b->back);
			return true;
		}
	}

	if (box1.max.z + 1.0f > box2.max.z) {
		dz = box2.max.z - box1.max.z;
		//obj1Pos.z += dz;

		//char* tp = (char*)malloc(sizeof(char*)*500); // allocate char
		//sprintf_s(tp,500,"In with the new?????\n\0");
		//OutputDebugString(tp);

		posChanged = true;
		plane->m_stopState = plane->m_stopState | PlaneObject::STOP_FAR;
		if (b->l == BoxObject::BACK) {
			setNextBox(b,b->front);
			return true;
		}
	}

	if (box1.max.z > box2.max.z) {
		dz = box2.max.z - box1.max.z;
		obj1Pos.z += dz;



		posChanged = true;
		plane->m_stopState = plane->m_stopState | PlaneObject::STOP_FAR;
		if (b->l == BoxObject::BACK) {
			setNextBox(b,b->front);
			return true;
		}
	}

	if (!posChanged) {
		// No walls hit, so plane can move normally
		plane->m_stopState = PlaneObject::NO_STOP;
	}

	
    obj1.setPPosition(obj1Pos);


	



	return true;
  }

  return false;
}