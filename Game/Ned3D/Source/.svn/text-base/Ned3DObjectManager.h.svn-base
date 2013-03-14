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

/// \file Ned3DObjectManager.h
/// \brief Defines the object management class for Ned3D

/// Last updated July 12, 2005

#ifndef __NED3DOBJECTMANAGER_H_INCLUDED__
#define __NED3DOBJECTMANAGER_H_INCLUDED__

#include "Common/Vector3.h"
#include "Common/EulerAngles.h"
#include "Objects/GameObjectManager.h"
#include "ObjectTypes.h"
#include "ColorObject.h"



//struct for bullet stuff:

struct btIntermediateResult : public btDiscreteCollisionDetectorInterface::Result
	{

		btIntermediateResult():m_hasResult(false)
		{
		}
		
		btVector3 m_normalOnBInWorld;
		btVector3 m_pointInWorld;
		btScalar m_depth;
		bool	m_hasResult;

		virtual void setShapeIdentifiersA(int partId0,int index0)
		{
			(void)partId0;
			(void)index0;
		}
		virtual void setShapeIdentifiersB(int partId1,int index1)
		{
			(void)partId1;
			(void)index1;
		}
		void addContactPoint(const btVector3& normalOnBInWorld,const btVector3& pointInWorld,btScalar depth)
		{
			m_normalOnBInWorld = normalOnBInWorld;
			m_pointInWorld = pointInWorld;
			m_depth = depth;
			m_hasResult = true;
		}
	};

class Model;
class ModelManager;
class PlaneObject;
class EnemyObject;
class BulletObject;
class TerrainObject;
class WaterObject;
class SiloObject;
class WindmillObject;
class Terrain;
class Water;

/// \brief Derived object manager to handle Ned3D objects specifically.
class Ned3DObjectManager : public GameObjectManager
{
    

	


  public:
    
	btIntermediateResult renderCallback;

	ObjectSet m_boxes; ///> Tracks which "walls" are up
	ObjectSet m_boxesToAdd; ///> Tracks which "walls" are down, but ready to be added

    Ned3DObjectManager(); ///< Constructs a Ned3DObjectManager.
	~Ned3DObjectManager(); ///< Destructs physics stuff


	float* getBounds(); /// gets bounds for current wall

    /// \brief Tells the object manager which model manager to use.
    /// \param models The model manager to use for the objects.
    void setModelManager(ModelManager &models);
  
    virtual void clear(); ///< Clears the object manager, removing all objects.
    virtual void handleInteractions(); ///< Handles the interactions between all the objects.

	bool importXml(const std::string &fileName, bool defaultDirectory=true);

    /// \brief Spawns a plane object.
    /// \param position Position to place the object.
    /// \param orientation Initial orientation of the object.
    /// \return The unique ID of the new object.
    unsigned int spawnPlane(const Vector3 &position = Vector3::kZeroVector, const EulerAngles &orientation = EulerAngles::kEulerAnglesIdentity);

    /// \brief Spawns a enemy object.
    /// \param position Position to place the object.
    /// \param orientation Initial orientation of the object.
    /// \param speed Initial speed of the object.
    /// \return The unique ID of the new object.
    unsigned int spawnEnemy(const Vector3 &position = Vector3::kZeroVector, const EulerAngles &orientation = EulerAngles::kEulerAnglesIdentity, float speed = 0.0f);

    /// \brief Spawns a enemy object.
    /// \param position Position to place the object.
    /// \param circleCenter The center of the circle along which the object will move
    /// \param speed Initial speed of the object.
    /// \param flyLeft Whether the object should move counter-clockwise around the circle.
    /// \return The unique ID of the new object.
    unsigned int spawnEnemy(const Vector3 &position, const Vector3 &circleCenter, float speed = 0.0f, bool flyLeft = true);

    /// \brief Spawns a bullet object.
    /// \param position Position to place the object.
    /// \param orientation Initial orientation of the object.
    /// \return The unique ID of the new object.
    unsigned int spawnBullet(const Vector3 &position = Vector3::kZeroVector, const EulerAngles &orientation = EulerAngles::kEulerAnglesIdentity, const Color color = RED);

    /// \brief Spawns a terrain object.
    /// \param terrain The terrain data
    /// \return The unique ID of the new object.
    unsigned int spawnTerrain(Terrain *terrain);

	/// Spawns a box object, which acts as a wall for m_bounded objects
	unsigned int spawnBox(float t, float b, float l, float r,float back = -1000000.0f, float f= 1000000.0f, char* leaving="far");

    /// \brief Spawns a water object.
    /// \param water The water data
    /// \return The unique ID of the new object.
    unsigned int spawnWater(Water *water);

    /// \brief Spawns a silo object.
    /// \param position Position to place the object.
    /// \param orientation Initial orientation of the object.
    /// \return The unique ID of the new object.
    unsigned int spawnSilo(const Vector3 &position = Vector3::kZeroVector, const EulerAngles &orientation = EulerAngles::kEulerAnglesIdentity);

    /// \brief Spawns a windmill object.
    /// \param position Position to place the object.
    /// \param orientation Initial orientation of the object.
    /// \return The unique ID of the new object.
    unsigned int spawnWindmill(const Vector3 &position = Vector3::kZeroVector, const EulerAngles &orientation = EulerAngles::kEulerAnglesIdentity);

    unsigned int getEnemy(); ///< Returns an index to the first enemy in the list.

    TerrainObject *getTerrainObject() { return m_terrain; } ///< Returns the pointer to the terrain object
    WaterObject *getWaterObject() { return m_water; } ///< Returns the pointer to the water object

    /// \brief Returns true if a enemy intersects the ray.
    /// \param position Starting point of the bullet object.
    /// \param direction Direction of the bullet object.
    bool rayIntersectEnemy(const Vector3 &position, const Vector3 direction);

    /// \brief Returns the pointer to the plane object.
    /// \return Pointer to the plane object.
    PlaneObject *getPlaneObject() { return m_plane; }
    
    /// \brief Deletes a particular object.
    /// \param object Pointer to the object to be deleted.
    virtual void deleteObject(GameObject *object);

  protected:
	bool interactPlaneBox(PlaneObject &plane, BoxObject &box); // Handles plane-wall interactions
    bool interactPlaneEnemy(PlaneObject &plane, EnemyObject &enemy); ///< Handles plane-enemy interactions, such as collision
    bool interactPlaneTerrain(PlaneObject &plane, TerrainObject &terrain); ///< Handles possible plane-terrain collision
    bool interactPlaneWater(PlaneObject &plane, WaterObject &water); ///< Handles possible plane-water collision
    bool interactPlaneFurniture(PlaneObject &plane, GameObject &furniture); ///< Handles possible plane-furniture collision
    bool interactEnemyEnemy(EnemyObject &enemy1, EnemyObject &enemy2); ///< Handles enemy-enemy interactions, such as possible collision
    bool interactEnemyTerrain(EnemyObject &enemy, TerrainObject &terrain); ///< Handles enemy-terrain interactions, such as possible collision
    bool interactEnemyBullet(EnemyObject &enemy, BulletObject &bullet); ///< Handles possible enemy-bullet collision
    
	void setNextBox(BoxObject* b, float wall); ///< Sets next wall

    void shootEnemy(EnemyObject &enemy); ///< Handles enemy-bullet collision
    
    bool enforcePosition(GameObject &moving, GameObject &stationary); ///< Blocks a moving object from intersecting a stationary object.
    bool enforcePositions(GameObject &obj1, GameObject &obj2); ///< Blocks two moving objects from intersecting each other.
    
	int m_startingEnemies;

    // Ned3D-specific references
    
    ModelManager *m_models; ///< Pointer to the model manager to be used when creating objects.

    Model *m_planeModel; ///< Pointer to the plane model.
    Model *m_enemyModel; ///< Pointer to the enemy model.
    Model *m_bulletModel; ///< Pointer to the bullet model.
    Model *m_siloModel; ///< Pointer to the silo model.
    Model *m_windmillModel; ///< Pointer to the windmill model.

    PlaneObject *m_plane;  ///> Points to the sole plane object.
    ObjectSet m_enemys;     ///> Tracks the very evil enemys
    ObjectSet m_bullets;   ///> Tracks bullets
    TerrainObject *m_terrain; ///> Points to the sole terrain object.  (not owned)
    WaterObject *m_water; ///> Points to the sole water object.  (not owned)
    ObjectSet m_furniture; ///> Silos, windmills, etc.
};


#endif

