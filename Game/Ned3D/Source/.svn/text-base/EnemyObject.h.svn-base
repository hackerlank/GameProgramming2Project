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

/// \file EnemyObject.h
/// \brief Interface for the EnemyObject.

/// Edited by the SAGE team, 2005
/// Last updated June 13, 2006
#ifndef __ENEMYOBJECT_H_INCLUDED__
#define __ENEMYOBJECT_H_INCLUDED__

#include "ColorObject.h"
#include "Objects/GameObject.h"

/// \brief Represents a enemy object.
class EnemyObject : public ColorObject
{
public:
  friend class Ned3DObjectManager;

  /// Represent the different movement patterns.
  enum MovementPattern
  {
    MP_HOVER,      ///< Enemy hovers in place
    MP_STRAIGHT,   ///< Enemy flies straight forward
    MP_CIRCLING    ///< Enemy circles a point    
  };
  
  /// Represent the different behavior states.
  enum BehaviorState
  {
    BS_CRUISING,    ///< Enemy is flying, unaware of anything
    BS_DYING        ///< Enemy is falling to ground
  };

  EnemyObject(Model *m);  ///< Constructs a enemy using the given model.
  ~EnemyObject();         ///< Destroys the enemy.
  
  // Standard frame functions
  
  virtual void process(float dt); ///< Processes enemy game logic.
  virtual void move(float dt); ///< Handles moving the enemy.
  
  
  // Enemy-specific functions
  
  void setMovementPattern(MovementPattern pattern);
  void setCirclingParameters(const Vector3 &circleCenter, bool flyLeft);
  void setCirclingParameters(const Vector3 &circleCenter, bool flyLeft, float circleRadius);

  void setDying(); ///< Called when enemy is to fall to the ground and die.
  bool isDying(); ///< Returns true if the enemy is falling to the ground

  void killObject(); ///< Kill the enemy specifically

protected:
  
  // Protected members
  
  BehaviorState m_behavior; ///< Enemy behaviour state.
  MovementPattern m_movement; ///< Movement pattern.

  float m_colorTimer;

  int m_dyingFeatherTrail; ///< Dying handle to the particle effect of the feather trail.

  Vector3 m_v3Velocity; ///< Holds the velocity of the object.
  
  // Movement parameters
  Vector3 m_circleCenter; ///< Point about which to circle.
  bool m_circleLeft; ///< Whether to circle clockwise or counter-clockwise.
  float m_circleRadius; ///< Radius of the circle that the enemy makes.
  
};

#endif