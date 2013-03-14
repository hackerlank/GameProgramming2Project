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

/// \file tethercamera.cpp
/// \brief Code for the TetherCamera class.

/////////////////////////////////////////////////////////////////////////////
//
// 3D Math Primer for Games and Graphics Development
//
// Camera.cpp - camera utilities
//
// Visit gamemath.com for the latest version of this file.
//
/////////////////////////////////////////////////////////////////////////////

#define _USE_MATH_DEFINES

#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "common/renderer.h"
#include "common/RotationMatrix.h"
#include "DerivedCameras/tethercamera.h"
#include "objects/gameobjectmanager.h"
#include "objects/gameobject.h"
#include "assert.h"
#include "../../Ned3D/Source/Ned3DObjectManager.h"

enum TurnState
{
TS_STRAIGHT, TS_LEFT, TS_RIGHT,
};

/// \param objectManager Pointer to the object manager.  This is needed
/// to look up the location of the target object.
TetherCamera::TetherCamera (GameObjectManager* objectManager):
  m_objects(objectManager),
  rightCounter(0)
{  
}

// Resets camera to behind target object
void TetherCamera::reset() 
{
  Vector3 target;
  float targetHeading;

  assert(m_objects); // object manager doesn't exist
  GameObject* obj = m_objects->getObjectPointer(m_targetObjectID);    
  
  assert(obj); // object to follow doesn't exist  
  targetHeading = obj->getOrientation().heading;
  target = obj->getPosition();

  
  target.y += 3.0f;

  cameraOrient.set(targetHeading, 0.0f,0.0f);

  RotationMatrix cameraMatrix;
  cameraMatrix.setup(cameraOrient);

  Vector3 bOffset(0.0f,0.0f, -maxDist);
  
  Vector3 iOffset = cameraMatrix.objectToInertial(bOffset);
  cameraPos = target + iOffset;
}

// brief Specfies a target object
/// \param objectID Object ID that can be used to retreive the target object
/// from the object manager.
void TetherCamera::setTargetObject(unsigned int objectID)
{
 m_targetObjectID = objectID;
 return;
}

// processes movement of the camera
/// \param elapsed The elapsed time in seconds since the last call to this
/// function.
void TetherCamera::process(float elapsed) 
{ 
  float targetHeading;
  Vector3 target;

  assert(m_objects); // check object manager  
  GameObject* obj = m_objects->getObjectPointer(m_targetObjectID);
  assert(obj); // check object
  targetHeading = obj->getOrientation().heading;
  target = obj->getPosition();

  float* floats = ((Ned3DObjectManager*)m_objects)->getBounds();
  float front = floats[5];
  float back = floats[4];
  float right = floats[3];
  float left = floats[2];
  float top = floats[1];
  float bottom = floats[0];

  target.y += 3.0f;

  // Compute current vector to target
  Vector3 iDelta = target - cameraPos;

  // Compute current distance
  float dist = iDelta.magnitude();

  // Get normalized direction vector
  Vector3 iDir = iDelta / dist;
  


  GameObject* plane = m_objects->getObjectPointer("Plane");
  EulerAngles orientation = plane->getOrientation();
  Vector3 Change(sin(orientation.heading),0,cos(orientation.heading));
  
  char* tp = (char*)malloc(sizeof(char*)*500); // allocate char
  sprintf_s(tp,500,"%f %f\n\0",Change.x,Change.z);

  

  if (plane->m_bAllRange) {
	  
	  
	  float cspeed = .05f;
	  cameraPos = ((cspeed-elapsed)/cspeed)*cameraPos + (elapsed/cspeed)*(plane->getPosition() - 50.0f*Change);
	  Vector3 horizontal = Vector3::crossProduct(iDelta,Vector3(0.0f,1.0f,0.0f)); // horizontal vector for moving camera

  }

  // Clamp within desired range to get distance
  // for next frame
  if (dist < minDist)
    dist = minDist;
  
  if (dist > maxDist) 
    dist = maxDist;

  float zoom = 1.0f; // a kludge I added in to zoom out
  // Recompute difference vector
  iDelta = iDir * dist * zoom; 


  int axis = 0; // 0 is x, 1 is z
  // Compute camera's position

  if (! m_objects->getObjectPointer("Plane")->m_bAllRange) {
	  minDist = 40.0f;
	  maxDist = 50.0f;
	  float followX = 15.0f;  // how far away before the camera starts following closer?  inf originally
	  if ( iDelta.x > followX) {
		  iDelta.x = followX;
	  } else if (iDelta.x < -followX) {
		  iDelta.x = -followX;
	  }

	  float heading = plane->getOrientation().heading;

		// heading left so do left camera stuff
		if ( cameraOrient.heading - heading > .03f  ) {
			// camera isn't in the direction we are turning, so we turn it step by step
			cameraOrient.heading -= .02f;
		} else if ( cameraOrient.heading - heading < -.03f  ) {
			// camera isn't in the direction we are turning, so we turn it step by step
			cameraOrient.heading += .02f;
		} else {
			// camera is close, but fix it to be sure
			cameraOrient.heading = plane->getOrientation().heading;
		}
	  
	  if ( abs(plane->getOrientation().heading - M_PI/2) < 0.1f) {
		  // heading right so do left camera stuff
		  left = back;
		  right = front;
		  axis = 1;
	  } else if ( abs(plane->getOrientation().heading + M_PI/2) < 0.1f) {
		  // heading left so do right camera stuff
		  left = front;
		  right = back;
		  axis = 1;
	  }

	  float closeToWall = abs((right+left)/4); // how close to wall before camera doesnt move?  0.0f originally

	  float d = 0.0f;
	  if (axis == 1) {
		  d = min(abs(target.z-right),abs(target.z-left));
	  } else {
		  d = min(abs(target.x-right),abs(target.x-left));
	  }
	  
	  closeToWall = abs(((right-left)/2) );

	  if ( d < closeToWall) {

		  float originalX = cameraPos.x;
		  float cspeed = .05f;

		  

		  cameraPos = ((cspeed-elapsed)/cspeed)*cameraPos + (elapsed/cspeed)*(plane->getPosition() - 50.0f*Change);
		  Vector3 horizontal = Vector3::crossProduct(iDelta,Vector3(0.0f,1.0f,0.0f)); // horizontal vector for moving camera
		  
		  if (axis == 0 && plane->getPosition().z < cameraPos.z) {
			  cameraPos.z -= 2*abs(cameraPos.z-plane->getPosition().z);
		  }

		  //cameraPos.x = originalX;
		  //OutputDebugString(tp);  debug stuff
		  //cameraPos.x = (cameraPos.x-left)/((right+left)/2-left)*(left-closeToWall);
		  
		  float Kludgefactor = 0.7f + abs(cameraPos.x)/50.0f;					// plane hits the wall at around e.  Coincidence?  I don't think so.
		  if (axis==1) {
			  Kludgefactor = 0.7f + abs(cameraPos.z)/50.0f;
		  }

		  Kludgefactor = abs((right-left)/2)*Kludgefactor;	// make it based on dist from left to right
		  
		  float angleKludge = ((float)M_PI);
		  angleKludge = angleKludge-(log(d+1.0f)/log(closeToWall+1.0f))*angleKludge;

		  if (axis==1) {
			  if ( abs(plane->getOrientation().heading + M_PI/2) < 0.1f) { // going left
				  if (abs(target.z-right) < closeToWall) {
					  //if (right < 0) closeToWall = -closeToWall;
					  float component1 = 0.0f;//log(closeToWall-d+Kludgefactor)/log(closeToWall+Kludgefactor)*(right-closeToWall);
					  if (cameraPos.z < 0) {
						  cameraPos.z =  /*component1 + */log(closeToWall+Kludgefactor)/log(d+Kludgefactor)*cameraPos.z;  // fix the x
					  } else {
						cameraPos.z =  /*component1 + */log(d+Kludgefactor)/log(closeToWall+Kludgefactor)*cameraPos.z;  // fix the x
					  }
				  } else {
					  //if (left > 0) closeToWall = -closeToWall;
						//cameraOrient.set(cameraOrient.heading+angleKludge,cameraOrient.bank,cameraOrient.pitch);
					  if (cameraPos.z < 0) {
						  cameraPos.z =  /*component1 + */log(d+Kludgefactor)/log(closeToWall+Kludgefactor)*cameraPos.z;  // fix the x
					  } else {
						  cameraPos.z =  /*component1 + */log(closeToWall+Kludgefactor)/log(d+Kludgefactor)*cameraPos.z;  // fix the x
					  }
				  }
			  } else { // going right -- invert
				  if (abs(target.z-right) < closeToWall) {
					  //if (right < 0) closeToWall = -closeToWall;
					  float component1 = 0.0f;//log(closeToWall-d+Kludgefactor)/log(closeToWall+Kludgefactor)*(right-closeToWall);
					  if (cameraPos.z < 0) {
						  cameraPos.z =  /*component1 + */log(d+Kludgefactor)/log(closeToWall+Kludgefactor)*cameraPos.z;  // fix the x
					  } else {
						  cameraPos.z =  /*component1 + */log(closeToWall+Kludgefactor)/log(d+Kludgefactor)*cameraPos.z;  // fix the x
						
					  }
				  } else {
					  //if (left > 0) closeToWall = -closeToWall;
						//cameraOrient.set(cameraOrient.heading+angleKludge,cameraOrient.bank,cameraOrient.pitch);
					  if (cameraPos.z < 0) {
						  cameraPos.z =  /*component1 + */log(closeToWall+Kludgefactor)/log(d+Kludgefactor)*cameraPos.z;  // fix the x
					  } else {
						  cameraPos.z =  /*component1 + */log(d+Kludgefactor)/log(closeToWall+Kludgefactor)*cameraPos.z;  // fix the x
					  }
				  }
			  }

		  } else {

			  if (abs(target.x-right) < closeToWall) {
				  //if (right < 0) closeToWall = -closeToWall;
				  float component1 = 0.0f;//log(closeToWall-d+Kludgefactor)/log(closeToWall+Kludgefactor)*(right-closeToWall);
				  if (cameraPos.x < 0) {
					  cameraPos.x =  /*component1 + */log(closeToWall+Kludgefactor)/log(d+Kludgefactor)*cameraPos.x;  // fix the x
				  } else {
					cameraPos.x =  /*component1 + */log(d+Kludgefactor)/log(closeToWall+Kludgefactor)*cameraPos.x;  // fix the x
				  }
			  } else {
				  //if (left > 0) closeToWall = -closeToWall;
					//cameraOrient.set(cameraOrient.heading+angleKludge,cameraOrient.bank,cameraOrient.pitch);
				  if (cameraPos.x < 0) {
					  cameraPos.x =  /*component1 + */log(d+Kludgefactor)/log(closeToWall+Kludgefactor)*cameraPos.x;  // fix the x
				  } else {
					  cameraPos.x =  /*component1 + */log(closeToWall+Kludgefactor)/log(d+Kludgefactor)*cameraPos.x;  // fix the x
				  }
			  }

		  }
	  } else { 
		  //cameraPos = target - iDelta;
		  float cspeed = .05f;
		  cameraPos = ((cspeed-elapsed)/cspeed)*cameraPos + (elapsed/cspeed)*(plane->getPosition() - 50.0f*Change);
		  Vector3 horizontal = Vector3::crossProduct(iDelta,Vector3(0.0f,1.0f,0.0f)); // horizontal vector for moving camera
	  }

	  // Compute heading/pitch to look in given direction
	   
  } else {
	  minDist = 40.0f;
	  maxDist = 50.0f;
	  // Compute camera's position
	  //cameraPos = target - iDelta;

	  // Compute heading/pitch to look in given direction
	  cameraOrient.heading = atan2(iDir.x, iDir.z);
  }

  
  cameraOrient.pitch = -asin(iDir.y);
  cameraOrient.bank = 0.0f;
}