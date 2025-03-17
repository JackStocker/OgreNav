/*
    OgreCrowd
    ---------

    Copyright (c) 2012 Jonas Hauquier

    Additional contributions by:

    - mkultra333
    - Paul Wilson

    Sincere thanks and to:

    - Mikko Mononen (developer of Recast navigation libraries)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

*/

#ifndef __OgreRecastDefinitions_h_
#define __OgreRecastDefinitions_h_



/**
  * This file sets up all definitions needed by Recast/Detour.
  * Most of it is just taken from the official demo application.
  **/

// recast/detour stuff
#include "Recast.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"

#include <OgreVector3.h>

#define MAX_SINGLEPATHPOLY   512  // max number of polygons in a single path
#define MAX_EXTENDEDPATHPOLY 1024 // max number of polygons in a combined path (where the first path didn't reach the destination)
#define MAX_PATHVERT         1024 // most verts in a path

enum PolyAreas : unsigned short // Each area can only have one type
{
   // Ground area types
   POLYAREA_GRASS       = 0x0001,
   POLYAREA_WATER       = 0x0002,
   POLYAREA_ROAD        = 0x0004,
   POLYAREA_SAND        = 0x0008,
   POLYAREA_GATE        = 0x0010,
   POLYAREA_TYPE_MASK   = 0xFFFF,
} ;

enum PolyFlags : unsigned short // Each poly can have multiple flags in a mask where one bit must pass in order to accept the poly
{
   // Movement types
   POLYFLAGS_WALK        = 0x0001, // Any land poly area
   POLYFLAGS_FLOAT       = 0x0002, // Any water poly area
   POLYFLAGS_MOVE_MASK   = 0x000F,

   // Player restrictions
   POLYFLAGS_PLAYER_1    = 0x0010,
   POLYFLAGS_PLAYER_2    = 0x0020,
   POLYFLAGS_PLAYER_3    = 0x0040,
   POLYFLAGS_PLAYER_4    = 0x0080,
   POLYFLAGS_PLAYER_5    = 0x0100,
   POLYFLAGS_PLAYER_6    = 0x0200,
   POLYFLAGS_PLAYER_7    = 0x0400,
   POLYFLAGS_PLAYER_8    = 0x0800,
   POLYFLAGS_PLAYER_9    = 0x1000,
   POLYFLAGS_PLAYER_10   = 0x2000,
   POLYFLAGS_PLAYER_11   = 0x4000,
   POLYFLAGS_PLAYER_12   = 0x8000,
   POLYFLAGS_ALL_PLAYERS = 0xFFF0,

   POLYFLAGS_ALL         = 0xFFFF, // All poly areas
} ;

//
struct TerrainArea // Only square for the moment
{
   Ogre::Vector3 Centre ;
   float         Width ;
   float         Depth ;
   unsigned int  AreaId ; // Area identifier from OgreRecastDefinitions.h::PolyAreas
} ;

using TerrainAreaVector = std::vector <TerrainArea> ;

#endif // #ifndef __OgreRecastDefinitions_h_
