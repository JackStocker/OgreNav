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
#pragma once

#include "OgreRecastDefinitions.h"
#include "OgreDetourTileCache.h"
#include "PlayerFlagQueryFilter.h"

#include <Ogre.h>

class  OgreRecastNavmeshPruner ;
struct OgreRecastConfigParams ;
class  NavMeshDebug ;
class  dtNavMeshQuery ;

enum class FindPathReturnCode
{
   PATH_FOUND                  = 0,  //  0   found path
   CANNOT_FIND_START           = -1, //  -1  Couldn't find polygon nearest to start point
   CANNOT_FIND_END             = -2, //  -2  Couldn't find polygon nearest to end point
   CANNOT_CREATE_PATH          = -3, //  -3  Couldn't create a path
   CANNOT_FIND_PATH            = -4, //  -4  Couldn't find a path
   CANNOT_FIND_STRAIGHT_PATH   = -6, //  -6  Couldn't find a straight path
   CANNOT_CREATE_STRAIGHT_PATH = -5, //  -5  Couldn't create a straight path
} ;

// This class serves as a wrapper between Ogre and Recast/Detour
class OgreRecast
{
public:
   OgreRecast ( const OgreRecastConfigParams &config_params ) ;

   void
   Update ( const float delta_time,
            const bool  until_up_to_date ) ;

   bool
   Generate ( const unsigned int         max_num_obstacles,
              const int                  tile_size,
              std::vector<Ogre::Entity*> source_meshes,
              const TerrainAreaVector    &area_list ) ;

   bool
   Load ( const Ogre::String         &filename,
          const unsigned int         max_num_obstacles,
          const int                  tile_size,
          std::vector<Ogre::Entity*> source_meshes ) ;

   bool
   Save ( const Ogre::String &filename ) ;

   // A navigation mesh must have been Generated or Loaded before this is called otherwise a nullptr is returned.
   // When a navigation mesh is deleted (e.g. Generate or Load called again will delete any existing), then the
   // previous debugger is invalid as the returned pointer is tied to a given navigation mesh instance.
   std::unique_ptr <NavMeshDebug>
   CreateNavMeshDebugger () ;

   dtObstacleRef
   AddObstacle ( const Ogre::Vector3  &min,
                 const Ogre::Vector3  &max,
                 const unsigned char  area_id,
                 const unsigned short flags ) ;

   dtObstacleRef
   AddObstacle ( const Ogre::Vector3  &centre,
                 const float          width,
                 const float          depth,
                 const float          height,
                 const float          y_rotation, // radians
                 const unsigned char  area_id,
                 const unsigned short flags ) ;

   const dtTileCacheObstacle *
   GetObstacleByRef ( dtObstacleRef ref ) ;

   bool
   RemoveObstacle ( dtObstacleRef ref ) ;

   int
   AddConvexVolume ( ConvexVolume *vol ) ;

   bool
   DeleteConvexVolume ( int volume_index ) ;

   // Find a path beween start point and end point and, if possible, generates a list of lines in a path.
   // It might fail if the start or end points aren't near any navmesh polygons, or if the path is too long,
   // or it can't make a path, or various other reasons.
   FindPathReturnCode
   FindPath ( float                      *start_pos,
              float                      *end_pos,
              const unsigned int         include_flags,
              const unsigned int         exclude_flags,
              std::vector<Ogre::Vector3> &path ) ;

   FindPathReturnCode
   FindPath ( const Ogre::Vector3        &start_pos,
              const Ogre::Vector3        &end_pos,
              const unsigned int         include_flags,
              const unsigned int         exclude_flags,
              std::vector<Ogre::Vector3> &path ) ;

   // Find a point on the navmesh closest to the specified point position, within predefined
   // bounds.
   // Returns true if such a point is found (returned as resultPt), returns false
   // if no point is found. When false is returned, resultPt is not altered.
   bool
   FindNearestPointOnNavmesh ( const Ogre::Vector3 &position,
                               const unsigned int  include_flags,
                               const unsigned int  exclude_flags,
                               Ogre::Vector3       &result_point ) ;

   bool
   FindNearestPolyOnNavmesh ( const Ogre::Vector3 &position,
                              const unsigned int  include_flags,
                              const unsigned int  exclude_flags,
                              Ogre::Vector3       &result_point,
                              dtPolyRef           &result_poly ) ;

   // Convenience function for converting between Ogre::Vector3 and float* used by recast.
   static void
   OgreVect3ToFloatA ( const Ogre::Vector3 &vect,
                       float               *result ) ;

   //  Convenience function for converting between float* used by recast and Ogre::Vector3.
   static void
   FloatAToOgreVect3 ( const float   *vect,
                       Ogre::Vector3 &result ) ;

private :
   // Configure navbuild parameters for this module
   void
   ConfigureBuildParameters ( const OgreRecastConfigParams &config_params ) ;

   rcConfig                              RecastConfig ;
   rcContext                             BuildContext ;
   std::unique_ptr <OgreDetourTileCache> TileCache ;
   dtNavMeshQuery                        NavQuery ;

   // The poly filter that will be used for all (random) point and nearest poly searches.
   PlayerFlagQueryFilter QueryFilter ;

   // The offset size (box) around points used to look for nav polygons.
   // This offset is used in all search for points on the navmesh.
   // The maximum offset that a specified point can be off from the navmesh.
   float PolySearchBox [ 3 ] ;
} ;
