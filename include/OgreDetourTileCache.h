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

#include <Ogre.h>
#include "DetourTileCacheBuilder.h"
#include "DetourTileCache.h"
#include "DetourCommon.h"
#include "fastlz.h"
#include "InputGeom.h"
#include "OgreRecastDefinitions.h"

class OgreRecast ;

// Implementation of the meshProcess callback that detourTileCache
// does after building a navmesh. It allows you to do some extra
// processing on the navmesh, such as connecting off-mesh connections
// and assigning flags to certain poly areas.
// The reason it is initialized with an inputGeom object is that it
// is intended that the inputGeom not only stores the input geometry,
// but also information that has to be added to the navmesh in this
// post-processing phase.
struct MeshProcess : public dtTileCacheMeshProcess
{
   // Callback that happens after navmesh has been constructed.
   // Allows you to do some additional post-processing on the navmesh,
   // such as adding off-mesh connections or marking poly areas with
   // certain flags.
   virtual void
   process ( dtNavMeshCreateParams *params,
             unsigned char         *poly_areas,
             unsigned short        *poly_flags )
    {
        // Update poly flags from areas.
        for ( auto poly_index = 0 ; poly_index < params->polyCount ; ++poly_index )
        {
            if ( poly_areas [ poly_index ] == DT_TILECACHE_WALKABLE_AREA )
            {
                poly_areas [ poly_index ] = POLYAREA_GRASS ;
            }

            if ( ( poly_areas [ poly_index ] == POLYAREA_GRASS ) ||
                 ( poly_areas [ poly_index ] == POLYAREA_SAND ) ||
                 ( poly_areas [ poly_index ] == POLYAREA_ROAD ) )
            {
                poly_flags [ poly_index ] |= POLYFLAGS_WALK ;
            }
            else if ( poly_areas [ poly_index ] == POLYAREA_WATER )
            {
                poly_flags [ poly_index ] |= POLYFLAGS_FLOAT ;
            }
            else if ( poly_areas [ poly_index ] == POLYAREA_GATE )
            {
               //std::cout <<  "Gate flags before: " << poly_flags [ poly_index ] << " after: " << ( poly_flags [ poly_index ] | POLYFLAGS_WALK | POLYFLAGS_ALL_PLAYERS ) << std::endl ;

               poly_flags [ poly_index ] |= POLYFLAGS_WALK ;

               // All polygons by default allow all players
               poly_flags [ poly_index ] |= POLYFLAGS_ALL_PLAYERS ;
            }
        }
    }
} ;

// FastLZ implementation of detour tile cache tile compressor.
// You can define a custom implementation if you wish to use
// a different compression algorithm for compressing your
// detour heightfield tiles.
// The result of compression runs is the data that detourTileCache
// stores in memory (or can save out to disk).
// The compressed heightfield tiles are stored in ram as they allow
// to quickly generate a navmesh tile, possibly with obstacles added
// to them, without the need for a full rebuild.
struct FastLZCompressor : public dtTileCacheCompressor
{
   virtual int
   maxCompressedSize ( const int buffer_size )
   {
      return static_cast <int> ( buffer_size * 1.05f ) ;
   }

   virtual dtStatus
   compress ( const unsigned char *buffer,
              const int           buffer_size,
              unsigned char       *compressed,
              const int           /*max_compressed_size*/,
              int                 *compressed_size )
   {
      *compressed_size = fastlz_compress ( static_cast <const void * const> ( buffer ), buffer_size, compressed ) ;
      return DT_SUCCESS ;
   }

   virtual dtStatus
   decompress ( const unsigned char *compressed,
                const int           compressed_size,
                unsigned char       *buffer,
                const int           max_buffer_size,
                int                 *buffer_size )
   {
          *buffer_size = fastlz_decompress ( compressed, compressed_size, buffer, max_buffer_size ) ;
          return ( ( *buffer_size < 0 ) ? DT_FAILURE : DT_SUCCESS ) ;
   }
} ;

// Allows a custom memory allocation technique to be implemented
// for storing compressed tiles. This implementation does a linear
// memory allocation.
struct LinearAllocator : public dtTileCacheAlloc
{
   unsigned char *buffer ;
   int           capacity ;
   int           top ;
   int           high ;

   LinearAllocator ( const int cap ) :
      buffer   ( nullptr ),
      capacity ( 0 ),
      top      ( 0 ),
      high     ( 0 )
   {
      resize ( cap ) ;
   }

   ~LinearAllocator ()
   {
      dtFree ( buffer ) ;
   }

   void
   resize ( const int cap )
   {
      if ( buffer )
      {
         dtFree ( buffer ) ;
      }

      buffer   = static_cast <unsigned char *> ( dtAlloc ( cap, DT_ALLOC_PERM ) ) ;
      capacity = cap ;
   }

   virtual void
   reset ()
   {
      high = dtMax ( high, top ) ;
      top  = 0 ;
   }

   virtual void *
   alloc ( const int size )
   {
      if ( ! buffer )
      {
         return 0 ;
      }

      if ( ( top + size ) > capacity )
      {
         return 0 ;
      }

      unsigned char *mem = &buffer [ top ] ;

      top += size ;

      return mem ;
   }

   virtual void
   free ( void */*ptr*/ )
   {
      // Empty
   }
} ;

// Maximum layers (floor levels) that 2D navmeshes can have in the tilecache.
// This determines the domain size of the tilecache pages, as their dimensions
// are width*height*layers.
static const int MAX_LAYERS = 1 ;

// Struct that stores the actual tile data in binary form.
struct TileCacheData
{
    unsigned char *data ;
    int           dataSize ;
} ;

// Rasterization context stores temporary data used
// when rasterizing inputGeom into a navmesh.
struct RasterizationContext
{
   RasterizationContext () :
      solid    ( nullptr ),
      triareas ( nullptr ),
      lset     ( nullptr ),
      chf      ( nullptr ),
      ntiles   ( 0 )
   {
      memset ( tiles, 0, sizeof ( TileCacheData ) * MAX_LAYERS ) ;
   }

   ~RasterizationContext ()
   {
      rcFreeHeightField ( solid ) ;

      delete [] triareas ;

      rcFreeHeightfieldLayerSet ( lset ) ;
      rcFreeCompactHeightfield ( chf ) ;

      for ( int i = 0 ; i < MAX_LAYERS ; ++i )
      {
         dtFree ( tiles [ i ].data ) ;
         tiles [ i ].data = 0 ;
      }
   }

   rcHeightfield         *solid ;
   unsigned char         *triareas ;
   rcHeightfieldLayerSet *lset ;
   rcCompactHeightfield  *chf ;
   TileCacheData         tiles [ MAX_LAYERS ] ;
   int                   ntiles ;
} ;

// Build context stores temporary data used while
// building a navmesh tile.
struct BuildContext
{
   inline BuildContext ( struct dtTileCacheAlloc *a ) : layer(0), lcset(0), lmesh(0), alloc(a) {}
   inline ~BuildContext() { purge(); }

   void purge()
   {
      dtFreeTileCacheLayer(alloc, layer);
      layer = 0;
      dtFreeTileCacheContourSet(alloc, lcset);
      lcset = 0;
      dtFreeTileCachePolyMesh(alloc, lmesh);
      lmesh = 0;
   }

   struct dtTileCacheLayer* layer;
   struct dtTileCacheContourSet* lcset;
   struct dtTileCachePolyMesh* lmesh;
   struct dtTileCacheAlloc* alloc;
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

// DetourTileCache manages a large grid of individual navmeshes stored in pages to
// allow managing a navmesh for a very large map. Navmesh pages can be requested
// when needed or swapped out when they are no longer needed.
// Using a tilecache the navigation problem is localized to one tile, but pathfinding
// can still find a path that references to other neighbour tiles on the higher hierarchy
// level of the tilecache. Localizing the pathfinding problem allows it to be more scalable,
// also for very large worlds.
// DetouTileCache stores navmeshes in an intermediary format as 2D heightfields
// that can have multiple levels. It allows to quickly generate a 3D navmesh from
// this intermediary format, with the additional option of adding or removing
// temporary obstacles to the navmesh and regenerating it.
class OgreDetourTileCache
{
public :
   // Create a tilecache that will build a tiled recast navmesh stored at the specified
   // OgreRecast component. Will use specified tilesize (a multiple of 8 between 16 and 128),
   // all other configuration parameters are copied from the OgreRecast component configuration.
   // Tilesize is the number of (recast) cells per tile.
   OgreDetourTileCache ( OgreRecast         &recast,
                         rcContext          &context,
                         rcConfig           &config,
                         dtNavMeshQuery     &nav_query,
                         const unsigned int max_num_obstacles,
                         const int          tile_size ) ;
   ~OgreDetourTileCache () ;

   class NavMeshDebug *
   CreateDebugger () ;

   // Build all tiles of the tilecache and construct a recast navmesh from the
   // specified entities. These entities need to be already added to the scene so that
   // their world position and orientation can be calculated.
   //
   // This is an Ogre adaptation of Sample_TempObstacles::handleBuild()
   // First init the OgreRecast module like you would construct a simple single
   // navmesh, then invoke this method instead of OgreRecast::NavMeshBuild() to create
   // a tileCache from the specified ogre geometry.
   // The specified ogre entities need to be added to a scenenode in the scene before this
   // method is called.
   // The resulting navmesh will be created in the OgreRecast module, at OgreRecast::m_navMesh;
   //
   // Will issue a configure() call so the entities specified will determine the world bounds
   // of the tilecache.
   bool
   TileCacheBuild ( std::vector<Ogre::Entity*> srcMeshes,
                    const TerrainAreaVector    &area_list ) ;

   bool
   SaveAll ( const Ogre::String &filename ) ;

   bool
   LoadAll ( const Ogre::String         &filename,
             std::vector<Ogre::Entity*> srcMeshes ) ;

   // Update (tick) the tilecache.
   // You must call this method in your render loop continuously to dynamically
   // update the navmesh when obstacles are added or removed.
   // Navmesh rebuilding happens per tile and only where needed. Tile rebuilding is
   // timesliced.
   void
   HandleUpdate ( const float delta_time,
                  const bool  until_up_to_date ) ; // Continue processing the tile cache obstacles until the entire navmesh is up-to-date

   // Add a temporary obstacle to the tilecache (as a deferred request).
   // The navmesh will be updated correspondingly after the next (one or many)
   // update() call as a deferred command.
   // If m_tileCache->m_params->maxObstacles obstacles are already added, this call
   // will have no effect. Also, at one time only MAX_REQUESTS can be added, or nothing
   // will happen.
   //
   // If successful returns a reference to the added obstacle.
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

   // Remove temporary (cylindrical) obstacle with specified reference. The affected tiles
   // will be rebuilt. This operation is deferred and will happen in one of the next
   // update() calls. At one time only MAX_REQUESTS obstacles can be removed, or nothing will happen.
   bool
   RemoveObstacle ( dtObstacleRef obstacleRef ) ;

   int
   AddConvexVolume ( ConvexVolume *vol ) ;

   bool
   DeleteConvexVolume ( int i ) ;

private :
   // Configure the tilecache for building navmesh tiles from the specified input geometry.
   // The inputGeom is mainly used for determining the bounds of the world for which a navmesh
   // will be built, so at least bmin and bmax of inputGeom should be set to your world's outer
   // bounds. This world bounding box is used to calculate the grid size that the tilecache has
   // to initialize.
   // This method has to be called once after construction, and before any tile builds happen.
   bool
   ConfigureTileCacheContext () ;

   // Build the 2D navigation grid divided in layers that is the intermediary format stored in the tilecache.
   // Builds the specified tile from the given input geometry. Only the part of the geometry that intersects the
   // needed tile is used.
   // From this format a 3D navmesh can be quickly generated at runtime.
   // This process uses a large part of the recast navmesh building pipeline (implemented in OgreRecast::NavMeshBuild()),
   // up till step 4.
   int
   RasterizeTileLayers ( const int     tx,
                         const int     ty,
                         TileCacheData *tiles,
                         const int     maxTiles ) ;

   bool
   InitTileCache () ; // Inits the tilecache. Helper used by constructors.

   // InputGeom from which the tileCache is initially inited (it's bounding box is considered the bounding box
   // for the entire world that the navmesh will cover). Tile build methods without specific geometry or entity
   // input will build navmesh from this geometry.
   // It also stored the convex temp obstacles. (will be gone in the future)
   // In the future this variable will probably disappear.
   InputGeom      *InputGeometry ;
   OgreRecast     &Recast ; // Ogre Recast component that holds the recast config and where the navmesh will be built.
   dtNavMeshQuery &NavQuery ;

   struct LinearAllocator  *m_talloc ; // The tile cache memory allocator implementation used.
   struct FastLZCompressor *m_tcomp ; // The tile compression implementation used.

   // Callback handler that processes right after processing
   // a tile mesh. Adds off-mesh connections to the mesh.
   struct MeshProcess *m_tmproc ;

   class dtTileCache  *m_tileCache ; // The detourTileCache component this class wraps.
   class dtNavMesh    *m_navMesh ;
   rcConfig           &m_cfg ; // Recast config (copied from the OgreRecast component).
   dtTileCacheParams  m_tcparams ; // DetourTileCache configuration parameters.
   rcContext          &m_ctx ; // Context that stores temporary working variables when navmesh building.

   // Configuration parameters.
   int          m_maxTiles ;
   int          m_maxPolysPerTile ;
   int          m_tileSize ;
   unsigned int MaxNumObstacles ;
   float        m_cellSize ;
   int          m_tw ; // Size of the tile grid (x dimension)
   int          m_th ; // Size of the tile grid (y dimension)

   // Maximum number of convex volume obstacles that can be added to this inputGeom.
   static const int MAX_VOLUMES = 1024 ;

   // Convex Volumes (temporary) added to this geometry.
   ConvexVolume *m_volumes [ MAX_VOLUMES ] ;
   int          m_volumeCount ;

   struct TileCacheSetHeader
   {
      int               magic ;
      int               version ;
      int               numTiles ;
      dtNavMeshParams   meshParams ;
      dtTileCacheParams cacheParams ;
      rcConfig          recastConfig ;
   } ;

   struct TileCacheTileHeader
   {
      dtCompressedTileRef tileRef ;
      int                 dataSize ;
   } ;
} ;
