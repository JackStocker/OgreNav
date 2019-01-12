#include "GlobalTypes.h"

// Detour
#include "DetourTileCache.h"
#include "DetourNavMesh.h"

// Std
#include <vector>

class DebugManager ;

class OgreDetourTileCache ;

class NavMeshDebug
{
public :
   NavMeshDebug ( const OgreDetourTileCache &ogre_tile_cache ) ;
   ~NavMeshDebug () ;

   void
   SetDrawTiles ( const bool draw_tiles ) ;

   void
   SetDrawObstacles ( const bool draw_obstacles ) ;

   void
   RedrawTile ( const std::size_t tile_x,
                const std::size_t tile_z ) ;

   void
   AddObstacle ( const dtObstacleRef       &obstacle_ref,
                 const dtTileCacheObstacle &obstacle ) ;

   void
   RemoveObstacle ( const dtObstacleRef &obstacle_ref ) ;

   void
   RedrawAll () ;

private :
   void
   DrawEntireNavMesh ( const dtTileCache          &tile_cache,
                       const dtNavMesh            &nav_mesh,
                       const class dtNavMeshQuery &nav_query ) ;

   void
   DrawAllTiles ( const dtNavMesh      &mesh,
                  const dtNavMeshQuery &query ) ;

   void
   DrawAllObstacles ( const dtTileCache &tile_cache ) ;

   void
   DrawGrid ( const class InputGeom &input_geom ) ;

   void
   DrawInputMesh ( const InputGeom &input_geom ) ;

   void
   RemoveAllTiles () ;

   void
   RemoveAllObstacles () ;

   void
   RemoveGrid () ;

   void
   RemoveInputMesh () ;

   void
   DrawTile ( const dtNavMesh      &mesh,
              const dtNavMeshQuery &query,
              const dtMeshTile     &tile,
              struct Tile          &debug_tile ) ;

   DebugId
   DrawObstacle ( const dtTileCache &tile_cache ) ;

   std::vector <DebugId>
   DrawPolysWithFlags ( const dtNavMesh         &mesh,
                        const unsigned short    poly_flags,
                        const Ogre::ColourValue &colour ) ;

   DebugId
   DrawNavMeshPoly ( const dtNavMesh         &mesh,
                     dtPolyRef               ref,
                     const Ogre::ColourValue &colour ) ;

   DebugId
   DrawTilePolys ( const dtNavMesh      &mesh,
                   const dtNavMeshQuery &query,
                   const dtMeshTile     &tile ) ;

   DebugId
   DrawTilePolyBoundaries ( const dtMeshTile        &tile,
                            const Ogre::ColourValue &colour,
                            const float             line_width,
                            bool                    draw_areas ) ;

   DebugId
   DrawTileVertices ( const dtMeshTile        &tile,
                      const Ogre::ColourValue &colour,
                      const float             half_size ) ;

   DebugId
   GeneratorBoxForObstacle ( const dtTileCacheObstacle &obstacle ) ;

   std::vector <struct DebugPolyTriangle>
   GenerateTriangleListFromPoly ( const dtMeshTile        &tile,
                                  const dtPoly            &poly,
                                  const std::size_t       poly_index,
                                  const Ogre::ColourValue &colour ) ;

   std::vector <struct DebugPolyTriangle>
   GenerateDot ( const Ogre::Vector3     &position,
                 const Ogre::ColourValue &colour,
                 const float             half_size ) ;

   DebugManager              &CurrentDebugManager ;
   const OgreDetourTileCache &OgreTileCache ;

   std::vector <struct Tile>     TileList ;
   std::vector <struct Obstacle> ObstacleList ;
   DebugId                       GridDebugId ;
   DebugId                       InputMeshDebugId ;
   bool                          DrawTiles ;
   bool                          DrawObstacles ;
} ;
