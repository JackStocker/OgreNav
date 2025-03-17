#include "GlobalTypes.h"

// Detour
#include "DetourTileCache.h"
#include "DetourNavMesh.h"

// Std
#include <vector>

class DebugManager ;

class OgreRecast ;
class dtNavMeshQuery ;
struct rcHeightfield ;

class NavMeshDebug
{
public :
   NavMeshDebug ( const OgreRecast     &recast,
                  const dtTileCache    &tile_cache,
                  const dtNavMesh      &nav_mesh,
                  const dtNavMeshQuery &nav_query ) ;
   ~NavMeshDebug () ;

   void
   SetDrawTiles ( const bool draw_tiles ) ;

   void
   SetDrawObstacles ( const bool draw_obstacles ) ;

   void
   SetDrawGrid ( const bool draw_grid ) ;

   void
   RedrawTile ( const std::size_t tile_x,
                const std::size_t tile_z ) ;

   void
   RedrawAllTilesUnderObstacles () ;

   void
   AddObstacle ( const dtObstacleRef       &obstacle_ref,
                 const dtTileCacheObstacle &obstacle ) ;

   void
   RemoveObstacle ( const dtObstacleRef &obstacle_ref ) ;

   void
   RedrawAll () ;

private :
   void
   DrawEntireNavMesh ( const dtTileCache    &tile_cache,
                       const dtNavMesh      &nav_mesh,
                       const dtNavMeshQuery &nav_query ) ;

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
   DrawHeightField ( const rcHeightfield &height_field ) ;

   void
   RemoveAllTiles () ;

   void
   RemoveAllObstacles () ;

   void
   RemoveGrid () ;

   void
   RemoveInputMesh () ;

   void
   DrawTile ( const dtPolyRef      base_polyref,
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
   DrawNavMeshPoly ( const dtMeshTile        &tile,
                     const dtPoly            &poly,
                     const Ogre::ColourValue &colour ) ;

   DebugId
   DrawTilePolys ( const dtPolyRef      base_polyref,
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

   DebugManager         &CurrentDebugManager ;
   const OgreRecast     &Recast ;
   const dtTileCache    &TileCache ;
   const dtNavMesh      &NavMesh ;
   const dtNavMeshQuery &NavQuery ;

   std::vector <struct Tile>     TileList ;
   std::vector <struct Obstacle> ObstacleList ;
   DebugId                       GridDebugId ;
   DebugId                       InputMeshDebugId ;
   bool                          DrawTiles ;
   bool                          DrawObstacles ;
   bool                          DrawGrid_ ;
} ;
