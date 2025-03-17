#include "NavMeshDebug.h"
#include "DebugManager.h"
#include "SoftAssert.h"

// Detour
#include "OgreDetourTileCache.h"
#include "OgreRecast.h"

struct Tile
{
   std::size_t           TileXIndex ;
   std::size_t           TileZIndex ;
   std::vector <DebugId> PolygonList ;
   std::vector <DebugId> DotList ;
} ;

struct Obstacle
{
   dtObstacleRef ObstacleRef ;
   DebugId       PolyId ;
} ;

template <typename IntType>
typename std::enable_if <std::is_integral <IntType>::value, IntType>::type // Only integer types
GetBit ( const IntType integer,
         const short   bit_index )
{
   return ( integer & ( 1 << bit_index ) ) >> bit_index ;
}

Ogre::ColourValue
IntToColour ( const int   integer,
              const float alpha_percent )
{
   const auto r = GetBit ( integer, 1 ) + GetBit ( integer, 3 ) * 2 + 1 ;
   const auto g = GetBit ( integer, 2 ) + GetBit ( integer, 4 ) * 2 + 1 ;
   const auto b = GetBit ( integer, 0 ) + GetBit ( integer, 5 ) * 2 + 1 ;

   return Ogre::ColourValue ( r * 0.25f, g * 0.25f, b * 0.25f, alpha_percent ) ;
}

unsigned int
CountSetBits ( unsigned int number )
{
   unsigned int bit_count = 0 ;

   while ( number > 0 )
   {
       bit_count += ( number & 1 ) ;
       number >>= 1 ;
   }

   return bit_count ;
}

Ogre::ColourValue
AreaToColour ( const unsigned int area_id,
               const unsigned int flags = 0 )
{
   switch ( area_id )
   {
   case POLYAREA_GRASS :
      {
         return Ogre::ColourValue ( 0, 0.75f, 1, 1 ) ; // light blue
      }
   case POLYAREA_WATER :
      {
         return Ogre::ColourValue ( 0, 0, 1, 1 ) ; // blue
      }
   case POLYAREA_ROAD :
      {
         return Ogre::ColourValue ( 0.196f, 0.078f, 0.047f, 1 ) ; // brown
      }
   case POLYAREA_GATE :
      {
         const auto  bits_set   = CountSetBits ( flags & POLYFLAGS_ALL_PLAYERS ) ;
         const float visibility = std::min ( bits_set * 0.5f, 1.0f ) ;
         return ( Ogre::ColourValue ( 0, 1, 1, 1 ) * visibility ) ; // cyan
      }
   case POLYAREA_SAND :
      {
         return Ogre::ColourValue ( 1, 1, 0, 1 ) ; // yellow
      }
   default :
      {
         return Ogre::ColourValue ( 1, 0, 0, 1 ) ; // red
      }
   }
}

float
PointDistanceToLine2d ( const float *point,
                        const float *line_start,
                        const float *line_end )
{
   const float line_delta_x        = line_end [ 0 ] - line_start [ 0 ] ;
   const float line_delta_z        = line_end [ 2 ] - line_start [ 2 ] ;
   float       point_delta_x       = point [ 0 ] - line_start [ 0 ] ;
   float       point_delta_z       = point [ 2 ] - line_start [ 2 ] ;
   float       squared_line_delta  = ( ( line_delta_x * line_delta_x ) + ( line_delta_z * line_delta_z ) ) ;
   float       squared_point_delta = ( ( line_delta_x * point_delta_x ) + ( line_delta_z * point_delta_z ) ) ;

   if ( squared_line_delta != 0 )
   {
      squared_point_delta /= squared_line_delta ;
   }

   point_delta_x = line_start [ 0 ] + ( squared_point_delta * line_delta_x ) - point [ 0 ] ;
   point_delta_z = line_start [ 2 ] + ( squared_point_delta * line_delta_z ) - point [ 2 ] ;

   return ( ( point_delta_x * point_delta_x ) +
            ( point_delta_z * point_delta_z ) ) ;
}

NavMeshDebug::
NavMeshDebug ( const OgreRecast     &recast,
               const dtTileCache    &tile_cache,
               const dtNavMesh      &nav_mesh,
               const dtNavMeshQuery &nav_query ) :
   CurrentDebugManager ( DebugManager::GetDebugManager () ),
   Recast              ( recast ),
   TileCache           ( tile_cache ),
   NavMesh             ( nav_mesh ),
   NavQuery            ( nav_query )
{
   GridDebugId      = INVALID_DEBUG_ID ;
   InputMeshDebugId = INVALID_DEBUG_ID ;
   DrawTiles        = false ;
   DrawObstacles    = false ;
   DrawGrid_        = false ;
}

NavMeshDebug::
~NavMeshDebug ()
{
   RemoveAllTiles () ;
   RemoveAllObstacles () ;
   RemoveGrid () ;
   RemoveInputMesh () ;
}

void
NavMeshDebug::
SetDrawTiles ( const bool draw_tiles )
{
   if ( DrawTiles != draw_tiles )
   {
      DrawTiles = draw_tiles ;

      if ( ! DrawTiles )
      {
         RemoveAllTiles () ;
      }
      else
      {
         DrawAllTiles ( NavMesh, NavQuery ) ;
      }
   }
}

void
NavMeshDebug::
SetDrawObstacles ( const bool draw_obstacles )
{
   if ( DrawObstacles != draw_obstacles )
   {
      DrawObstacles = draw_obstacles ;

      if ( ! DrawObstacles )
      {
         RemoveAllObstacles () ;
      }
      else
      {
         DrawAllObstacles ( TileCache ) ;
      }
   }
}

void
NavMeshDebug::
SetDrawGrid ( const bool draw_grid )
{
   if ( DrawGrid_ != draw_grid )
   {
      DrawGrid_ = draw_grid ;

      if ( ! DrawGrid_ )
      {
         RemoveGrid () ;
      }
      else
      {
         //if ( Recast.GetInputGeometry () )
         //{
         //   DrawGrid ( *Recast.GetInputGeometry () ) ;
         //}

         for ( auto* solid : Recast.GetHeightField () )
         {
            DrawHeightField ( *solid ) ;
         }
      }
   }
}

void
NavMeshDebug::
RedrawTile ( const std::size_t tile_x,
             const std::size_t tile_z )
{
   if ( DrawTiles )
   {
      // Find or create tile
      auto tile_iter = std::find_if ( TileList.begin (),
                                      TileList.end (),
                                      [&] ( const Tile &tile )
                                         {
                                            return ( ( tile.TileXIndex == tile_x ) &&
                                                     ( tile.TileZIndex == tile_z ) ) ;
                                         } ) ;

      if ( tile_iter == TileList.end () )
      {
         Tile new_tile = { tile_x, tile_z, {}, {} } ;
         TileList.emplace_back ( new_tile ) ;

         tile_iter = std::prev ( TileList.end () ) ;
      }
      else
      {
         auto &tile = *tile_iter ;

         for ( auto &poly_id : tile.PolygonList )
         {
            CurrentDebugManager.DestroyDebugObject ( poly_id ) ;
         }

         for ( auto &dot_id : tile.DotList )
         {
            CurrentDebugManager.DestroyDebugObject ( dot_id ) ;
         }

         tile.PolygonList.clear () ;
         tile.DotList.clear () ;
      }

      const dtMeshTile *tile = NavMesh.getTileAt ( tile_x, tile_z, 0 ) ;

      if ( tile )
      {
         DrawTile ( NavMesh.getPolyRefBase ( tile ), NavQuery, *tile, *tile_iter ) ;
      }
   }
}

void
NavMeshDebug::
RedrawAllTilesUnderObstacles ()
{
   if ( DrawTiles )
   {
      for ( int obstacle_index = 0 ; obstacle_index < TileCache.getObstacleCount () ; ++obstacle_index )
      {
         const dtTileCacheObstacle *obstacle = TileCache.getObstacle ( obstacle_index ) ;

         for ( int tile_index = 0 ; tile_index < obstacle->ntouched ; ++tile_index )
         {
            const dtCompressedTile *tile = TileCache.getTileByRef ( obstacle->touched [ tile_index ] ) ;

            if ( tile )
            {
               RedrawTile ( tile->header->tx, tile->header->ty ) ;
            }
         }
      }
   }
}

void
NavMeshDebug::
AddObstacle ( const dtObstacleRef       &obstacle_ref,
              const dtTileCacheObstacle &obstacle )
{
   if ( DrawObstacles )
   {
      Obstacle new_obstacle = { obstacle_ref, GeneratorBoxForObstacle ( obstacle ) } ;
      ObstacleList.emplace_back ( new_obstacle ) ;
   }
}

void
NavMeshDebug::
RemoveObstacle ( const dtObstacleRef &obstacle_ref )
{
   if ( DrawObstacles )
   {
      auto obstacle_iter = std::find_if ( ObstacleList.begin (),
                                          ObstacleList.end (),
                                          [obstacle_ref] ( const Obstacle &obstacle )
                                             {
                                                return ( obstacle.ObstacleRef == obstacle_ref ) ;
                                             } ) ;

      if ( obstacle_iter != ObstacleList.end () )
      {
         if ( obstacle_iter->PolyId != INVALID_DEBUG_ID )
         {
            CurrentDebugManager.DestroyDebugObject ( obstacle_iter->PolyId ) ;
         }

         ObstacleList.erase ( obstacle_iter ) ;
      }
   }
}

void
NavMeshDebug::
RedrawAll ()
{
   RemoveAllTiles () ;
   RemoveAllObstacles () ;
   RemoveGrid () ;
   RemoveInputMesh () ;

   DrawEntireNavMesh ( TileCache, NavMesh, NavQuery ) ;
}

void
NavMeshDebug::
DrawEntireNavMesh ( const dtTileCache    &tile_cache,
                    const dtNavMesh      &nav_mesh,
                    const dtNavMeshQuery &nav_query )
{
   //DrawInputMesh ( input_geom ) ;

   DrawAllObstacles ( tile_cache ) ;

   if ( Recast.GetInputGeometry () )
   {
      DrawGrid ( *Recast.GetInputGeometry () ) ;
   }

   DrawAllTiles ( nav_mesh, nav_query ) ;

   // Enabled to draw polys with a given flag
   //DrawPolysWithFlags ( nav_mesh, POLYFLAGS_WALK, Ogre::ColourValue ( 0, 0, 0, 0.5f ) ) ;
}

void
NavMeshDebug::
DrawAllTiles ( const dtNavMesh      &mesh,
               const dtNavMeshQuery &query )
{
   for ( auto tile_index = 0 ; tile_index < mesh.getMaxTiles () ; ++tile_index )
   {
      const dtMeshTile *tile = mesh.getTile ( tile_index ) ;

      if ( tile->header )
      {
         Tile new_tile = { static_cast <std::size_t> ( tile->header->x ), static_cast <std::size_t> ( tile->header->y ), {}, {} } ;

         DrawTile ( mesh.getPolyRefBase ( tile ), query, *tile, new_tile ) ;

         TileList.emplace_back ( new_tile ) ;
      }
   }
}

void
NavMeshDebug::
DrawAllObstacles ( const dtTileCache &tile_cache )
{
   for ( auto obstacle_index = 0 ; obstacle_index < tile_cache.getObstacleCount () ; ++obstacle_index )
   {
      const dtTileCacheObstacle *obstacle = tile_cache.getObstacle ( obstacle_index ) ;

      if ( obstacle &&
           ( obstacle->state != DT_OBSTACLE_EMPTY ) )
      {
         dtObstacleRef obstacle_ref = tile_cache.getObstacleRef ( obstacle ) ;

         Obstacle new_obstacle = { obstacle_ref, GeneratorBoxForObstacle ( *obstacle ) } ;
         ObstacleList.emplace_back ( new_obstacle ) ;
      }
   }
}

void
NavMeshDebug::
DrawGrid ( const InputGeom &input_geom )
{
   const Ogre::ColourValue colour ( 0, 0, 0, 0.25f ) ;
   const float             line_width = 1.0f ;
   const float             cell_size  = 0.3f ;
   const float             tile_size  = 40 ;

   const float *bmin       = input_geom.getMeshBoundsMin () ;
   const float *bmax       = input_geom.getMeshBoundsMax () ;
   int         grid_width  = 0 ;
   int         grid_height = 0 ;

   rcCalcGridSize ( bmin, bmax, cell_size, &grid_width, &grid_height ) ;

   const float start_x      = bmin [ 0 ] ;
   const float start_y      = bmin [ 1 ] ;
   const float start_z      = bmin [ 2 ] ;
   const int   column_count = ( grid_width + static_cast <int> ( tile_size ) -1 ) / static_cast <int> ( tile_size ) ;
   const int   row_count    = ( grid_height + static_cast <int> ( tile_size ) -1 ) / static_cast <int> ( tile_size ) ;
   const float element_size = tile_size * cell_size ;

   std::vector <Ogre::Vector3> points ;

   for ( auto row_index = 0 ; row_index <= row_count ; ++row_index )
   {
      points.push_back ( Ogre::Vector3 ( start_x,                                   start_y, start_z + ( row_index * element_size ) ) ) ;
      points.push_back ( Ogre::Vector3 ( start_x + ( column_count * element_size ), start_y, start_z + ( row_index * element_size ) ) ) ;
   }

   for ( auto column_index = 0 ; column_index <= column_count ; ++column_index )
   {
      points.push_back ( Ogre::Vector3 ( start_x + ( column_index * element_size ), start_y, start_z                                ) ) ;
      points.push_back ( Ogre::Vector3 ( start_x + ( column_index * element_size ), start_y, start_z + ( row_count * element_size ) ) ) ;
   }

   GridDebugId = CurrentDebugManager.CreateDebugPoly ( points, 0.0f, colour, line_width ) ;
}

void
NavMeshDebug::
DrawInputMesh ( const InputGeom &input_geom )
{
   const float *verts             = input_geom.getVerts () ;
   const int   *tris              = input_geom.getTris () ;
   const float *normals           = input_geom.getNormals () ;
   const int   ntris              = input_geom.getTriCount () ;
   const float walkableSlopeAngle = 60.0f ;

   if ( verts &&
        tris &&
        normals )
   {
      const float walkableThr = cosf ( Ogre::Math::DegreesToRadians ( walkableSlopeAngle ) ) ;
      const auto  unwalkable  = Ogre::ColourValue ( 0.75f, 0.5f, 0, 1.0f ) ;

      std::vector <DebugPolyTriangle> triangle_list ;

      for ( auto triangle_index = 0 ; triangle_index < ntris * 3 ; triangle_index += 3 )
      {
         std::array <Ogre::Vector3, 3> points ;
         Ogre::ColourValue             colour = Ogre::ColourValue::Black ;

         const float *norm = &normals [ triangle_index ] ;
         auto        a     = static_cast <unsigned char> ( 220 * ( 2 + norm [ 0 ] + norm [ 1 ] ) / 4 ) / 255.0f ;

         if ( norm [ 1 ] < walkableThr )
         {
            colour = Ogre::Math::lerp ( Ogre::ColourValue ( a, a, a, 1.0f ), unwalkable, 0.25f ) ;
         }
         else
         {
            colour = Ogre::ColourValue ( a, a, a, 1.0f ) ;
         }

         colour.a = 0.75 ;

         const float *va = &verts [ tris [ triangle_index + 0 ] * 3 ] ;
         const float *vb = &verts [ tris [ triangle_index + 1 ] * 3 ] ;
         const float *vc = &verts [ tris [ triangle_index + 2 ] * 3 ] ;

         points [ 0 ] = Ogre::Vector3 ( va [ 0 ], va [ 1 ] + 0.1f, va [ 2 ] ) ;
         points [ 1 ] = Ogre::Vector3 ( vb [ 0 ], vb [ 1 ] + 0.1f, vb [ 2 ] ) ;
         points [ 2 ] = Ogre::Vector3 ( vc [ 0 ], vc [ 1 ] + 0.1f, vc [ 2 ] ) ;

         DebugPolyTriangle new_triangle = { points, Ogre::Vector3 ( norm [ 0 ], norm [ 1 ], norm [ 2 ] ), colour } ;
         triangle_list.emplace_back ( new_triangle ) ;
      }

      InputMeshDebugId = CurrentDebugManager.CreateDebugTrianglePoly ( triangle_list, Ogre::ColourValue::Black ) ;
   }
}

void
NavMeshDebug::
DrawHeightField ( const rcHeightfield &height_field )
{
   const float* orig = height_field.bmin ;
   const float cs = height_field.cs ;
   const float ch = height_field.ch ;

   const int w = height_field.width ;
   const int h = height_field.height ;

   for ( int y = 0 ; y < h ; ++y )
   {
      for ( int x = 0 ; x < w ; ++x )
      {
         float fx = orig [ 0 ] + x * cs ;
         float fz = orig [ 2 ] + y * cs ;
         const rcSpan* s = height_field.spans [ x + y * w ] ;

         while ( s )
         {
            Ogre::Vector3 min ( fx, orig [ 1 ] + s->smin * ch, fz ) ;
            Ogre::Vector3 max ( fx + cs, orig [ 1 ] + s->smax * ch, fz + cs ) ;

            if ( std::abs ( min.y ) > 0.1f )
            {
               CurrentDebugManager.CreateDebugAABB ( min, max, Ogre::ColourValue ( 0.5f, 0.5f, 0.5f, 0.75f ) ) ;
            }

            s = s->next;
         }
      }
   }
}

void
NavMeshDebug::
RemoveAllTiles ()
{
   for ( auto &tile : TileList )
   {
      for ( auto &poly_id : tile.PolygonList )
      {
         CurrentDebugManager.DestroyDebugObject ( poly_id ) ;
      }

      for ( auto &dot_id : tile.DotList )
      {
         CurrentDebugManager.DestroyDebugObject ( dot_id ) ;
      }

      tile.PolygonList.clear () ;
      tile.DotList.clear () ;
   }

   TileList.clear () ;
}

void
NavMeshDebug::
RemoveAllObstacles ()
{
   for ( auto &obstacle : ObstacleList )
   {
      CurrentDebugManager.DestroyDebugObject ( obstacle.PolyId ) ;
   }

   ObstacleList.clear () ;
}

void
NavMeshDebug::
RemoveGrid ()
{
   if ( GridDebugId != INVALID_DEBUG_ID )
   {
      CurrentDebugManager.DestroyDebugObject ( GridDebugId ) ;
   }
}

void
NavMeshDebug::
RemoveInputMesh ()
{
   if ( InputMeshDebugId != INVALID_DEBUG_ID )
   {
      CurrentDebugManager.DestroyDebugObject ( InputMeshDebugId ) ;
   }
}

void
NavMeshDebug::
DrawTile ( const dtPolyRef      base_polyref,
           const dtNavMeshQuery &query,
           const dtMeshTile     &tile,
           Tile                 &debug_tile )
{
   // Draw polys that make up the tile
   debug_tile.PolygonList.push_back ( DrawTilePolys ( base_polyref, query, tile ) ) ;

   // Draw inter poly boundaries
   debug_tile.PolygonList.push_back ( DrawTilePolyBoundaries ( tile, Ogre::ColourValue ( 0, 0.188f, 0.25f, 0.125f ), 1.5f, true ) ) ;

   // Draw outer poly boundaries
   debug_tile.PolygonList.push_back ( DrawTilePolyBoundaries ( tile, Ogre::ColourValue ( 0, 0.188f, 0.25f, 0.86f ), 2.5f, false ) ) ;

   // Draw vertex 'dots'
   debug_tile.DotList.push_back ( DrawTileVertices ( tile, Ogre::ColourValue ( 0, 0, 0, 0.77f ), 0.25f ) ) ;
}

std::vector <DebugId>
NavMeshDebug::
DrawPolysWithFlags ( const dtNavMesh         &mesh,
                     const unsigned short    poly_flags,
                     const Ogre::ColourValue &colour )
{
   std::vector <DebugId> poly_debug_id_list ;

   for ( auto tile_index = 0 ; tile_index < mesh.getMaxTiles () ; ++tile_index )
   {
      const dtMeshTile *tile = mesh.getTile ( tile_index ) ;

      if ( tile->header )
      {
         dtPolyRef base = mesh.getPolyRefBase ( tile ) ;

         for ( auto poly_index = 0 ; poly_index < tile->header->polyCount ; ++poly_index )
         {
            const dtPoly *poly = &tile->polys [ poly_index ] ;

            if ( ( poly->flags & poly_flags ) != 0 )
            {
               const dtPolyRef  ref   = ( base | static_cast <dtPolyRef> ( poly_index ) ) ;
               const dtMeshTile *tile = nullptr ;
               const dtPoly     *poly = nullptr ;

               if ( ! dtStatusFailed ( mesh.getTileAndPolyByRef ( ref, &tile, &poly ) ) )
               {
                  DebugId debug_id = DrawNavMeshPoly ( *tile, *poly, colour ) ;

                  if ( debug_id != INVALID_DEBUG_ID )
                  {
                     poly_debug_id_list.push_back ( debug_id ) ;
                  }
               }
            }
         }
      }
   }

   return poly_debug_id_list ;
}

DebugId
NavMeshDebug::
DrawNavMeshPoly ( const dtMeshTile        &tile,
                  const dtPoly            &poly,
                  const Ogre::ColourValue &colour )
{
   const unsigned int poly_index = static_cast <unsigned int> ( &poly - tile.polys ) ;

   if ( poly.getType () != DT_POLYTYPE_OFFMESH_CONNECTION )
   {
      auto triangle_list = GenerateTriangleListFromPoly ( tile, poly, poly_index, colour ) ;

      return CurrentDebugManager.CreateDebugTrianglePoly ( triangle_list, Ogre::ColourValue::Black ) ;
   }

   return INVALID_DEBUG_ID ;
}

DebugId
NavMeshDebug::
DrawTilePolys ( const dtPolyRef      base_polyref,
                const dtNavMeshQuery &query,
                const dtMeshTile     &tile )
{
   std::vector <DebugPolyTriangle> triangle_list ;

   for ( auto poly_index = 0 ; poly_index < tile.header->polyCount ; ++poly_index )
   {
      const dtPoly *poly = &tile.polys [ poly_index ] ;

      if ( poly->getType () != DT_POLYTYPE_OFFMESH_CONNECTION )
      {
         Ogre::ColourValue colour = Ogre::ColourValue::Black ;

         if ( query.isInClosedList ( base_polyref | static_cast <dtPolyRef> ( poly_index ) ) )
         {
            colour = Ogre::ColourValue ( 1.0f, 0.75f, 0, 0.25f ) ;
         }
         else
         {
            colour = AreaToColour ( poly->getArea (), poly->flags ) ;
            colour.a = 0.25f ;
         }

         auto poly_triangle_list = GenerateTriangleListFromPoly ( tile, *poly, poly_index, colour ) ;
         triangle_list.insert ( triangle_list.end (), poly_triangle_list.begin (), poly_triangle_list.end () ) ;
      }
   }

   return CurrentDebugManager.CreateDebugTrianglePoly ( triangle_list, Ogre::ColourValue::Black, 2.5f ) ;
}

DebugId
NavMeshDebug::
DrawTilePolyBoundaries ( const dtMeshTile        &tile,
                         const Ogre::ColourValue &colour,
                         const float             line_width,
                         bool                    draw_areas )
{
   const float                 distance_threshold = 0.01f * 0.01f ;
   std::vector <DebugPolyLine> debug_poly_list ;

   for ( auto poly_index = 0 ; poly_index < tile.header->polyCount ; ++poly_index )
   {
      const dtPoly *poly = &tile.polys [ poly_index ] ;

      if ( poly->getType () != DT_POLYTYPE_OFFMESH_CONNECTION )
      {
         const dtPolyDetail *poly_detail = &tile.detailMeshes [ poly_index ] ;

         for ( auto poly_vertex_index = 0 ; poly_vertex_index < poly->vertCount ; ++poly_vertex_index )
         {
            DebugPolyLine debug_poly_line ;
            debug_poly_line.Colour = colour ;

            if ( draw_areas )
            {
               if ( poly->neis [ poly_vertex_index ] != 0 )
               {
                  if ( poly->neis [ poly_vertex_index ] & DT_EXT_LINK )
                  {
                     bool connected = false ;

                     for ( auto link_index = poly->firstLink ; link_index != DT_NULL_LINK ; link_index = tile.links [ link_index ].next )
                     {
                        if ( tile.links [ link_index ].edge == poly_vertex_index )
                        {
                           connected = true ;
                           break ;
                        }
                     }

                     if ( connected )
                     {
                        debug_poly_line.Colour = Ogre::ColourValue ( 1.0f, 1.0f, 1.0f, 0.188f ) ;
                     }
                     else
                     {
                        debug_poly_line.Colour = Ogre::ColourValue ( 0, 0, 0, 0.188f ) ;
                     }
                  }
                  else
                  {
                     debug_poly_line.Colour = Ogre::ColourValue ( 0, 0.188f, 0.25f, 0.125f ) ;
                  }
               }
            }

            if ( ! draw_areas ||
                 ( poly->neis [ poly_vertex_index ] == 0 ) )
            {
               const float *poly_start = &tile.verts [ poly->verts [ poly_vertex_index ] * 3 ] ;
               const float *poly_end   = &tile.verts [ poly->verts [ ( poly_vertex_index + 1 ) % poly->vertCount ] * 3 ] ;

               // Draw detail mesh edges which align with the actual poly edge.
               // This is really slow.
               for ( auto triangle_index = 0 ; triangle_index < poly_detail->triCount ; ++triangle_index )
               {
                  const unsigned char *triangle = &tile.detailTris [ ( poly_detail->triBase + triangle_index ) * 4 ] ;
                  const float         *triangle_vertex [ 3 ] ;

                  for ( auto vertex_element = 0 ; vertex_element < 3 ; ++vertex_element )
                  {
                     if ( triangle [ vertex_element ] < poly->vertCount )
                     {
                        triangle_vertex [ vertex_element ] = &tile.verts [ poly->verts [ triangle [ vertex_element ] ] * 3 ] ;
                     }
                     else
                     {
                        triangle_vertex [ vertex_element ] = &tile.detailVerts [ ( poly_detail->vertBase + ( triangle [ vertex_element ] - poly->vertCount ) ) * 3 ] ;
                     }
                  }

                  for ( auto vertex_element_index = 0, next_vertex_element_index = 2 ; vertex_element_index < 3 ; next_vertex_element_index = vertex_element_index++ )
                  {
                     if ( ( ( triangle [ 3 ] >> ( next_vertex_element_index * 2 ) ) & 0x3 ) != 0 )
                     {
                        if ( ( PointDistanceToLine2d ( triangle_vertex [ next_vertex_element_index ], poly_start, poly_end ) < distance_threshold ) &&
                             ( PointDistanceToLine2d ( triangle_vertex [ vertex_element_index ],      poly_start, poly_end ) < distance_threshold ) )
                        {
                           debug_poly_line.Points.push_back ( Ogre::Vector3 ( triangle_vertex [ next_vertex_element_index ] [ 0 ], triangle_vertex [ next_vertex_element_index ] [ 1 ], triangle_vertex [ next_vertex_element_index ] [ 2 ] ) ) ;
                           debug_poly_line.Points.push_back ( Ogre::Vector3 ( triangle_vertex [ vertex_element_index ] [ 0 ],      triangle_vertex [ vertex_element_index ] [ 1 ],      triangle_vertex [ vertex_element_index ] [ 2 ] ) ) ;
                        }
                     }
                  }
               }

               debug_poly_list.push_back ( debug_poly_line ) ;
            }
         }
      }
   }

   return CurrentDebugManager.CreateDebugPolys ( debug_poly_list, line_width ) ;
}

DebugId
NavMeshDebug::
DrawTileVertices ( const dtMeshTile        &tile,
                   const Ogre::ColourValue &colour,
                   const float             half_size )
{
   // Draw vertex 'dots'
   std::vector <DebugPolyTriangle> triangle_list ;

   for ( auto vert_index = 0 ; vert_index < tile.header->vertCount ; ++vert_index )
   {
      const float *vertex = &tile.verts [ vert_index * 3 ] ;

      auto dot_triangle_list = GenerateDot ( Ogre::Vector3 ( vertex [ 0 ], vertex [ 1 ], vertex [ 2 ] ), colour, half_size ) ;
      triangle_list.insert ( triangle_list.begin (), dot_triangle_list.begin (), dot_triangle_list.end () ) ;
   }

   return CurrentDebugManager.CreateDebugTrianglePoly ( triangle_list, Ogre::ColourValue::Black, 2.5f ) ;
}

DebugId
NavMeshDebug::
GeneratorBoxForObstacle ( const dtTileCacheObstacle &obstacle )
{
   if ( obstacle.type == DT_OBSTACLE_CYLINDER )
   {
      return CurrentDebugManager.CreateDebugCircle ( Ogre::Vector3 ( obstacle.cylinder.pos [ 0 ],
                                                                     obstacle.cylinder.pos [ 1 ],
                                                                     obstacle.cylinder.pos [ 2 ] ),
                                                     obstacle.cylinder.radius,
                                                     Ogre::ColourValue::Black ) ;
   }
   else if ( obstacle.type == DT_OBSTACLE_BOX )
   {
      Ogre::Vector3 min ;
      Ogre::Vector3 max ;

      //OgreRecast::FloatAToOgreVect3 ( obstacle.box.bmin, min ) ;
      //OgreRecast::FloatAToOgreVect3 ( obstacle.box.bmax, max ) ;

      OgreRecast::FloatAToOgreVect3 ( obstacle.BoundsMin, min ) ;
      OgreRecast::FloatAToOgreVect3 ( obstacle.BoundsMax, max ) ;

      return CurrentDebugManager.CreateDebugAABB ( min, max, Ogre::ColourValue ( 0.5f, 0.5f, 0.5f, 0.75f ) ) ;
   }
   else if ( obstacle.type == DT_OBSTACLE_ORIENTED_BOX )
   {
      Ogre::Vector3 centre ;
      Ogre::Vector3 half_extents ;

      OgreRecast::FloatAToOgreVect3 ( obstacle.orientedBox.center,      centre ) ;
      OgreRecast::FloatAToOgreVect3 ( obstacle.orientedBox.halfExtents, half_extents ) ;

      float left  = -half_extents.x ;
      float far_   = -half_extents.z ;
      float right = +half_extents.x ;
      float near_  = +half_extents.z ;

      auto rotate_point = [] ( float &x,
                               float &z,
                               const float rot_aux [ 2 ] )
                               {
                                  float x2   = 2.0f * x ;
                                  float z2   = 2.0f * z ;
                                  x = ( rot_aux [ 1 ] * x2 ) + ( rot_aux [ 0 ] * z2 ) ;
                                  z = ( rot_aux [ 1 ] * z2 ) - ( rot_aux [ 0 ] * x2 ) ;
                               } ;

      rotate_point ( left,  far_,  obstacle.orientedBox.rotAux ) ;
      rotate_point ( right, near_, obstacle.orientedBox.rotAux ) ;

      Ogre::Vector3 far_left_bottom   = centre + Ogre::Vector3 ( left,  -half_extents.y, far_ ) ;
      Ogre::Vector3 far_right_bottom  = centre + Ogre::Vector3 ( right, -half_extents.y, far_ ) ;
      Ogre::Vector3 near_left_bottom  = centre + Ogre::Vector3 ( left,  -half_extents.y, near_ ) ;
      Ogre::Vector3 near_right_bottom = centre + Ogre::Vector3 ( right, -half_extents.y, near_ ) ;
      Ogre::Vector3 far_left_top      = centre + Ogre::Vector3 ( left,  +half_extents.y, far_ ) ;
      Ogre::Vector3 far_right_top     = centre + Ogre::Vector3 ( right, +half_extents.y, far_ ) ;
      Ogre::Vector3 near_left_top     = centre + Ogre::Vector3 ( left,  +half_extents.y, near_ ) ;
      Ogre::Vector3 near_right_top    = centre + Ogre::Vector3 ( right, +half_extents.y, near_ ) ;

      std::vector <Ogre::Vector3> point_list = { far_left_bottom, far_left_top, far_right_top, far_right_bottom, // Far side
                                                 far_right_top, near_right_top, near_right_bottom, // Right side
                                                 near_right_top, near_left_top, near_left_bottom, // Near side
                                                 near_left_top, far_left_top } ; // Left size

      return CurrentDebugManager.CreateDebugPoly ( point_list, 0.1f, Ogre::ColourValue::Black ) ;
   }
   else if ( obstacle.type == DT_OBSTACLE_CONVEX_POLYGON )
   {
      SoftAssert ( obstacle.convexPolygon.nverts == 4 ) ;

      std::vector <Ogre::Vector3> point_list = { Ogre::Vector3 ( obstacle.convexPolygon.verts [ 0 ], obstacle.convexPolygon.verts [ 1 ], obstacle.convexPolygon.verts [ 2 ] ),
                                                 Ogre::Vector3 ( obstacle.convexPolygon.verts [ 3 ], obstacle.convexPolygon.verts [ 4 ], obstacle.convexPolygon.verts [ 5 ] ),
                                                 Ogre::Vector3 ( obstacle.convexPolygon.verts [ 6 ], obstacle.convexPolygon.verts [ 7 ], obstacle.convexPolygon.verts [ 8 ] ),
                                                 Ogre::Vector3 ( obstacle.convexPolygon.verts [ 9 ], obstacle.convexPolygon.verts [ 10 ], obstacle.convexPolygon.verts [ 11 ] ),
                                                 Ogre::Vector3 ( obstacle.convexPolygon.verts [ 0 ], obstacle.convexPolygon.verts [ 1 ], obstacle.convexPolygon.verts [ 2 ] ) } ;

      return CurrentDebugManager.CreateDebugPoly ( point_list, 0.1f, Ogre::ColourValue::Black ) ;
   }
   else
   {
      SoftAssert ( false && "Obstacle found with an invalid type" ) ;
      return INVALID_DEBUG_ID ;
   }
}

std::vector <DebugPolyTriangle>
NavMeshDebug::
GenerateTriangleListFromPoly ( const dtMeshTile        &tile,
                               const dtPoly            &poly,
                               const std::size_t       poly_index,
                               const Ogre::ColourValue &colour )
{
   std::vector <DebugPolyTriangle> triangle_list ;

   const dtPolyDetail *poly_detail = &tile.detailMeshes [ poly_index ] ;

   for ( auto triangle_index = 0 ; triangle_index < poly_detail->triCount ; ++triangle_index )
   {
      const unsigned char           *triangle = &tile.detailTris [ ( poly_detail->triBase + triangle_index ) * 4 ] ;
      std::array <Ogre::Vector3, 3> points ;

      for ( auto element_index = 0 ; element_index < 3 ; ++element_index )
      {
         if ( triangle [ element_index ] < poly.vertCount )
         {
            float *vertex = &tile.verts [ poly.verts [ triangle [ element_index ] ] * 3 ] ;

            points [ element_index ] = Ogre::Vector3 ( vertex [ 0 ], vertex [ 1 ], vertex [ 2 ] ) ;
         }
         else
         {
            float *vertex = &tile.detailVerts [ ( poly_detail->vertBase + triangle [ element_index ] - poly.vertCount ) * 3 ] ;

            points [ element_index ] = Ogre::Vector3 ( vertex [ 0 ], vertex [ 1 ], vertex [ 2 ] ) ;
         }
      }

      DebugPolyTriangle new_triangle = { points, Ogre::Vector3::UNIT_Y, colour } ;
      triangle_list.emplace_back ( new_triangle ) ;
   }

   return triangle_list ;
}

std::vector <DebugPolyTriangle>
NavMeshDebug::
GenerateDot ( const Ogre::Vector3     &position,
              const Ogre::ColourValue &colour,
              const float             half_size )
{
   std::vector <DebugPolyTriangle> dot_triangle_list ;

   const Ogre::Vector3 far_left   ( position.x - half_size, position.y + 0.1f, position.z - half_size ) ;
   const Ogre::Vector3 far_right  ( position.x + half_size, position.y + 0.1f, position.z - half_size ) ;
   const Ogre::Vector3 near_left  ( position.x - half_size, position.y + 0.1f, position.z + half_size ) ;
   const Ogre::Vector3 near_right ( position.x + half_size, position.y + 0.1f, position.z + half_size ) ;

   std::array <Ogre::Vector3, 3> first_half_points  = { far_left, near_right, far_right } ;
   std::array <Ogre::Vector3, 3> second_half_points = { far_left, near_left, near_right } ;

   DebugPolyTriangle first_triangle  = { first_half_points,  Ogre::Vector3::UNIT_Y, colour } ;
   DebugPolyTriangle second_triangle = { second_half_points, Ogre::Vector3::UNIT_Y, colour } ;

   dot_triangle_list.push_back ( first_triangle ) ;
   dot_triangle_list.push_back ( second_triangle ) ;

   return dot_triangle_list ;
}

