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
CreateAllTiles () ;

void
CreateAllObstacles () ;

DebugId
CreateDebugBoxForObstacle ( const dtTileCacheObstacle &obstacle ) ;

void
RemoveAllTiles () ;

void
RemoveAllObstacles () ;

void
DecompressAndDrawTile ( struct Tile &debug_tile ) ;

void
CreateDebugObjectsForTile ( struct Tile                      &debug_tile,
                            const struct dtTileCachePolyMesh &mesh,
                            const float                      *orig,
                            const float                      cs,
                            const float                      ch ) ;

NavMeshDebug::
NavMeshDebug ( OgreDetourTileCache &ogre_tile_cache ) :
   CurrentDebugManager ( DebugManager::GetDebugManager () ),
   OgreTileCache       ( ogre_tile_cache )
{
   DrawTiles     = false ;
   DrawObstacles = false ;
}

NavMeshDebug::
~NavMeshDebug ()
{
   RemoveAllTiles () ;
   RemoveAllObstacles () ;
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
         CreateAllTiles () ;
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
         CreateAllObstacles () ;
      }
   }
}

void
NavMeshDebug::
RedrawTile ( const std::size_t tile_x,
             const std::size_t tile_z )
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
      Tile new_tile = { tile_x, tile_z } ;
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

   DecompressAndDrawTile ( *tile_iter ) ;
}

void
NavMeshDebug::
AddObstacle ( const dtObstacleRef       &obstacle_ref,
              const dtTileCacheObstacle &obstacle )
{
   if ( DrawObstacles )
   {
      Obstacle new_obstacle = { obstacle_ref, CreateDebugBoxForObstacle ( obstacle ) } ;
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
CreateAllTiles ()
{
   const auto &tile_grid_size = OgreTileCache.GetTileGridSize () ;

   for ( auto tile_z = 0 ; tile_z < tile_grid_size.y ; ++tile_z )
   {
      for ( auto tile_x = 0 ; tile_x < tile_grid_size.x ; ++tile_x )
      {
         RedrawTile ( tile_x, tile_z ) ;
      }
   }
}

void
NavMeshDebug::
CreateAllObstacles ()
{
   auto &tile_cache = OgreTileCache.GetTileCache () ;

   for ( auto obstacle_index = 0 ; obstacle_index < tile_cache.getObstacleCount () ; ++obstacle_index )
   {
      const auto *obstacle = tile_cache.getObstacle ( obstacle_index ) ;

      if ( ( obstacle->state != DT_OBSTACLE_PROCESSING ) &&
           ( obstacle->state != DT_OBSTACLE_REMOVING ) )
      {
         const auto obstacle_ref = tile_cache.getObstacleRef ( obstacle ) ;

         // Create Obstacle if missing
         auto obstacle_iter = std::find_if ( ObstacleList.begin (),
                                             ObstacleList.end (),
                                             [obstacle_ref] ( const Obstacle &obstacle )
                                                {
                                                   return ( obstacle.ObstacleRef == obstacle_ref ) ;
                                                } ) ;

         if ( obstacle_iter == ObstacleList.end () )
         {
            Obstacle new_obstacle = { obstacle_ref, INVALID_DEBUG_ID } ;
            ObstacleList.emplace_back ( new_obstacle ) ;

            obstacle_iter = std::prev ( ObstacleList.end () ) ;
         }

         if ( obstacle_iter->PolyId == INVALID_DEBUG_ID )
         {
            obstacle_iter->PolyId = CreateDebugBoxForObstacle ( *obstacle ) ;
         }
      }
   }
}

DebugId
NavMeshDebug::
CreateDebugBoxForObstacle ( const dtTileCacheObstacle &obstacle )
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

      OgreRecast::FloatAToOgreVect3 ( obstacle.box.bmin, min ) ;
      OgreRecast::FloatAToOgreVect3 ( obstacle.box.bmax, max ) ;

      return CurrentDebugManager.CreateDebugAABB ( min, max, Ogre::ColourValue ( 0.5f, 0.5f, 0.5f, 0.5f ) ) ;
   }
   else if ( obstacle.type == DT_OBSTACLE_ORIENTED_BOX )
   {
      Ogre::Vector3 centre ;
      Ogre::Vector3 half_extents ;

      OgreRecast::FloatAToOgreVect3 ( obstacle.orientedBox.center,      centre ) ;
      OgreRecast::FloatAToOgreVect3 ( obstacle.orientedBox.halfExtents, half_extents ) ;

      float left  = -half_extents.x ;
      float far   = -half_extents.z ;
      float right = +half_extents.x ;
      float near  = +half_extents.z ;

      auto rotate_point = [] ( float &x,
                               float &z,
                               const float rot_aux [ 2 ] )
                               {
                                  float x2   = 2.0f * x ;
                                  float z2   = 2.0f * z ;
                                  x = ( rot_aux [ 1 ] * x2 ) + ( rot_aux [ 0 ] * z2 ) ;
                                  z = ( rot_aux [ 1 ] * z2 ) - ( rot_aux [ 0 ] * x2 ) ;
                               } ;

      rotate_point ( left,  far,  obstacle.orientedBox.rotAux ) ;
      rotate_point ( right, near, obstacle.orientedBox.rotAux ) ;

      Ogre::Vector3 far_left_bottom   = centre + Ogre::Vector3 ( left,  -half_extents.y, far ) ;
      Ogre::Vector3 far_right_bottom  = centre + Ogre::Vector3 ( right, -half_extents.y, far ) ;
      Ogre::Vector3 near_left_bottom  = centre + Ogre::Vector3 ( left,  -half_extents.y, near ) ;
      Ogre::Vector3 near_right_bottom = centre + Ogre::Vector3 ( right, -half_extents.y, near ) ;
      Ogre::Vector3 far_left_top      = centre + Ogre::Vector3 ( left,  +half_extents.y, far ) ;
      Ogre::Vector3 far_right_top     = centre + Ogre::Vector3 ( right, +half_extents.y, far ) ;
      Ogre::Vector3 near_left_top     = centre + Ogre::Vector3 ( left,  +half_extents.y, near ) ;
      Ogre::Vector3 near_right_top    = centre + Ogre::Vector3 ( right, +half_extents.y, near ) ;

      std::vector <Ogre::Vector3> point_list = { far_left_bottom, far_left_top, far_right_top, far_right_bottom, // Far side
                                                 far_right_top, near_right_top, near_right_bottom, // Right side
                                                 near_right_top, near_left_top, near_left_bottom, // Near side
                                                 near_left_top, far_left_top } ; // Left size

      return CurrentDebugManager.CreateDebugPoly ( point_list, 0.1f, Ogre::ColourValue::Black ) ;
   }
   else if ( obstacle.type == DT_OBSTACLE_CONVEX_POLYGON )
   {
      assert ( obstacle.convexPolygon.nverts == 4 ) ;

      std::vector <Ogre::Vector3> point_list = { Ogre::Vector3 ( obstacle.convexPolygon.verts [ 0 ], obstacle.convexPolygon.verts [ 1 ], obstacle.convexPolygon.verts [ 2 ] ),
                                                 Ogre::Vector3 ( obstacle.convexPolygon.verts [ 3 ], obstacle.convexPolygon.verts [ 4 ], obstacle.convexPolygon.verts [ 5 ] ),
                                                 Ogre::Vector3 ( obstacle.convexPolygon.verts [ 6 ], obstacle.convexPolygon.verts [ 7 ], obstacle.convexPolygon.verts [ 8 ] ),
                                                 Ogre::Vector3 ( obstacle.convexPolygon.verts [ 9 ], obstacle.convexPolygon.verts [ 10 ], obstacle.convexPolygon.verts [ 11 ] ),
                                                 Ogre::Vector3 ( obstacle.convexPolygon.verts [ 0 ], obstacle.convexPolygon.verts [ 1 ], obstacle.convexPolygon.verts [ 2 ] ) } ;

      return CurrentDebugManager.CreateDebugPoly ( point_list, 0.1f, Ogre::ColourValue::Black ) ;
   }
   else
   {
      assert ( false && "Obstacle found with an invalid type" ) ;
      return INVALID_DEBUG_ID ;
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
DecompressAndDrawTile ( Tile &debug_tile )
{
   struct TileCacheBuildContext
   {
      TileCacheBuildContext ( struct dtTileCacheAlloc *a ) :
         layer ( nullptr ),
         lcset ( nullptr ),
         lmesh ( nullptr ),
         alloc ( a )
      {
      }

      ~TileCacheBuildContext ()
      {
         purge () ;
      }

      void purge ()
      {
         dtFreeTileCacheLayer ( alloc, layer ) ;
         layer = nullptr ;

         dtFreeTileCacheContourSet ( alloc, lcset ) ;
         lcset = nullptr ;

         dtFreeTileCachePolyMesh ( alloc, lmesh ) ;
         lmesh = nullptr ;
      }

      struct dtTileCacheLayer      *layer ;
      struct dtTileCacheContourSet *lcset ;
      struct dtTileCachePolyMesh   *lmesh ;
      struct dtTileCacheAlloc      *alloc ;
   } ;

   dtCompressedTileRef     tiles [ MAX_LAYERS ] ;
   auto                    &tile_cache = OgreTileCache.GetTileCache () ;
   const int               ntiles      = tile_cache.getTilesAt ( debug_tile.TileXIndex, debug_tile.TileZIndex, tiles, MAX_LAYERS ) ;
   dtTileCacheAlloc        *talloc     = tile_cache.getAlloc () ;
   const dtTileCacheParams *params     = tile_cache.getParams () ;

   for ( int i = 0; i < ntiles; ++i )
   {
      const dtCompressedTile *tile = tile_cache.getTileByRef ( tiles [ i ] ) ;

      talloc->reset () ;

      TileCacheBuildContext bc ( talloc ) ;

      const int walkableClimbVx = ( int )( params->walkableClimb / params->ch ) ;
      dtStatus  status ;

      // Decompress tile layer data.
      status = dtDecompressTileCacheLayer ( talloc, tile_cache.getCompressor (), tile->data, tile->dataSize, &bc.layer ) ;

      if ( dtStatusFailed ( status ) )
         return ;

      // Build navmesh
      status = dtBuildTileCacheRegions ( talloc, *bc.layer, walkableClimbVx ) ;

      if ( dtStatusFailed ( status ) )
         return ;

      //TODO this part is replicated from navmesh tile building in DetourTileCache. Maybe that can be reused.
      // Also is it really necessary to do an extra navmesh rebuild from compressed tile just to draw it?
      // Can't I just draw it somewhere where the navmesh is rebuilt?
      bc.lcset = dtAllocTileCacheContourSet ( talloc ) ;

      if ( ! bc.lcset )
         return ;

      status = dtBuildTileCacheContours ( talloc, *bc.layer, walkableClimbVx, params->maxSimplificationError, *bc.lcset ) ;

      if ( dtStatusFailed ( status ) )
         return ;

      bc.lmesh = dtAllocTileCachePolyMesh ( talloc ) ;

      if ( ! bc.lmesh )
         return ;

      status = dtBuildTileCachePolyMesh ( talloc, *bc.lcset, *bc.lmesh ) ;

      if ( dtStatusFailed ( status ) )
         return ;

      // TODO this is a dirty quickfix that should be gone as soon as there is a rebuildTile(tileref) method
      CreateDebugObjectsForTile ( debug_tile, *bc.lmesh, tile->header->bmin, params->cs, params->ch ) ;
   }
}

void
NavMeshDebug::
CreateDebugObjectsForTile ( Tile                             &debug_tile,
                            const struct dtTileCachePolyMesh &mesh,
                            const float                      *orig,
                            const float                      cs,
                            const float                      ch )
{
   const int            nvp    = mesh.nvp;
   const unsigned short *verts = mesh.verts;
   const unsigned short *polys = mesh.polys;
   const unsigned char  *areas = mesh.areas;
   const int            npolys = mesh.npolys;
   const auto           m_navMeshOffsetFromGround = 0.1f;

   if ( npolys > 0 )
   {
      // Draw the tile itself
      std::vector <DebugPolyTriangle> tile_triangle_list ;

      for (int i = 0; i < npolys; ++i)
      {
         const unsigned short* p = &polys[i*nvp*2];

         unsigned short vi[3];
         for (int j = 2; j < nvp; ++j) // go through all verts in the polygon
         {
            if (p[j] == RC_MESH_NULL_IDX) break;
            vi[0] = p[0];
            vi[1] = p[j-1];
            vi[2] = p[j];

            std::array <Ogre::Vector3, 3> points ;
            Ogre::Vector3                 normal = Ogre::Vector3::UNIT_Y ;
            Ogre::ColourValue             colour = Ogre::ColourValue::Black ;

            for (int k = 0; k < 3; ++k) // create a 3-vert triangle for each 3 verts in the polygon.
            {
               const unsigned short* v = &verts[vi[k]*3];
               const float x = orig[0] + v[0]*cs;
               const float y = orig[1] + (v[1]/*+1*/)*ch;
               const float z = orig[2] + v[2]*cs;

               points [ k ] = Ogre::Vector3 ( x, y + m_navMeshOffsetFromGround, z ) ;

               switch ( areas [ i ] )
               {
               case POLYAREA_GRASS :
                  {
                     colour = DebugManager::GREEN ;
                     break ;
                  }
               case POLYAREA_WATER :
                  {
                     colour = DebugManager::BLUE ;
                     break ;
                  }
               case POLYAREA_ROAD :
                  {
                     colour = Ogre::ColourValue ( 0.3f, 0.3f, 0.3f ) ;
                     break ;
                  }
               case POLYAREA_SAND :
                  {
                     colour = DebugManager::GOLD ;
                     break ;
                  }
               case POLYAREA_GATE :
                  {
                     colour = DebugManager::ORANGE ;
                     break ;
                  }
               } ;
            }

            DebugPolyTriangle new_triangle = { points, normal, colour } ;
            tile_triangle_list.emplace_back ( new_triangle ) ;
         }
      }

      debug_tile.PolygonList.push_back ( CurrentDebugManager.CreateDebugTrianglePoly ( tile_triangle_list, Ogre::ColourValue::Black, true ) ) ;

      // Draw navmesh edges between neighbouring polygons
      for (int i = 0; i < npolys; ++i)
      {
         const unsigned short* p = &polys[i*nvp*2];

         for (int j = 0; j < nvp; ++j)
         {
            if (p[j] == RC_MESH_NULL_IDX) break;
            if (p[nvp+j] == RC_MESH_NULL_IDX) continue;
            int vi[2];
            vi[0] = p[j];
            if (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX)
               vi[1] = p[0];
            else
               vi[1] = p[j+1];

            std::vector <Ogre::Vector3> neighbour_edge_points ;

            for (int k = 0; k < 2; ++k)
            {
               const unsigned short* v = &verts[vi[k]*3];
               const float x = orig[0] + v[0]*cs;
               const float y = orig[1] + (v[1]/*+1*/)*ch /*+ 0.1f*/;
               const float z = orig[2] + v[2]*cs;

               neighbour_edge_points.push_back ( Ogre::Vector3 ( x, y + m_navMeshOffsetFromGround, z ) ) ;
            }

            //neighbour_edge_points.push_back ( Ogre::Vector3 ( -1,-1,-1 ) ) ;
            debug_tile.PolygonList.push_back ( CurrentDebugManager.CreateDebugPoly ( neighbour_edge_points, 0.0f, Ogre::ColourValue::Black ) ) ;
         }
      }

      // Draw navmesh outer edges (boundaries)
      for (int i = 0; i < npolys; ++i)
      {
         const unsigned short* p = &polys[i*nvp*2];

         for (int j = 0; j < nvp; ++j)
         {
            if (p[j] == RC_MESH_NULL_IDX) break;
            if (p[nvp+j] != RC_MESH_NULL_IDX) continue;
            int vi[2];
            vi[0] = p[j];
            if (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX)
               vi[1] = p[0];
            else
               vi[1] = p[j+1];

            std::vector <Ogre::Vector3> boundary_points ;

            for (int k = 0; k < 2; ++k)
            {
               const unsigned short* v = &verts[vi[k]*3];
               const float x = orig[0] + v[0]*cs;
               const float y = orig[1] + (v[1]/*+1*/)*ch /*+ 0.1f*/;
               const float z = orig[2] + v[2]*cs;

               boundary_points.push_back ( Ogre::Vector3 ( x, y + m_navMeshOffsetFromGround, z ) ) ;
            }

            //boundary_points.push_back ( Ogre::Vector3 ( -1,-1,-1 ) ) ;
            debug_tile.PolygonList.push_back ( CurrentDebugManager.CreateDebugPoly ( boundary_points, 0.0f, Ogre::ColourValue ( 0.5f, 0.5f, 0.5f ) ) ) ;
         }
      }
   }
}
