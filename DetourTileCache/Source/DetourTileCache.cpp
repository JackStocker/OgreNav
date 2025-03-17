#include "DetourTileCache.h"
#include "DetourTileCacheBuilder.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMesh.h"
#include "DetourCommon.h"
#include "DetourMath.h"
#include "DetourAlloc.h"
#include "DetourAssert.h"
#include <string.h>
#include <new>

#include "OgreRecastDefinitions.h" // For POLYAREA_GATE
#include <iostream>

dtTileCache* dtAllocTileCache()
{
   void* mem = dtAlloc(sizeof(dtTileCache), DT_ALLOC_PERM);
   if (!mem) return 0;
   return new(mem) dtTileCache;
}

void dtFreeTileCache(dtTileCache* tc)
{
   if (!tc) return;
   tc->~dtTileCache();
   dtFree(tc);
}

static bool contains(const dtCompressedTileRef* a, const int n, const dtCompressedTileRef v)
{
   for (int i = 0; i < n; ++i)
      if (a[i] == v)
         return true;
   return false;
}

inline int computeTileHash(int x, int y, const int mask)
{
   const unsigned int h1 = 0x8da6b343; // Large multiplicative constants;
   const unsigned int h2 = 0xd8163841; // here arbitrarily chosen primes
   unsigned int n = h1 * x + h2 * y;
   return (int)(n & mask);
}


struct NavMeshTileBuildContext
{
   inline NavMeshTileBuildContext(struct dtTileCacheAlloc* a) : layer(0), lcset(0), lmesh(0), alloc(a) {}
   inline ~NavMeshTileBuildContext() { purge(); }
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
};


dtTileCache::dtTileCache() :
   m_tileLutSize(0),
   m_tileLutMask(0),
   m_posLookup(0),
   m_nextFreeTile(0),
   m_tiles(0),
   m_saltBits(0),
   m_tileBits(0),
   m_talloc(0),
   m_tcomp(0),
   m_tmproc(0),
   m_obstacles(0),
   m_nextFreeObstacle(0),
   m_nreqs(0),
   m_nupdate(0)
{
   memset(&m_params, 0, sizeof(m_params));
   memset(m_reqs, 0, sizeof(ObstacleRequest) * MAX_REQUESTS);
}

dtTileCache::~dtTileCache()
{
   for (int i = 0; i < m_params.maxTiles; ++i)
   {
      if (m_tiles[i].flags & DT_COMPRESSEDTILE_FREE_DATA)
      {
         dtFree(m_tiles[i].data);
         m_tiles[i].data = 0;
      }
   }
   dtFree(m_obstacles);
   m_obstacles = 0;
   dtFree(m_posLookup);
   m_posLookup = 0;
   dtFree(m_tiles);
   m_tiles = 0;
   m_nreqs = 0;
   m_nupdate = 0;
}

const dtCompressedTile* dtTileCache::getTileByRef(dtCompressedTileRef ref) const
{
   if (!ref)
      return 0;
   unsigned int tileIndex = decodeTileIdTile(ref);
   unsigned int tileSalt = decodeTileIdSalt(ref);
   if ((int)tileIndex >= m_params.maxTiles)
      return 0;
   const dtCompressedTile* tile = &m_tiles[tileIndex];
   if (tile->salt != tileSalt)
      return 0;
   return tile;
}


dtStatus dtTileCache::init(const dtTileCacheParams* params,
                     dtTileCacheAlloc* talloc,
                     dtTileCacheCompressor* tcomp,
                     dtTileCacheMeshProcess* tmproc)
{
   m_talloc = talloc;
   m_tcomp = tcomp;
   m_tmproc = tmproc;
   m_nreqs = 0;
   memcpy(&m_params, params, sizeof(m_params));

   // Alloc space for obstacles.
   m_obstacles = (dtTileCacheObstacle*)dtAlloc(sizeof(dtTileCacheObstacle)*m_params.maxObstacles, DT_ALLOC_PERM);
   if (!m_obstacles)
      return DT_FAILURE | DT_OUT_OF_MEMORY;
   memset(m_obstacles, 0, sizeof(dtTileCacheObstacle)*m_params.maxObstacles);
   m_nextFreeObstacle = 0;
   for (int i = m_params.maxObstacles-1; i >= 0; --i)
   {
      m_obstacles[i].salt = 1;
      m_obstacles[i].next = m_nextFreeObstacle;
      m_nextFreeObstacle = &m_obstacles[i];
   }

   // Init tiles
   m_tileLutSize = dtNextPow2(m_params.maxTiles/4);
   if (!m_tileLutSize) m_tileLutSize = 1;
   m_tileLutMask = m_tileLutSize-1;

   m_tiles = (dtCompressedTile*)dtAlloc(sizeof(dtCompressedTile)*m_params.maxTiles, DT_ALLOC_PERM);
   if (!m_tiles)
      return DT_FAILURE | DT_OUT_OF_MEMORY;
   m_posLookup = (dtCompressedTile**)dtAlloc(sizeof(dtCompressedTile*)*m_tileLutSize, DT_ALLOC_PERM);
   if (!m_posLookup)
      return DT_FAILURE | DT_OUT_OF_MEMORY;
   memset(m_tiles, 0, sizeof(dtCompressedTile)*m_params.maxTiles);
   memset(m_posLookup, 0, sizeof(dtCompressedTile*)*m_tileLutSize);
   m_nextFreeTile = 0;
   for (int i = m_params.maxTiles-1; i >= 0; --i)
   {
      m_tiles[i].salt = 1;
      m_tiles[i].next = m_nextFreeTile;
      m_nextFreeTile = &m_tiles[i];
   }

   // Init ID generator values.
   m_tileBits = dtIlog2(dtNextPow2((unsigned int)m_params.maxTiles));
   // Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
   m_saltBits = dtMin((unsigned int)31, 32 - m_tileBits);
   if (m_saltBits < 10)
      return DT_FAILURE | DT_INVALID_PARAM;

   return DT_SUCCESS;
}

int dtTileCache::getTilesAt(const int tx, const int ty, dtCompressedTileRef* tiles, const int maxTiles) const
{
   int n = 0;

   // Find tile based on hash.
   int h = computeTileHash(tx,ty,m_tileLutMask);
   dtCompressedTile* tile = m_posLookup[h];
   while (tile)
   {
      if (tile->header &&
         tile->header->tx == tx &&
         tile->header->ty == ty)
      {
         if (n < maxTiles)
            tiles[n++] = getTileRef(tile);
      }
      tile = tile->next;
   }

   return n;
}

dtCompressedTile* dtTileCache::getTileAt(const int tx, const int ty, const int tlayer)
{
   // Find tile based on hash.
   int h = computeTileHash(tx,ty,m_tileLutMask);
   dtCompressedTile* tile = m_posLookup[h];
   while (tile)
   {
      if (tile->header &&
         tile->header->tx == tx &&
         tile->header->ty == ty &&
         tile->header->tlayer == tlayer)
      {
         return tile;
      }
      tile = tile->next;
   }
   return 0;
}

dtCompressedTileRef dtTileCache::getTileRef(const dtCompressedTile* tile) const
{
   if (!tile) return 0;
   const unsigned int it = (unsigned int)(tile - m_tiles);
   return (dtCompressedTileRef)encodeTileId(tile->salt, it);
}

dtObstacleRef dtTileCache::getObstacleRef(const dtTileCacheObstacle* ob) const
{
   if (!ob) return 0;
   const unsigned int idx = (unsigned int)(ob - m_obstacles);
   return encodeObstacleId(ob->salt, idx);
}

const dtTileCacheObstacle* dtTileCache::getObstacleByRef(dtObstacleRef ref)
{
   if (!ref)
      return 0;
   unsigned int idx = decodeObstacleIdObstacle(ref);
   if ((int)idx >= m_params.maxObstacles)
      return 0;
   const dtTileCacheObstacle* ob = &m_obstacles[idx];
   unsigned int salt = decodeObstacleIdSalt(ref);
   if (ob->salt != salt)
      return 0;
   return ob;
}

dtStatus
dtTileCache::
SetObstacleFlags ( dtNavMesh           &navmesh,
                   dtTileCacheObstacle &obstacle )
{
   if ( ( obstacle.state == DT_OBSTACLE_PROCESSED ) &&
        ( obstacle.area_id == POLYAREA_GATE ) ) // Currently only GATE obstacles apply flags
   {
      // Obstacles must apply their flags to the specific polygons that they cover to mark the area.
      // We don't know which polygons were created because of each obstacle so we have to query the tiles that are under us,
      // then determine which polygon(s) are within the obstacles bounds, and apply them directly.
      for ( auto tile_index = 0 ; tile_index < obstacle.ntouched ; ++tile_index )
      {
         const auto         tile_ref  = obstacle.touched [ tile_index ] ;
         const unsigned int tile_id   = decodeTileIdTile ( tile_ref ) ;
         const unsigned int tile_salt = decodeTileIdSalt ( tile_ref ) ;

         if ( static_cast <int> ( tile_id ) < m_params.maxTiles )
         {
            dtCompressedTile &tile = m_tiles [ tile_id ] ;

            if ( tile.salt == tile_salt )
            {
               auto* dt_tile = navmesh.getTileAt ( tile.header->tx, tile.header->ty, tile.header->tlayer ) ;

               if ( dt_tile ) // I guess this could change if the tile is rebuilt?
               {
                  for ( auto poly_index = 0 ; poly_index < dt_tile->header->polyCount ; ++poly_index )
                  {
                     auto &poly = dt_tile->polys [ poly_index ] ;

                     if ( poly.getArea () == POLYAREA_GATE )
                     {
                        // Determine if this polygon is under the obstacle by getting the centre of the polygon and checking it is within the obstacle bounds
                        float poly_middle_x = 0 ;
                        float poly_middle_y = 0 ;
                        float poly_middle_z = 0 ;

                        for ( auto poly_vertex_index = 0U ; poly_vertex_index < poly.vertCount ; ++poly_vertex_index )
                        {
                           const float* vertex = &dt_tile->verts [ poly.verts [ poly_vertex_index ] * 3 ] ;

                           poly_middle_x += vertex [ 0 ] ;
                           poly_middle_y += vertex [ 1 ] ;
                           poly_middle_z += vertex [ 2 ] ;
                        }

                        poly_middle_x /= poly.vertCount ;
                        poly_middle_y /= poly.vertCount ;
                        poly_middle_z /= poly.vertCount ;

                        // Only the polygons that are within the obstacle bounds should have the flags applied, otherwise we will apply GATE flags to every polygon in the tile
                        if ( ( poly_middle_x >= obstacle.BoundsMin [ 0 ] ) &&
                             ( poly_middle_x <= obstacle.BoundsMax [ 0 ] ) &&
                             ( poly_middle_y >= obstacle.BoundsMin [ 1 ] ) &&
                             ( poly_middle_y <= obstacle.BoundsMax [ 1 ] ) &&
                             ( poly_middle_z >= obstacle.BoundsMin [ 2 ] ) &&
                             ( poly_middle_z <= obstacle.BoundsMax [ 2 ] ) )
                        {
                           poly.flags = obstacle.flag ;
                        }
                     }
                  }
               }
            }
         }
      }

      return DT_SUCCESS ;
   }
   else
   {
      return DT_FAILURE ;
   }
}

void
dtTileCache::
ApplyObstacleFlagsToTile ( dtNavMesh                 &navmesh,
                           const dtCompressedTile    &tile,
                           const dtCompressedTileRef ref )
{
   // Obstacles must apply their flags to the specific polygons that they cover to mark the area.
   // We don't know which polygons were created because of each obstacle so we have to look through them and find the tiles that are touched,
   // then determine which polygon(s) are within the obstacles bounds, and apply them directly.
   auto* dt_tile = navmesh.getTileAt ( tile.header->tx, tile.header->ty, tile.header->tlayer ) ;

   if ( dt_tile ) // I guess this could change if the tile is rebuilt?
   {
      for ( auto poly_index = 0 ; poly_index < dt_tile->header->polyCount ; ++poly_index )
      {
         auto& poly = dt_tile->polys [ poly_index ] ;

         if ( poly.getArea () == POLYAREA_GATE )
         {
            // Determine if this polygon is under the obstacle by getting the centre of the polygon and checking it is within the obstacle bounds
            float poly_middle_x = 0 ;
            float poly_middle_y = 0 ;
            float poly_middle_z = 0 ;

            for ( auto poly_vertex_index = 0U ; poly_vertex_index < poly.vertCount ; ++poly_vertex_index )
            {
               const float* vertex = &dt_tile->verts [ poly.verts [ poly_vertex_index ] * 3 ] ;

               poly_middle_x += vertex [ 0 ] ;
               poly_middle_y += vertex [ 1 ] ;
               poly_middle_z += vertex [ 2 ] ;
            }

            poly_middle_x /= poly.vertCount ;
            poly_middle_y /= poly.vertCount ;
            poly_middle_z /= poly.vertCount ;

            for ( int i = 0; i < m_params.maxObstacles; ++i )
            {
               const auto &obstacle = m_obstacles [ i ] ;

               if ( ( ( obstacle.state == DT_OBSTACLE_PROCESSING ) || // When we are adding a new obstacle it will still be processing until buildNavMeshTile has returned
                      ( obstacle.state == DT_OBSTACLE_PROCESSED ) ) &&
                    ( obstacle.area_id == POLYAREA_GATE ) && // Currently only GATE obstacles apply flags
                    contains ( obstacle.touched, obstacle.ntouched, ref ) ) // We only care if this obstacle touches this tile
               {
                  // Only the polygons that are within the obstacle bounds should have the flags applied, otherwise we will apply GATE flags to every polygon in the tile
                  if ( ( poly_middle_x >= obstacle.BoundsMin [ 0 ] ) &&
                       ( poly_middle_x <= obstacle.BoundsMax [ 0 ] ) &&
                       ( poly_middle_y >= obstacle.BoundsMin [ 1 ] ) &&
                       ( poly_middle_y <= obstacle.BoundsMax [ 1 ] ) &&
                       ( poly_middle_z >= obstacle.BoundsMin [ 2 ] ) &&
                       ( poly_middle_z <= obstacle.BoundsMax [ 2 ] ) )
                  {
                     poly.flags = obstacle.flag ;
                  }
               }
            }
         }
      }
   }
}

dtStatus dtTileCache::addTile(unsigned char* data, const int dataSize, unsigned char flags, dtCompressedTileRef* result)
{
   // Make sure the data is in right format.
   dtTileCacheLayerHeader* header = (dtTileCacheLayerHeader*)data;
   if (header->magic != DT_TILECACHE_MAGIC)
      return DT_FAILURE | DT_WRONG_MAGIC;
   if (header->version != DT_TILECACHE_VERSION)
      return DT_FAILURE | DT_WRONG_VERSION;

   // Make sure the location is free.
   if (getTileAt(header->tx, header->ty, header->tlayer))
      return DT_FAILURE;

   // Allocate a tile.
   dtCompressedTile* tile = 0;
   if (m_nextFreeTile)
   {
      tile = m_nextFreeTile;
      m_nextFreeTile = tile->next;
      tile->next = 0;
   }

   // Make sure we could allocate a tile.
   if (!tile)
      return DT_FAILURE | DT_OUT_OF_MEMORY;

   // Insert tile into the position lut.
   int h = computeTileHash(header->tx, header->ty, m_tileLutMask);
   tile->next = m_posLookup[h];
   m_posLookup[h] = tile;

   // Init tile.
   const int headerSize = dtAlign4(sizeof(dtTileCacheLayerHeader));
   tile->header = (dtTileCacheLayerHeader*)data;
   tile->data = data;
   tile->dataSize = dataSize;
   tile->compressed = tile->data + headerSize;
   tile->compressedSize = tile->dataSize - headerSize;
   tile->flags = flags;

   if (result)
      *result = getTileRef(tile);

   return DT_SUCCESS;
}

dtStatus dtTileCache::removeTile(dtCompressedTileRef ref, unsigned char** data, int* dataSize)
{
   if (!ref)
      return DT_FAILURE | DT_INVALID_PARAM;
   unsigned int tileIndex = decodeTileIdTile(ref);
   unsigned int tileSalt = decodeTileIdSalt(ref);
   if ((int)tileIndex >= m_params.maxTiles)
      return DT_FAILURE | DT_INVALID_PARAM;
   dtCompressedTile* tile = &m_tiles[tileIndex];
   if (tile->salt != tileSalt)
      return DT_FAILURE | DT_INVALID_PARAM;

   // Remove tile from hash lookup.
   const int h = computeTileHash(tile->header->tx,tile->header->ty,m_tileLutMask);
   dtCompressedTile* prev = 0;
   dtCompressedTile* cur = m_posLookup[h];
   while (cur)
   {
      if (cur == tile)
      {
         if (prev)
            prev->next = cur->next;
         else
            m_posLookup[h] = cur->next;
         break;
      }
      prev = cur;
      cur = cur->next;
   }

   // Reset tile.
   if (tile->flags & DT_COMPRESSEDTILE_FREE_DATA)
   {
      // Owns data
      dtFree(tile->data);
      tile->data = 0;
      tile->dataSize = 0;
      if (data) *data = 0;
      if (dataSize) *dataSize = 0;
   }
   else
   {
      if (data) *data = tile->data;
      if (dataSize) *dataSize = tile->dataSize;
   }

   tile->header = 0;
   tile->data = 0;
   tile->dataSize = 0;
   tile->compressed = 0;
   tile->compressedSize = 0;
   tile->flags = 0;

   // Update salt, salt should never be zero.
   tile->salt = (tile->salt+1) & ((1<<m_saltBits)-1);
   if (tile->salt == 0)
      tile->salt++;

   // Add to free list.
   tile->next = m_nextFreeTile;
   m_nextFreeTile = tile;

   return DT_SUCCESS;
}


dtStatus dtTileCache::addObstacle(const float* pos, const float radius, const float height, dtObstacleRef* result,
                                  const unsigned char  area_id,
                                  const unsigned short flag )
{
   if (m_nreqs >= MAX_REQUESTS)
      return DT_FAILURE | DT_BUFFER_TOO_SMALL;

   dtTileCacheObstacle* ob = 0;
   if (m_nextFreeObstacle)
   {
      ob = m_nextFreeObstacle;
      m_nextFreeObstacle = ob->next;
      ob->next = 0;
   }
   if (!ob)
      return DT_FAILURE | DT_OUT_OF_MEMORY;

   unsigned short salt = ob->salt;
   memset(ob, 0, sizeof(dtTileCacheObstacle));
   ob->salt = salt;
   ob->state = DT_OBSTACLE_PROCESSING;
   ob->type = DT_OBSTACLE_CYLINDER;
   dtVcopy(ob->cylinder.pos, pos);
   ob->cylinder.radius = radius;
   ob->cylinder.height = height;

   ob->area_id = area_id ;
   ob->flag    = flag ;

   getObstacleBounds ( ob, ob->BoundsMin, ob->BoundsMax ) ;

   ObstacleRequest* req = &m_reqs[m_nreqs++];
   memset(req, 0, sizeof(ObstacleRequest));
   req->action = REQUEST_ADD;
   req->ref = getObstacleRef(ob);

   if (result)
      *result = req->ref;

   return DT_SUCCESS;
}

dtStatus dtTileCache::addBoxObstacle(const float* bmin, const float* bmax, dtObstacleRef* result,
                                     const unsigned char  area_id,
                                     const unsigned short flag )
{
   if (m_nreqs >= MAX_REQUESTS)
      return DT_FAILURE | DT_BUFFER_TOO_SMALL;

   dtTileCacheObstacle* ob = 0;
   if (m_nextFreeObstacle)
   {
      ob = m_nextFreeObstacle;
      m_nextFreeObstacle = ob->next;
      ob->next = 0;
   }
   if (!ob)
      return DT_FAILURE | DT_OUT_OF_MEMORY;

   unsigned short salt = ob->salt;
   memset(ob, 0, sizeof(dtTileCacheObstacle));
   ob->salt = salt;
   ob->state = DT_OBSTACLE_PROCESSING;
   ob->type = DT_OBSTACLE_BOX;
   //dtVcopy(ob->box.bmin, bmin);
   //dtVcopy(ob->box.bmax, bmax);

   dtVcopy(ob->BoundsMin, bmin);
   dtVcopy(ob->BoundsMax, bmax);
   getObstacleBounds ( ob, ob->BoundsMin, ob->BoundsMax ) ;

   ob->area_id = area_id ;
   ob->flag    = flag ;

   ObstacleRequest* req = &m_reqs[m_nreqs++];
   memset(req, 0, sizeof(ObstacleRequest));
   req->action = REQUEST_ADD;
   req->ref = getObstacleRef(ob);

   if (result)
      *result = req->ref;

   return DT_SUCCESS;
}

dtStatus dtTileCache::addBoxObstacle(const float* center, const float* halfExtents, const float yRadians, dtObstacleRef* result,
                                     const unsigned char  area_id,
                                     const unsigned short flag )
{
   if (m_nreqs >= MAX_REQUESTS)
      return DT_FAILURE | DT_BUFFER_TOO_SMALL;

   dtTileCacheObstacle* ob = 0;
   if (m_nextFreeObstacle)
   {
      ob = m_nextFreeObstacle;
      m_nextFreeObstacle = ob->next;
      ob->next = 0;
   }
   if (!ob)
      return DT_FAILURE | DT_OUT_OF_MEMORY;

   unsigned short salt = ob->salt;
   memset(ob, 0, sizeof(dtTileCacheObstacle));
   ob->salt = salt;
   ob->state = DT_OBSTACLE_PROCESSING;
   ob->type = DT_OBSTACLE_ORIENTED_BOX;
   dtVcopy(ob->orientedBox.center, center);
   dtVcopy(ob->orientedBox.halfExtents, halfExtents);

   float coshalf= cosf(0.5f*yRadians);
   float sinhalf = sinf(-0.5f*yRadians);
   ob->orientedBox.rotAux[0] = coshalf*sinhalf;
   ob->orientedBox.rotAux[1] = coshalf*coshalf - 0.5f;

   ob->area_id = area_id ;
   ob->flag    = flag ;

   getObstacleBounds ( ob, ob->BoundsMin, ob->BoundsMax ) ;

   ObstacleRequest* req = &m_reqs[m_nreqs++];
   memset(req, 0, sizeof(ObstacleRequest));
   req->action = REQUEST_ADD;
   req->ref = getObstacleRef(ob);

   if (result)
      *result = req->ref;

   return DT_SUCCESS;
}

// Add polygon obstacle
dtStatus
dtTileCache::
addPolygonObstacle ( const float   *convexHullVertices,
                     int           numConvexHullVertices,
                     const float   height,
                     dtObstacleRef *result,
                     const unsigned char  area_id,
                     const unsigned short flag )
{
   if ( m_nreqs >= MAX_REQUESTS )
   {
      return DT_FAILURE | DT_BUFFER_TOO_SMALL ;
   }

   dtTileCacheObstacle* ob = 0 ;

   if ( m_nextFreeObstacle )
   {
      ob = m_nextFreeObstacle ;
      m_nextFreeObstacle = ob->next ;
      ob->next = 0 ;
   }

   if ( !ob )
   {
      return DT_FAILURE | DT_OUT_OF_MEMORY ;
   }

   unsigned short salt = ob->salt ;
   memset ( ob, 0, sizeof ( dtTileCacheObstacle ) ) ;
   ob->salt = salt ;
   ob->state = DT_OBSTACLE_PROCESSING ;
   ob->type = DT_OBSTACLE_CONVEX_POLYGON;
   ob->convexPolygon.height = height ;
   ob->convexPolygon.nverts = numConvexHullVertices ;

   for ( int i = 0 ; i < ob->convexPolygon.nverts ; i++ )
   {
      dtVcopy ( &ob->convexPolygon.verts [ i * 3 ], &convexHullVertices [ i * 3 ] ) ;
   }

   ob->area_id = area_id ;
   ob->flag    = flag ;

   getObstacleBounds ( ob, ob->BoundsMin, ob->BoundsMax ) ;

   ObstacleRequest* req = &m_reqs [ m_nreqs++ ] ;
   memset ( req, 0, sizeof ( ObstacleRequest ) ) ;
   req->action = REQUEST_ADD ;
   req->ref = getObstacleRef ( ob ) ;

   if ( result )
   {
      *result = req->ref ;
   }

   return DT_SUCCESS ;
}

dtStatus dtTileCache::removeObstacle(const dtObstacleRef ref)
{
   if (!ref)
      return DT_SUCCESS;
   if (m_nreqs >= MAX_REQUESTS)
      return DT_FAILURE | DT_BUFFER_TOO_SMALL;

   ObstacleRequest* req = &m_reqs[m_nreqs++];
   memset(req, 0, sizeof(ObstacleRequest));
   req->action = REQUEST_REMOVE;
   req->ref = ref;

   return DT_SUCCESS;
}


dtStatus dtTileCache::queryTiles(const float* bmin, const float* bmax,
                         dtCompressedTileRef* results, int* resultCount, const int maxResults) const
{
   const int MAX_TILES = 32;
   dtCompressedTileRef tiles[MAX_TILES];

   int n = 0;

   const float tw = m_params.width * m_params.cs;
   const float th = m_params.height * m_params.cs;
   const int tx0 = (int)dtMathFloorf((bmin[0]-m_params.orig[0]) / tw);
   const int tx1 = (int)dtMathFloorf((bmax[0]-m_params.orig[0]) / tw);
   const int ty0 = (int)dtMathFloorf((bmin[2]-m_params.orig[2]) / th);
   const int ty1 = (int)dtMathFloorf((bmax[2]-m_params.orig[2]) / th);

   for (int ty = ty0; ty <= ty1; ++ty)
   {
      for (int tx = tx0; tx <= tx1; ++tx)
      {
         const int ntiles = getTilesAt(tx,ty,tiles,MAX_TILES);

         for (int i = 0; i < ntiles; ++i)
         {
            const dtCompressedTile* tile = &m_tiles[decodeTileIdTile(tiles[i])];
            float tbmin[3], tbmax[3];
            calcTightTileBounds(tile->header, tbmin, tbmax);

            if (dtOverlapBounds(bmin,bmax, tbmin,tbmax))
            {
               if (n < maxResults)
                  results[n++] = tiles[i];
            }
         }
      }
   }

   *resultCount = n;

   return DT_SUCCESS;
}

dtStatus dtTileCache::update(const float /*dt*/, dtNavMesh* navmesh,
                      bool* upToDate)
{
   if (m_nupdate == 0)
   {
      // Process requests.
      for (int i = 0; i < m_nreqs; ++i)
      {
         ObstacleRequest* req = &m_reqs[i];

         unsigned int idx = decodeObstacleIdObstacle(req->ref);
         if ((int)idx >= m_params.maxObstacles)
            continue;
         dtTileCacheObstacle* ob = &m_obstacles[idx];
         unsigned int salt = decodeObstacleIdSalt(req->ref);
         if (ob->salt != salt)
            continue;

         if (req->action == REQUEST_ADD)
         {
            // Find touched tiles.
            //float bmin[3], bmax[3];
            //getObstacleBounds(ob, bmin, bmax);

            int ntouched = 0;
            //queryTiles(bmin, bmax, ob->touched, &ntouched, DT_MAX_TOUCHED_TILES);
            queryTiles ( ob->BoundsMin, ob->BoundsMax, ob->touched, &ntouched, DT_MAX_TOUCHED_TILES );
            ob->ntouched = (unsigned char)ntouched;
            // Add tiles to update list.
            ob->npending = 0;
            for (int j = 0; j < ob->ntouched; ++j)
            {
               if (m_nupdate < MAX_UPDATE)
               {
                  if (!contains(m_update, m_nupdate, ob->touched[j]))
                     m_update[m_nupdate++] = ob->touched[j];
                  ob->pending[ob->npending++] = ob->touched[j];
               }
            }
         }
         else if (req->action == REQUEST_REMOVE)
         {
            // Prepare to remove obstacle.
            ob->state = DT_OBSTACLE_REMOVING;
            // Add tiles to update list.
            ob->npending = 0;
            for (int j = 0; j < ob->ntouched; ++j)
            {
               if (m_nupdate < MAX_UPDATE)
               {
                  if (!contains(m_update, m_nupdate, ob->touched[j]))
                     m_update[m_nupdate++] = ob->touched[j];
                  ob->pending[ob->npending++] = ob->touched[j];
               }
            }
         }
      }

      m_nreqs = 0;
   }

   dtStatus status = DT_SUCCESS;
   // Process updates
   if (m_nupdate)
   {
      // Build mesh
      const dtCompressedTileRef ref = m_update[0];
      status = buildNavMeshTile(ref, navmesh);
      m_nupdate--;
      if (m_nupdate > 0)
         memmove(m_update, m_update+1, m_nupdate*sizeof(dtCompressedTileRef));

      // Update obstacle states.
      for (int i = 0; i < m_params.maxObstacles; ++i)
      {
         dtTileCacheObstacle* ob = &m_obstacles[i];
         if (ob->state == DT_OBSTACLE_PROCESSING || ob->state == DT_OBSTACLE_REMOVING)
         {
            // Remove handled tile from pending list.
            for (int j = 0; j < (int)ob->npending; j++)
            {
               if (ob->pending[j] == ref)
               {
                  ob->pending[j] = ob->pending[(int)ob->npending-1];
                  ob->npending--;
                  break;
               }
            }

            // If all pending tiles processed, change state.
            if (ob->npending == 0)
            {
               if (ob->state == DT_OBSTACLE_PROCESSING)
               {
                  ob->state = DT_OBSTACLE_PROCESSED;
               }
               else if (ob->state == DT_OBSTACLE_REMOVING)
               {
                  ob->state = DT_OBSTACLE_EMPTY;
                  // Update salt, salt should never be zero.
                  ob->salt = (ob->salt+1) & ((1<<16)-1);
                  if (ob->salt == 0)
                     ob->salt++;
                  // Return obstacle to free list.
                  ob->next = m_nextFreeObstacle;
                  m_nextFreeObstacle = ob;
               }
            }
         }
      }
   }

   if (upToDate)
      *upToDate = m_nupdate == 0 && m_nreqs == 0;

   // Below has been replaced by applying the flags when the tile is being built, so we don't need to search through the tile list
   //
   // After modifying the mesh we must re-apply any obstacle flags as they are cleared when the tiles are rebuilt
   /*{
      // Re-apply the obstacle flags
      for ( int i = 0; i < m_params.maxObstacles; ++i )
      {
         dtTileCacheObstacle* ob = &m_obstacles[ i ];

         if ( ob->state == DT_OBSTACLE_PROCESSED )
         {
            SetObstacleFlags ( *navmesh, *ob, i ) ;
         }
      }
   }*/

   return status;
}


dtStatus dtTileCache::buildNavMeshTilesAt(const int tx, const int ty, dtNavMesh* navmesh)
{
   const int MAX_TILES = 32;
   dtCompressedTileRef tiles[MAX_TILES];
   const int ntiles = getTilesAt(tx,ty,tiles,MAX_TILES);

   for (int i = 0; i < ntiles; ++i)
   {
      dtStatus status = buildNavMeshTile(tiles[i], navmesh);
      if (dtStatusFailed(status))
         return status;
   }

   return DT_SUCCESS;
}

dtStatus dtTileCache::buildNavMeshTile(const dtCompressedTileRef ref, dtNavMesh* navmesh)
{
   dtAssert(m_talloc);
   dtAssert(m_tcomp);

   unsigned int idx = decodeTileIdTile(ref);
   if (idx > (unsigned int)m_params.maxTiles)
      return DT_FAILURE | DT_INVALID_PARAM;
   const dtCompressedTile* tile = &m_tiles[idx];
   unsigned int salt = decodeTileIdSalt(ref);
   if (tile->salt != salt)
      return DT_FAILURE | DT_INVALID_PARAM;

   m_talloc->reset();

   NavMeshTileBuildContext bc(m_talloc);
   const int walkableClimbVx = (int)(m_params.walkableClimb / m_params.ch);
   dtStatus status;

   // Decompress tile layer data.
   status = dtDecompressTileCacheLayer(m_talloc, m_tcomp, tile->data, tile->dataSize, &bc.layer);
   if (dtStatusFailed(status))
      return status;

   // Rasterize obstacles.
   for (int i = 0; i < m_params.maxObstacles; ++i)
   {
      const dtTileCacheObstacle* ob = &m_obstacles[i];
      if (ob->state == DT_OBSTACLE_EMPTY || ob->state == DT_OBSTACLE_REMOVING)
         continue;
      if (contains(ob->touched, ob->ntouched, ref))
      {
          if (ob->type == DT_OBSTACLE_CYLINDER)
         {
            dtMarkCylinderArea(*bc.layer, tile->header->bmin, m_params.cs, m_params.ch,
                         ob->cylinder.pos, ob->cylinder.radius, ob->cylinder.height, ob->area_id);
         }
         else if (ob->type == DT_OBSTACLE_BOX)
         {
            //dtMarkBoxArea(*bc.layer, tile->header->bmin, m_params.cs, m_params.ch,
            //   ob->box.bmin, ob->box.bmax, ob->area_id);
            dtMarkBoxArea(*bc.layer, tile->header->bmin, m_params.cs, m_params.ch,
               ob->BoundsMin, ob->BoundsMax, ob->area_id);
         }
         else if (ob->type == DT_OBSTACLE_ORIENTED_BOX)
         {
            dtMarkBoxArea(*bc.layer, tile->header->bmin, m_params.cs, m_params.ch,
               ob->orientedBox.center, ob->orientedBox.halfExtents, ob->orientedBox.rotAux, ob->area_id);
         }
         else if (ob->type == DT_OBSTACLE_CONVEX_POLYGON)
         {
            dtMarkPolyArea ( *bc.layer,
                             tile->header->bmin,
                             m_params.cs,
                             m_params.ch,
                             ob->convexPolygon.verts,
                             ob->convexPolygon.nverts,
                             ob->area_id ) ;
         }
      }
   }

   // Build navmesh
   status = dtBuildTileCacheRegions(m_talloc, *bc.layer, walkableClimbVx);
   if (dtStatusFailed(status))
      return status;

   bc.lcset = dtAllocTileCacheContourSet(m_talloc);
   if (!bc.lcset)
      return DT_FAILURE | DT_OUT_OF_MEMORY;
   status = dtBuildTileCacheContours(m_talloc, *bc.layer, walkableClimbVx,
                             m_params.maxSimplificationError, *bc.lcset);
   if (dtStatusFailed(status))
      return status;

   bc.lmesh = dtAllocTileCachePolyMesh(m_talloc);
   if (!bc.lmesh)
      return DT_FAILURE | DT_OUT_OF_MEMORY;
   status = dtBuildTileCachePolyMesh(m_talloc, *bc.lcset, *bc.lmesh);
   if (dtStatusFailed(status))
      return status;

   // Early out if the mesh tile is empty.
   if (!bc.lmesh->npolys)
   {
      // Remove existing tile.
      navmesh->removeTile(navmesh->getTileRefAt(tile->header->tx,tile->header->ty,tile->header->tlayer),0,0);
      return DT_SUCCESS;
   }

   dtNavMeshCreateParams params;
   memset(&params, 0, sizeof(params));
   params.verts = bc.lmesh->verts;
   params.vertCount = bc.lmesh->nverts;
   params.polys = bc.lmesh->polys;
   params.polyAreas = bc.lmesh->areas;
   params.polyFlags = bc.lmesh->flags;
   params.polyCount = bc.lmesh->npolys;
   params.nvp = DT_VERTS_PER_POLYGON;
   params.walkableHeight = m_params.walkableHeight;
   params.walkableRadius = m_params.walkableRadius;
   params.walkableClimb = m_params.walkableClimb;
   params.tileX = tile->header->tx;
   params.tileY = tile->header->ty;
   params.tileLayer = tile->header->tlayer;
   params.cs = m_params.cs;
   params.ch = m_params.ch;
   params.buildBvTree = false;
   dtVcopy(params.bmin, tile->header->bmin);
   dtVcopy(params.bmax, tile->header->bmax);

   if (m_tmproc)
   {
      m_tmproc->process(&params, bc.lmesh->areas, bc.lmesh->flags);
   }

   unsigned char* navData = 0;
   int navDataSize = 0;
   if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
      return DT_FAILURE;

   // Remove existing tile.
   navmesh->removeTile(navmesh->getTileRefAt(tile->header->tx,tile->header->ty,tile->header->tlayer),0,0);

   // Add new tile, or leave the location empty.
   if (navData)
   {
      // Let the navmesh own the data.
      status = navmesh->addTile(navData,navDataSize,DT_TILE_FREE_DATA,0,0);
      if (dtStatusFailed(status))
      {
         dtFree(navData);
         return status;
      }
   }

   //
   ApplyObstacleFlagsToTile ( *navmesh, *tile, ref ) ;
   //

   return DT_SUCCESS;
}

void dtTileCache::calcTightTileBounds(const dtTileCacheLayerHeader* header, float* bmin, float* bmax) const
{
   const float cs = m_params.cs;
   bmin[0] = header->bmin[0] + header->minx*cs;
   bmin[1] = header->bmin[1];
   bmin[2] = header->bmin[2] + header->miny*cs;
   bmax[0] = header->bmin[0] + (header->maxx+1)*cs;
   bmax[1] = header->bmax[1];
   bmax[2] = header->bmin[2] + (header->maxy+1)*cs;
}

void dtTileCache::getObstacleBounds(const struct dtTileCacheObstacle* ob, float* bmin, float* bmax) const
{
    if (ob->type == DT_OBSTACLE_CYLINDER)
   {
      const dtObstacleCylinder &cl = ob->cylinder;

      bmin[0] = cl.pos[0] - cl.radius;
      bmin[1] = cl.pos[1];
      bmin[2] = cl.pos[2] - cl.radius;
      bmax[0] = cl.pos[0] + cl.radius;
      bmax[1] = cl.pos[1] + cl.height;
      bmax[2] = cl.pos[2] + cl.radius;
   }
   else if (ob->type == DT_OBSTACLE_BOX)
   {
      //dtVcopy(bmin, ob->box.bmin);
      //dtVcopy(bmax, ob->box.bmax);

       dtVcopy(bmin, ob->BoundsMin );
       dtVcopy(bmax, ob->BoundsMax );
   }
   else if (ob->type == DT_OBSTACLE_ORIENTED_BOX)
   {
      const dtObstacleOrientedBox &orientedBox = ob->orientedBox;

      float maxr = 1.41f*dtMax(orientedBox.halfExtents[0], orientedBox.halfExtents[2]);
      bmin[0] = orientedBox.center[0] - maxr;
      bmax[0] = orientedBox.center[0] + maxr;
      bmin[1] = orientedBox.center[1] - orientedBox.halfExtents[1];
      bmax[1] = orientedBox.center[1] + orientedBox.halfExtents[1];
      bmin[2] = orientedBox.center[2] - maxr;
      bmax[2] = orientedBox.center[2] + maxr;
   }
   else if (ob->type == DT_OBSTACLE_CONVEX_POLYGON)
   {
       // Convex hull obstacles
      dtVcopy ( bmin, ob->convexPolygon.verts ) ;
      dtVcopy ( bmax, ob->convexPolygon.verts ) ;

      bmax [ 1 ] = bmin [ 1 ] + ob->convexPolygon.height ;

      for ( int i = 1 ; i < ob->convexPolygon.nverts ; i++ )
      {
         if ( ob->convexPolygon.verts [ i * 3 ] < bmin [ 0 ] )
         {
            bmin [ 0 ] = ob->convexPolygon.verts [ i * 3 ] ;
         }
         else if ( ob->convexPolygon.verts [ i * 3 ] > bmax [ 0 ] )
         {
            bmax [ 0 ] = ob->convexPolygon.verts [ i * 3 ] ;
         }

         if ( ob->convexPolygon.verts [ i * 3 + 2 ] < bmin [ 2 ] )
         {
            bmin [ 2 ] = ob->convexPolygon.verts [ i * 3 + 2 ] ;
         }
         else if ( ob->convexPolygon.verts [ i * 3 + 2 ] > bmax [ 2 ] )
         {
            bmax [ 2 ] = ob->convexPolygon.verts [ i * 3 + 2 ] ;
         }
      }
   }
}
