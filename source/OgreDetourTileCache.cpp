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

#include "OgreDetourTileCache.h"
#include "NavMeshDebug.h"
#include "DetourTileCache.h"
#include <float.h>

// Boost
#include <boost/algorithm/clamp.hpp>

///// Static config parameters //////

// Max number of layers a tile can have
const int   EXPECTED_LAYERS_PER_TILE = 1;

// Extra padding added to the border size of tiles (together with agent radius)
const float BORDER_PADDING = 3;

const int TILECACHESET_MAGIC = 'T'<<24 | 'S'<<16 | 'E'<<8 | 'T'; //'TSET';
const int TILECACHESET_VERSION = 2;

/////////////////////////////////////

OgreDetourTileCache::OgreDetourTileCache(OgreRecast *recast, unsigned int max_num_obstacles, int tileSize)
    : m_recast(recast),
      m_tileSize(tileSize - (tileSize%8)),  // Make sure tilesize is a multiple of 8
      MaxNumObstacles(max_num_obstacles),
      m_keepInterResults(false),
      m_tileCache(0),
      m_cacheBuildTimeMs(0),
      m_cacheCompressedSize(0),
      m_cacheRawSize(0),
      m_cacheLayerCount(0),
      m_cacheBuildMemUsage(0),
      m_maxTiles(0),
      m_maxPolysPerTile(0),
      m_cellSize(0),
      m_tcomp(0),
      m_geom(0),
      m_th(0),
      m_tw(0)
{
    m_ctx = nullptr;

    m_talloc = new LinearAllocator(32000);
    m_tcomp = new FastLZCompressor;
    m_tmproc = new MeshProcess;

    // Sanity check on tilesize
    m_tileSize           = boost::algorithm::clamp ( m_tileSize, 16, 128 ) ;
    TempObstacleAdded    = false ;
    NavMeshDebugInstance = nullptr ;
}

OgreDetourTileCache::
~OgreDetourTileCache()
{
    dtFreeTileCache(m_tileCache);
    delete m_talloc;
    delete m_tcomp;
    delete m_tmproc;
}

bool OgreDetourTileCache::TileCacheBuild(std::vector<Ogre::Entity*> srcMeshes,
                                         const TerrainAreaVector    &area_list )
{
    m_geom = new InputGeom(srcMeshes);

    // Setup the terrain area volumes before the tile cache is built.
    // This will cause all of the areas marked to have the area id specified by AreaId.
    // The AreaId will then be used later to determine the area flags (such as walkability).
    // If this step is done after the tile cache is built then each tile will need to be rebuilt again and iterate over all of the
    // volumes again, taking a lot of time.
    for ( const auto &area : area_list )
    {
       const Ogre::Vector3 half_size = Ogre::Vector3 ( area.Width / 2.0f, 50.0f, area.Depth / 2.0f ) ;
       const Ogre::Vector3 min       = area.Centre - half_size ;
       const Ogre::Vector3 max       = area.Centre + half_size ;

       m_geom->addConvexVolume ( new ConvexVolume ( Ogre::AxisAlignedBox ( min, max ), area.AreaId ) ) ;
    }

    // Init configuration for specified geometry
    configure();

    dtStatus status;

    // Preprocess tiles.
    // Prepares navmesh tiles in a 2D intermediary format that allows quick conversion to a 3D navmesh
    m_cacheLayerCount = 0;
    m_cacheCompressedSize = 0;
    m_cacheRawSize = 0;

    for (int y = 0; y < m_th; ++y)
    {
        for (int x = 0; x < m_tw; ++x)
        {
            TileCacheData tiles[MAX_LAYERS];
            memset(tiles, 0, sizeof(tiles));
            int ntiles = rasterizeTileLayers(x, y, tiles, MAX_LAYERS);  // This is where the tile is built

            for (int i = 0; i < ntiles; ++i)
            {
                TileCacheData* tile = &tiles[i];
                status = m_tileCache->addTile(tile->data, tile->dataSize, DT_COMPRESSEDTILE_FREE_DATA, 0);  // Add compressed tiles to tileCache
                if (dtStatusFailed(status))
                {
                    dtFree(tile->data);
                    tile->data = 0;
                    continue;
                }

                m_cacheLayerCount++;
                m_cacheCompressedSize += tile->dataSize;
                m_cacheRawSize += calcLayerBufferSize(m_tcparams.width, m_tcparams.height);
            }
        }
    }

    // Build initial meshes
    // Builds detour compatible navmesh from all tiles.
    // A tile will have to be rebuilt if something changes, eg. a temporary obstacle is placed on it.
    for (int y = 0; y < m_th; ++y)
        for (int x = 0; x < m_tw; ++x)
            m_tileCache->buildNavMeshTilesAt(x,y, m_recast->m_navMesh); // This immediately builds the tile, without the need of a dtTileCache::update()

    m_cacheBuildMemUsage = m_talloc->high;


    // Count the total size of all generated tiles of the tiled navmesh
    const dtNavMesh* nav = m_recast->m_navMesh;
    int navmeshMemUsage = 0;
    for (int i = 0; i < nav->getMaxTiles(); ++i)
    {
        const dtMeshTile* tile = nav->getTile(i);
        if (tile->header)
            navmeshMemUsage += tile->dataSize;
    }



//    printf("navmeshMemUsage = %.1f kB\n", navmeshMemUsage/1024.0f);
    Ogre::LogManager::getSingletonPtr()->logMessage("Navmesh Mem Usage = "+ Ogre::StringConverter::toString(navmeshMemUsage/1024.0f) +" kB");
    Ogre::LogManager::getSingletonPtr()->logMessage("Tilecache Mem Usage = " +Ogre::StringConverter::toString(m_cacheCompressedSize/1024.0f) +" kB");


    return true;
}

bool OgreDetourTileCache::saveAll(Ogre::String filename)
{
    if (!m_tileCache) {
        Ogre::LogManager::getSingletonPtr()->logMessage("Error: OgreDetourTileCache::saveAll("+filename+"). Could not save tilecache, no tilecache to save.");
        return false;
    }

       FILE* fp = fopen(filename.data(), "wb");
       if (!fp) {
           Ogre::LogManager::getSingletonPtr()->logMessage("Error: OgreDetourTileCache::saveAll("+filename+"). Could not save file.");
           return false;
       }

// Store header.
       TileCacheSetHeader header;
       header.magic = TILECACHESET_MAGIC;
       header.version = TILECACHESET_VERSION;
       header.numTiles = 0;
       for (int i = 0; i < m_tileCache->getTileCount(); ++i)
       {
               const dtCompressedTile* tile = m_tileCache->getTile(i);
               if (!tile || !tile->header || !tile->dataSize) continue;
               header.numTiles++;
       }
       memcpy(&header.cacheParams, m_tileCache->getParams(), sizeof(dtTileCacheParams));
       memcpy(&header.meshParams, m_recast->m_navMesh->getParams(), sizeof(dtNavMeshParams));
       memcpy(&header.recastConfig, &m_cfg, sizeof(rcConfig));
       fwrite(&header, sizeof(TileCacheSetHeader), 1, fp);

       // Store tiles.
       for (int i = 0; i < m_tileCache->getTileCount(); ++i)
       {
               const dtCompressedTile* tile = m_tileCache->getTile(i);
               if (!tile || !tile->header || !tile->dataSize) continue;

               TileCacheTileHeader tileHeader;
               tileHeader.tileRef = m_tileCache->getTileRef(tile);
               tileHeader.dataSize = tile->dataSize;
               fwrite(&tileHeader, sizeof(tileHeader), 1, fp);

               fwrite(tile->data, tile->dataSize, 1, fp);
       }

       fclose(fp);
       return true;
}

bool OgreDetourTileCache::loadAll(Ogre::String filename,
                                  std::vector<Ogre::Entity*> srcMeshes)
{
       FILE* fp = fopen(filename.data(), "rb");
       if (!fp) {
           Ogre::LogManager::getSingletonPtr()->logMessage("Error: OgreDetourTileCache::loadAll("+filename+"). Could not open file.");
           return false;
       }

       // Read header.
       TileCacheSetHeader header;
       fread(&header, sizeof(TileCacheSetHeader), 1, fp);
       if (header.magic != TILECACHESET_MAGIC)
       {
           fclose(fp);
           Ogre::LogManager::getSingletonPtr()->logMessage("Error: OgreDetourTileCache::loadAll("+filename+"). File does not appear to contain valid tilecache data.");
           return false;
       }
       if (header.version != TILECACHESET_VERSION)
       {
           fclose(fp);
           Ogre::LogManager::getSingletonPtr()->logMessage("Error: OgreDetourTileCache::loadAll("+filename+"). File contains a different version of the tilecache data format ("+Ogre::StringConverter::toString(header.version)+" instead of "+Ogre::StringConverter::toString(TILECACHESET_VERSION)+").");
           return false;
       }

       m_recast->m_navMesh = dtAllocNavMesh();
       if (!m_recast->m_navMesh)
       {
           fclose(fp);
           Ogre::LogManager::getSingletonPtr()->logMessage("Error: OgreDetourTileCache::loadAll("+filename+"). Could not allocate navmesh.");
           return false;
       }
       dtStatus status = m_recast->m_navMesh->init(&header.meshParams);
       if (dtStatusFailed(status))
       {
           fclose(fp);
           Ogre::LogManager::getSingletonPtr()->logMessage("Error: OgreDetourTileCache::loadAll("+filename+"). Could not init navmesh.");
           return false;
       }

       m_tileCache = dtAllocTileCache();
       if (!m_tileCache)
       {
           fclose(fp);
           Ogre::LogManager::getSingletonPtr()->logMessage("Error: OgreDetourTileCache::loadAll("+filename+"). Could not allocate tilecache.");
           return false;
       }
       status = m_tileCache->init(&header.cacheParams, m_talloc, m_tcomp, m_tmproc);
       if (dtStatusFailed(status))
       {
           fclose(fp);
           Ogre::LogManager::getSingletonPtr()->logMessage("Error: OgreDetourTileCache::loadAll("+filename+"). Could not init tilecache.");
           return false;
       }

       memcpy(&m_cfg, &header.recastConfig, sizeof(rcConfig));

       // Read tiles.
       for (int i = 0; i < header.numTiles; ++i)
       {
               TileCacheTileHeader tileHeader;
               fread(&tileHeader, sizeof(tileHeader), 1, fp);
               if (!tileHeader.tileRef || !tileHeader.dataSize)
                       break;

               unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
               if (!data) break;
               memset(data, 0, tileHeader.dataSize);
               fread(data, tileHeader.dataSize, 1, fp);

               dtCompressedTileRef tile = 0;
               m_tileCache->addTile(data, tileHeader.dataSize, DT_COMPRESSEDTILE_FREE_DATA, &tile);

               if (tile)
                       m_tileCache->buildNavMeshTile(tile, m_recast->m_navMesh);
       }

       fclose(fp);


       // Init recast navmeshquery with created navmesh (in OgreRecast component)
       m_recast->m_navQuery = dtAllocNavMeshQuery();
       m_recast->m_navQuery->init(m_recast->m_navMesh, 2048);


       // Config
       // TODO handle this nicer, also inputGeom is not inited, making some functions crash
       m_cellSize = m_cfg.cs;
       m_tileSize = m_cfg.tileSize;

       // cache bounding box
       const float* bmin = m_cfg.bmin;
       const float* bmax = m_cfg.bmax;

       // Copy loaded config back to recast module
       memcpy(&m_recast->m_cfg, &m_cfg, sizeof(rcConfig));

       m_tileSize = m_cfg.tileSize;
       m_cellSize = m_cfg.cs;
       m_tcparams = header.cacheParams;

       // Determine grid size (number of tiles) based on bounding box and grid cell size
       int gw = 0, gh = 0;
       rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);   // Calculates total size of voxel grid
       const int ts = m_tileSize;
       const int tw = (gw + ts-1) / ts;    // Tile width
       const int th = (gh + ts-1) / ts;    // Tile height
       m_tw = tw;
       m_th = th;


       Ogre::LogManager::getSingletonPtr()->logMessage("Total Voxels: "+Ogre::StringConverter::toString(gw) + " x " + Ogre::StringConverter::toString(gh));
       Ogre::LogManager::getSingletonPtr()->logMessage("Tilesize: "+Ogre::StringConverter::toString(m_tileSize)+"  Cellsize: "+Ogre::StringConverter::toString(m_cellSize));
       Ogre::LogManager::getSingletonPtr()->logMessage("Tiles: "+Ogre::StringConverter::toString(m_tw)+" x "+Ogre::StringConverter::toString(m_th));


       // Max tiles and max polys affect how the tile IDs are caculated.
       // There are 22 bits available for identifying a tile and a polygon.
       int tileBits = rcMin((int)dtIlog2(dtNextPow2(tw*th*EXPECTED_LAYERS_PER_TILE)), 14);
       if (tileBits > 14) tileBits = 14;
       int polyBits = 22 - tileBits;
       m_maxTiles = 1 << tileBits;
       m_maxPolysPerTile = 1 << polyBits;
       Ogre::LogManager::getSingletonPtr()->logMessage("Max Tiles: " + Ogre::StringConverter::toString(m_maxTiles));
       Ogre::LogManager::getSingletonPtr()->logMessage("Max Polys: " + Ogre::StringConverter::toString(m_maxPolysPerTile));
       // End config ////





       // Build initial meshes
       // Builds detour compatible navmesh from all tiles.
       // A tile will have to be rebuilt if something changes, eg. a temporary obstacle is placed on it.
       for (int y = 0; y < m_th; ++y)
           for (int x = 0; x < m_tw; ++x)
           {
               m_tileCache->buildNavMeshTilesAt(x,y, m_recast->m_navMesh); // This immediately builds the tile, without the need of a dtTileCache::update()

               //drawDetail(x, y);
           }

   //    m_cacheBuildTimeMs = ctx->getAccumulatedTime(RC_TIMER_TOTAL)/1000.0f;
       m_cacheBuildMemUsage = m_talloc->high;


       // Count the total size of all generated tiles of the tiled navmesh
       const dtNavMesh* nav = m_recast->m_navMesh;
       int navmeshMemUsage = 0;
       for (int i = 0; i < nav->getMaxTiles(); ++i)
       {
           const dtMeshTile* tile = nav->getTile(i);
           if (tile->header)
               navmeshMemUsage += tile->dataSize;
       }


       Ogre::LogManager::getSingletonPtr()->logMessage("Navmesh Mem Usage = "+ Ogre::StringConverter::toString(navmeshMemUsage/1024.0f) +" kB");
       Ogre::LogManager::getSingletonPtr()->logMessage("Tilecache Mem Usage = " +Ogre::StringConverter::toString(m_cacheCompressedSize/1024.0f) +" kB");

       // Set member objects ready which would usually be done if the tile cache was built from scratch
       {
         assert ( !m_geom && !m_ctx) ;

         m_geom = new InputGeom ( std::move ( srcMeshes ) ) ;
         m_ctx  = m_recast->m_ctx ;
       }

       return true;
}

void
OgreDetourTileCache::
handleUpdate ( const float dt,
               const bool  until_up_to_date ) // Continue processing the tile cache obstacles until the entire navmesh is up-to-date
{
   if ( ! m_recast->m_navMesh )
   {
      return ;
   }

   if ( ! m_tileCache )
   {
      return ;
   }

   if ( ! until_up_to_date )
   {
      m_tileCache->update ( dt, m_recast->m_navMesh ) ;
   }
   else
   {
      bool up_to_date = false ;

      while ( ! up_to_date )
      {
         m_tileCache->update ( dt, m_recast->m_navMesh, &up_to_date ) ;
      }
   }

   if ( TempObstacleAdded &&
        NavMeshDebugInstance )
   {
      for ( int obstacle_index = 0 ; obstacle_index < m_tileCache->getObstacleCount () ; ++obstacle_index )
      {
         const dtTileCacheObstacle *obstacle = m_tileCache->getObstacle ( obstacle_index ) ;

         for ( int tile_index = 0 ; tile_index < obstacle->ntouched ; ++tile_index )
         {
            const dtCompressedTile *tile = m_tileCache->getTileByRef ( obstacle->touched [ tile_index ] ) ;

            if ( tile )
            {
               NavMeshDebugInstance->RedrawTile ( tile->header->tx, tile->header->ty ) ;
            }
         }
      }

      TempObstacleAdded = false ;
   }
}

dtObstacleRef
OgreDetourTileCache::
AddObstacle ( const Ogre::Vector3  &min,
              const Ogre::Vector3  &max,
              const unsigned char  area_id,
              const unsigned short flags )
{
   dtObstacleRef result = 0 ;

   if ( m_tileCache )
   {
      float bmin [ 3 ] ;
      float bmax [ 3 ] ;
      OgreRecast::OgreVect3ToFloatA ( min, bmin ) ;
      OgreRecast::OgreVect3ToFloatA ( max, bmax ) ;

      if ( m_tileCache->addBoxObstacle ( bmin, bmax, &result, area_id, flags ) == DT_SUCCESS ) // No rotation
      {
         TempObstacleAdded = true ;
      }
   }

   return result ;
}

dtObstacleRef
OgreDetourTileCache::
AddObstacle ( const Ogre::Vector3  &centre,
              const float          width,
              const float          depth,
              const float          height,
              const float          y_rotation, // radians
              const unsigned char  area_id,
              const unsigned short flags )
{
   dtObstacleRef result = 0 ;

   if ( m_tileCache )
   {
      float centre_position [ 3 ] ;
      float half_extents [ 3 ] ;
      OgreRecast::OgreVect3ToFloatA ( centre, centre_position ) ;
      OgreRecast::OgreVect3ToFloatA ( Ogre::Vector3 ( width, height, depth ) / 2.0f, half_extents ) ;

      if ( m_tileCache->addBoxObstacle ( centre_position, half_extents, y_rotation, &result, area_id, flags ) == DT_SUCCESS )
      {
         TempObstacleAdded = true ;
      }
   }

   return result ;
}

const dtTileCacheObstacle *
OgreDetourTileCache::
getObstacleByRef ( dtObstacleRef ref )
{
   return m_tileCache->getObstacleByRef ( ref ) ;
}

bool OgreDetourTileCache::RemoveObstacle(dtObstacleRef obstacleRef)
{
    if(m_tileCache->removeObstacle(obstacleRef) == DT_SUCCESS)
        return true;
    else
        return false;
}

bool OgreDetourTileCache::configure()
{
    // Reuse OgreRecast context for tiled navmesh building
    m_ctx = m_recast->m_ctx;

    if (!m_geom) {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: OgreDetourTileCache::configure: No vertices and triangles.");
        return false;
    }

    if (!m_geom->getChunkyMesh()) {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: OgreDetourTileCache::configure: Input mesh has no chunkyTriMesh built.");
        return false;
    }

    m_tmproc->init(m_geom);


    // Init cache bounding box
    const float* bmin = m_geom->getMeshBoundsMin();
    const float* bmax = m_geom->getMeshBoundsMax();

    // Navmesh generation params.
    // Use config from recast module
    m_cfg = m_recast->m_cfg;

    // Most params are taken from OgreRecast::configure, except for these:
    m_cfg.tileSize = m_tileSize;
    m_cfg.borderSize = (int) (m_cfg.walkableRadius + BORDER_PADDING); // Reserve enough padding.
    m_cfg.width = m_cfg.tileSize + m_cfg.borderSize*2;
    m_cfg.height = m_cfg.tileSize + m_cfg.borderSize*2;

    // Set mesh bounds
    rcVcopy(m_cfg.bmin, bmin);
    rcVcopy(m_cfg.bmax, bmax);
    // Also define navmesh bounds in recast component
    rcVcopy(m_recast->m_cfg.bmin, bmin);
    rcVcopy(m_recast->m_cfg.bmax, bmax);

    // Cell size navmesh generation property is copied from OgreRecast config
    m_cellSize = m_cfg.cs;

    // Determine grid size (number of tiles) based on bounding box and grid cell size
    int gw = 0, gh = 0;
    rcCalcGridSize(bmin, bmax, m_cellSize, &gw, &gh);   // Calculates total size of voxel grid
    const int ts = m_tileSize;
    const int tw = (gw + ts-1) / ts;    // Tile width
    const int th = (gh + ts-1) / ts;    // Tile height
    m_tw = tw;
    m_th = th;


    // Max tiles and max polys affect how the tile IDs are caculated.
    // There are 22 bits available for identifying a tile and a polygon.
    int tileBits = rcMin((int)dtIlog2(dtNextPow2(tw*th*EXPECTED_LAYERS_PER_TILE)), 14);
    if (tileBits > 14) tileBits = 14;
    int polyBits = 22 - tileBits;
    m_maxTiles = 1 << tileBits;
    m_maxPolysPerTile = 1 << polyBits;


    // Tile cache params.
    memset(&m_tcparams, 0, sizeof(m_tcparams));
    rcVcopy(m_tcparams.orig, bmin);
    m_tcparams.width = m_tileSize;
    m_tcparams.height = m_tileSize;
    m_tcparams.maxTiles = tw*th*EXPECTED_LAYERS_PER_TILE;
    m_tcparams.maxObstacles = MaxNumObstacles;    // Max number of temp obstacles that can be added to or removed from navmesh

    // Copy the rest of the parameters from OgreRecast config
    m_tcparams.cs = m_cfg.cs;
    m_tcparams.ch = m_cfg.ch;
    m_tcparams.walkableHeight = (float) m_cfg.walkableHeight;
    m_tcparams.walkableRadius = (float) m_cfg.walkableRadius;
    m_tcparams.walkableClimb = (float) m_cfg.walkableClimb;
    m_tcparams.maxSimplificationError = m_cfg.maxSimplificationError;

    return initTileCache();
}

int OgreDetourTileCache::rasterizeTileLayers(const int tx, const int ty, TileCacheData* tiles, const int maxTiles)
{
    if (!m_geom) {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildTile: Input mesh is not specified.");
        return 0;
    }

    if (!m_geom->getChunkyMesh()) {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildTile: Input mesh has no chunkyTriMesh built.");
        return 0;
    }

//TODO make these member variables?
    FastLZCompressor comp;
    RasterizationContext rc;

    const float* verts = m_geom->getVerts();
    const int nverts = m_geom->getVertCount();

    // The chunky tri mesh in the inputgeom is a simple spatial subdivision structure that allows to
    // process the vertices in the geometry relevant to this part of the tile.
    // The chunky tri mesh is a grid of axis aligned boxes that store indices to the vertices in verts
    // that are positioned in that box.
    const rcChunkyTriMesh* chunkyMesh = m_geom->getChunkyMesh();

    // Tile bounds.
    const float tcs = m_tileSize * m_cellSize;

    rcConfig tcfg;
    memcpy(&tcfg, &m_cfg, sizeof(tcfg));

    tcfg.bmin[0] = m_cfg.bmin[0] + tx*tcs;
    tcfg.bmin[1] = m_cfg.bmin[1];
    tcfg.bmin[2] = m_cfg.bmin[2] + ty*tcs;
    tcfg.bmax[0] = m_cfg.bmin[0] + (tx+1)*tcs;
    tcfg.bmax[1] = m_cfg.bmax[1];
    tcfg.bmax[2] = m_cfg.bmin[2] + (ty+1)*tcs;
    tcfg.bmin[0] -= tcfg.borderSize*tcfg.cs;
    tcfg.bmin[2] -= tcfg.borderSize*tcfg.cs;
    tcfg.bmax[0] += tcfg.borderSize*tcfg.cs;
    tcfg.bmax[2] += tcfg.borderSize*tcfg.cs;


    // This is part of the regular recast navmesh generation pipeline as in OgreRecast::NavMeshBuild()
    // but only up till step 4 and slightly modified.


    // Allocate voxel heightfield where we rasterize our input data to.
    rc.solid = rcAllocHeightfield();
    if (!rc.solid)
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildNavigation: Out of memory 'solid'.");
        return 0;
    }
    if (!rcCreateHeightfield(m_ctx, *rc.solid, tcfg.width, tcfg.height, tcfg.bmin, tcfg.bmax, tcfg.cs, tcfg.ch))
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildNavigation: Could not create solid heightfield.");
        return 0;
    }

    // Allocate array that can hold triangle flags.
    // If you have multiple meshes you need to process, allocate
    // an array which can hold the max number of triangles you need to process.
    rc.triareas = new unsigned char[chunkyMesh->maxTrisPerChunk];
    if (!rc.triareas)
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildNavigation: Out of memory 'm_triareas' ("+Ogre::StringConverter::toString(chunkyMesh->maxTrisPerChunk)+").");
        return 0;
    }

    float tbmin[2], tbmax[2];
    tbmin[0] = tcfg.bmin[0];
    tbmin[1] = tcfg.bmin[2];
    tbmax[0] = tcfg.bmax[0];
    tbmax[1] = tcfg.bmax[2];
    int cid[512];// TODO: Make grow when returning too many items.
    const int ncid = rcGetChunksOverlappingRect(chunkyMesh, tbmin, tbmax, cid, 512);
    if (!ncid)
    {
        return 0; // empty
    }

    for (int i = 0; i < ncid; ++i)
    {
        const rcChunkyTriMeshNode& node = chunkyMesh->nodes[cid[i]];
        const int* tris = &chunkyMesh->tris[node.i*3];
        const int ntris = node.n;

        memset(rc.triareas, 0, ntris*sizeof(unsigned char));
        rcMarkWalkableTriangles(m_ctx, tcfg.walkableSlopeAngle,
                                verts, nverts, tris, ntris, rc.triareas);

        rcRasterizeTriangles(m_ctx, verts, nverts, tris, rc.triareas, ntris, *rc.solid, tcfg.walkableClimb);
    }

    // Once all geometry is rasterized, we do initial pass of filtering to
    // remove unwanted overhangs caused by the conservative rasterization
    // as well as filter spans where the character cannot possibly stand.
    rcFilterLowHangingWalkableObstacles(m_ctx, tcfg.walkableClimb, *rc.solid);
    rcFilterLedgeSpans(m_ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid);
    rcFilterWalkableLowHeightSpans(m_ctx, tcfg.walkableHeight, *rc.solid);


    rc.chf = rcAllocCompactHeightfield();
    if (!rc.chf)
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildNavigation: Out of memory 'chf'.");
        return 0;
    }
    if (!rcBuildCompactHeightfield(m_ctx, tcfg.walkableHeight, tcfg.walkableClimb, *rc.solid, *rc.chf))
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildNavigation: Could not build compact data.");
        return 0;
    }

    // Erode the walkable area by agent radius.
    if (!rcErodeWalkableArea(m_ctx, tcfg.walkableRadius, *rc.chf))
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildNavigation: Could not erode.");
        return 0;
    }

    // Mark areas of dynamically added convex polygons
    const ConvexVolume* const* vols = m_geom->getConvexVolumes();
    for (int i  = 0; i < m_geom->getConvexVolumeCount(); ++i)
    {
       // TODO: Check if this is actually used, i.e. are there ever any convex volumes at this point?
       //       This causes the recast height map to be marked instead of the tile cache which would be done using dtMark...
       //       This may only affect the 'standard' navigation mesh, i.e. not used for a tiled navigation mesh.
        rcMarkConvexPolyArea(m_ctx, vols[i]->verts, vols[i]->nverts,
                             vols[i]->hmin, vols[i]->hmax,
                             (unsigned char)vols[i]->area, *rc.chf);
    }



    // Up till this part was more or less the same as OgreRecast::NavMeshBuild()
    // The following part is specific for creating a 2D intermediary navmesh tile.

    rc.lset = rcAllocHeightfieldLayerSet();
    if (!rc.lset)
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildNavigation: Out of memory 'lset'.");
        return 0;
    }
    if (!rcBuildHeightfieldLayers(m_ctx, *rc.chf, tcfg.borderSize, tcfg.walkableHeight, *rc.lset))
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildNavigation: Could not build heightfield layers.");
        return 0;
    }

    rc.ntiles = 0;
    for (int i = 0; i < rcMin(rc.lset->nlayers, MAX_LAYERS); ++i)
    {
        TileCacheData* tile = &rc.tiles[rc.ntiles++];
        const rcHeightfieldLayer* layer = &rc.lset->layers[i];

        // Store header
        dtTileCacheLayerHeader header;
        header.magic = DT_TILECACHE_MAGIC;
        header.version = DT_TILECACHE_VERSION;

        // Tile layer location in the navmesh.
        header.tx = tx;
        header.ty = ty;
        header.tlayer = i;
        dtVcopy(header.bmin, layer->bmin);
        dtVcopy(header.bmax, layer->bmax);

        // Tile info.
        header.width = (unsigned char)layer->width;
        header.height = (unsigned char)layer->height;
        header.minx = (unsigned char)layer->minx;
        header.maxx = (unsigned char)layer->maxx;
        header.miny = (unsigned char)layer->miny;
        header.maxy = (unsigned char)layer->maxy;
        header.hmin = (unsigned short)layer->hmin;
        header.hmax = (unsigned short)layer->hmax;

        dtStatus status = dtBuildTileCacheLayer(&comp, &header, layer->heights, layer->areas, layer->cons,
                                                &tile->data, &tile->dataSize);
        if (dtStatusFailed(status))
        {
            return 0;
        }
    }

    // Transfer ownsership of tile data from build context to the caller.
    int n = 0;
    for (int i = 0; i < rcMin(rc.ntiles, maxTiles); ++i)
    {
        tiles[n++] = rc.tiles[i];
        rc.tiles[i].data = 0;
        rc.tiles[i].dataSize = 0;
    }

    return n;
}

bool OgreDetourTileCache::initTileCache()
{
    // BUILD TileCache
    dtFreeTileCache(m_tileCache);

    dtStatus status;

    m_tileCache = dtAllocTileCache();
    if (!m_tileCache)
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildTiledNavigation: Could not allocate tile cache.");
        return false;
    }
    status = m_tileCache->init(&m_tcparams, m_talloc, m_tcomp, m_tmproc);
    if (dtStatusFailed(status))
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildTiledNavigation: Could not init tile cache.");
        return false;
    }

    dtFreeNavMesh(m_recast->m_navMesh);

    m_recast->m_navMesh = dtAllocNavMesh();
    if (!m_recast->m_navMesh)
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildTiledNavigation: Could not allocate navmesh.");
        return false;
    }


    // Init multi-tile navmesh parameters
    dtNavMeshParams params;
    memset(&params, 0, sizeof(params));
    rcVcopy(params.orig, m_tcparams.orig);   // Set world-space origin of tile grid
    params.tileWidth = m_tileSize*m_tcparams.cs;
    params.tileHeight = m_tileSize*m_tcparams.cs;
    params.maxTiles = m_maxTiles;
    params.maxPolys = m_maxPolysPerTile;

    status = m_recast->m_navMesh->init(&params);
    if (dtStatusFailed(status))
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildTiledNavigation: Could not init navmesh.");
        return false;
    }

    // Init recast navmeshquery with created navmesh (in OgreRecast component)
    m_recast->m_navQuery = dtAllocNavMeshQuery();
    status = m_recast->m_navQuery->init(m_recast->m_navMesh, 2048);
    if (dtStatusFailed(status))
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildTiledNavigation: Could not init Detour navmesh query");
        return false;
    }

    return true;
}
