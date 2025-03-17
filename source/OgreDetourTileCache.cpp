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
#include "OgreRecast.h"

/*
setOffMeshConnections(offMeshConnections: OffMeshConnectionParams[]): void {
    if (offMeshConnections.length <= 0) return;

    const verts: number[] = [];
    const rads: number[] = [];
    const dir: number[] = [];
    const areas: number[] = [];
    const flags: number[] = [];
    const userIds: number[] = [];

    for (let i = 0; i < offMeshConnections.length; i++) {
      const connection = offMeshConnections[i];

      verts.push(
        connection.startPosition.x,
        connection.startPosition.y,
        connection.startPosition.z
      );
      verts.push(
        connection.endPosition.x,
        connection.endPosition.y,
        connection.endPosition.z
      );

      rads.push(connection.radius);
      dir.push(connection.bidirectional ? 1 : 0);
      areas.push(connection.area ?? 0);
      flags.push(connection.flags ?? 1);
      userIds.push(connection.userId ?? 1000 + i);
    }

    Raw.DetourNavMeshBuilder.setOffMeshConnections(
      this.raw,
      offMeshConnections.length,
      verts,
      rads,
      dir,
      areas,
      flags,
      userIds
    );
  }
*/
void SetOffMeshConnections ( dtNavMeshCreateParams* navMeshCreateParams,
   int offMeshConCount,
   float* offMeshConVerts,
   float* offMeshConRad,
   unsigned char* offMeshConDirs,
   unsigned char* offMeshConAreas,
   unsigned short* offMeshConFlags,
   unsigned int* offMeshConUserId )
{
    int n = offMeshConCount;

    float *verts = new float[n * 3 * 2];
    memcpy(verts, offMeshConVerts, sizeof(float) * offMeshConCount * 3 * 2);

    float *rads = new float[n];
    memcpy(rads, offMeshConRad, sizeof(float) * n);

    unsigned char *dirs = new unsigned char[n];
    memcpy(dirs, offMeshConDirs, sizeof(unsigned char) * n);

    unsigned char *areas = new unsigned char[n];
    memcpy(areas, offMeshConAreas, sizeof(unsigned char) * n);

    unsigned short *flags = new unsigned short[n];
    memcpy(flags, offMeshConFlags, sizeof(unsigned short) * n);

    unsigned int *userIds = new unsigned int[n];
    memcpy(userIds, offMeshConUserId, sizeof(unsigned int) * n);

    navMeshCreateParams->offMeshConCount = offMeshConCount;
    navMeshCreateParams->offMeshConVerts = verts;
    navMeshCreateParams->offMeshConRad = rads;
    navMeshCreateParams->offMeshConDir = dirs;
    navMeshCreateParams->offMeshConAreas = areas;
    navMeshCreateParams->offMeshConFlags = flags;
    navMeshCreateParams->offMeshConUserID = userIds;
}

void
MeshProcess::
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
           poly_flags [ poly_index ] |= POLYFLAGS_WALK ;

           // All polygons by default allow all players
           poly_flags [ poly_index ] |= POLYFLAGS_ALL_PLAYERS ;

           // GATE obstacles apply flags to the navmesh polygons to indicate which players can walk through them.
           // This is done when the tile is re-build, the heightmap is marked with the POLYAREA_GATE area type.
           // We then appy the specific flags in SetObstacleFlags after the tile build is done.
           //
           // We cannot apply the specific flags here as we only have access to the heightmap, not the actual navmesh polygons which haven't been built yet.
        }
    }

    /*
    params->setOffMeshConnections([
      {
        startPosition: { x: 0, y: 5, z: 0 },
        endPosition: { x: 2, y: 0, z: 0 },
        radius: 0.5,
        bidirectional: false,
        area: 0,
        flags: 1,
      },
    ]);
    */

    /*
    /// Off-mesh connection vertices. [(ax, ay, az, bx, by, bz) * #offMeshConCount] [Unit: wu]
	const float* offMeshConVerts;
	/// Off-mesh connection radii. [Size: #offMeshConCount] [Unit: wu]
	const float* offMeshConRad;
	/// User defined flags assigned to the off-mesh connections. [Size: #offMeshConCount]
	const unsigned short* offMeshConFlags;
	/// User defined area ids assigned to the off-mesh connections. [Size: #offMeshConCount]
	const unsigned char* offMeshConAreas;
	/// The permitted travel direction of the off-mesh connections. [Size: #offMeshConCount]
	///
	/// 0 = Travel only from endpoint A to endpoint B.<br/>
	/// #DT_OFFMESH_CON_BIDIR = Bidirectional travel.
	const unsigned char* offMeshConDir;	
	/// The user defined ids of the off-mesh connection. [Size: #offMeshConCount]
	const unsigned int* offMeshConUserID;
	/// The number of off-mesh connections. [Limit: >= 0]
	int offMeshConCount;
   */

    /*
    auto verts = std::vector <float> ();
    auto rads = std::vector <float> ();
    auto dir = std::vector <unsigned char> ();
    auto areas = std::vector <unsigned char> ();
    auto flags = std::vector <unsigned short> ();
    auto userIds = std::vector <unsigned int> ();

    {
       verts.push_back (0.0f);
       verts.push_back (0.0f);
       verts.push_back (0.0f);

       verts.push_back (10.0f);
       verts.push_back (10.0f);
       verts.push_back (10.0f);

       rads.push_back ( 10.0f );
       dir.push_back ( true );
       areas.push_back ( POLYAREA_GRASS );
       flags.push_back ( POLYFLAGS_WALK );
       userIds.push_back ( userIds.size () );
    }

    SetOffMeshConnections(
       params,
       1,
       verts.data(),
       rads.data (),
       dir.data (),
       areas.data (),
       flags.data (),
       userIds.data ()
    );
    */
}

// Extra padding added to the border size of tiles (together with agent radius)
const float BORDER_PADDING = 3 ;

const int TILECACHESET_MAGIC   = 'T'<<24 | 'S'<<16 | 'E'<<8 | 'T' ; //'TSET';
const int TILECACHESET_VERSION = 2 ;

template <typename StructType>
void
StructToBytes ( std::vector <std::uint8_t> &bytes,
                const StructType           &source )
{
   const auto* to_copy = reinterpret_cast <const std::uint8_t*> ( &source ) ;

   bytes.insert ( bytes.end (), to_copy, to_copy + sizeof ( StructType ) ) ;
}

template <typename DataType>
void
DataToBytes ( std::vector <std::uint8_t> &bytes,
              const DataType*            source,
              const std::size_t          data_size )
{
   const auto* to_copy = reinterpret_cast <const std::uint8_t*> ( source ) ;

   bytes.insert ( bytes.end (), to_copy, to_copy + data_size ) ;
}

template <typename StructType>
void
BytesToStruct ( std::vector <std::uint8_t>::const_iterator       &current_iter,
                const std::vector <std::uint8_t>::const_iterator end_iter,
                StructType                                       &target )
{
   const auto remaining_space = std::distance ( current_iter, end_iter ) ;

   if ( static_cast <std::size_t> ( remaining_space ) >= sizeof ( StructType ) )
   {
      memcpy ( &target, &( *current_iter ), sizeof ( StructType ) ) ;

      current_iter += sizeof ( StructType ) ;
   }
   else
   {
      throw std::runtime_error ( "" ) ;
   }
}

template <typename DataType>
void
BytesToData  ( std::vector <std::uint8_t>::const_iterator       &current_iter,
               const std::vector <std::uint8_t>::const_iterator end_iter,
               DataType*                                        target,
               const std::size_t                                data_size )
{
   const auto remaining_space = std::distance ( current_iter, end_iter ) ;

   if ( static_cast <std::size_t> ( remaining_space ) >= data_size )
   {
      memcpy ( target, &( *current_iter ), data_size ) ;

      current_iter += data_size ;
   }
   else
   {
      throw std::runtime_error ( "" ) ;
   }
}

// Increase this to allow for longer path searches
const auto MAX_SEARCH_NODES = 2048 * 4 ;

OgreDetourTileCache::
OgreDetourTileCache ( OgreRecast         &recast,
                      rcContext          &context,
                      rcConfig           &config,
                      dtNavMeshQuery     &nav_query,
                      const unsigned int max_num_obstacles,
                      const int          tile_size,
                      const bool         keep_heightfield ) :
   Recast                ( recast ),
   m_tileSize            ( tile_size - ( tile_size % 8 ) ),  // Make sure tilesize is a multiple of 8
   MaxNumObstacles       ( max_num_obstacles ),
   m_tileCache           ( nullptr ),
   m_maxTiles            ( 0 ),
   m_maxPolysPerTile     ( 0 ),
   m_cellSize            ( 0 ),
   m_tcomp               ( nullptr ),
   InputGeometry         ( nullptr ),
   m_th                  ( 0 ),
   m_tw                  ( 0 ),
   //m_volumeCount         ( 0 ),
   m_ctx                 ( context ),
   m_cfg                 ( config ),
   NavQuery              ( nav_query ),
   m_keep_heightfield    ( keep_heightfield )
{
    m_talloc  = new LinearAllocator ( 32000 ) ;
    m_tcomp   = new FastLZCompressor ;
    m_tmproc  = new MeshProcess ;
    m_navMesh = nullptr ;

    // Sanity check on tilesize
    m_tileSize = std::clamp ( m_tileSize, 16, 128 ) ;
}

OgreDetourTileCache::
~OgreDetourTileCache ()
{
   dtFreeNavMesh ( m_navMesh ) ;
   dtFreeTileCache ( m_tileCache ) ;
   delete m_talloc ;
   delete m_tcomp ;
   delete m_tmproc ;
   delete InputGeometry ;
}

std::unique_ptr <NavMeshDebug>
OgreDetourTileCache::
CreateDebugger ()
{
   return std::make_unique <NavMeshDebug> ( Recast, *m_tileCache, *m_navMesh, NavQuery ) ;
}

const InputGeom*
OgreDetourTileCache::
GetInputGeometry () const
{
   return InputGeometry ;
}

const std::vector <rcHeightfield*>
OgreDetourTileCache::
GetHeightField () const
{
   return m_solid ;
}

bool
OgreDetourTileCache::
TileCacheBuild ( std::vector<Ogre::ManualObject*> srcMeshes,
                 const TerrainAreaVector    &area_list )
{
   InputGeometry = new InputGeom ( std::move ( srcMeshes ) ) ;

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

      // This is limited by the number of convex volumes allowed, we must tile march to expand them and reduce the count
      AddConvexVolume ( std::make_unique <ConvexVolume> ( Ogre::AxisAlignedBox ( min, max ), area.AreaId ) ) ;
   }

   // Init configuration for specified geometry
   ConfigureTileCacheContext () ;

   dtStatus status ;

   // Preprocess tiles.
   // Prepares navmesh tiles in a 2D intermediary format that allows quick conversion to a 3D navmesh
   for ( int y = 0 ; y < m_th ; ++y )
   {
      for ( int x = 0 ; x < m_tw ; ++x )
      {
         TileCacheData tiles [ MAX_LAYERS ] ;

         memset ( tiles, 0, sizeof ( tiles ) ) ;

         int ntiles = RasterizeTileLayers ( x, y, tiles, MAX_LAYERS ) ; // This is where the tile is built

         for ( int i = 0 ; i < ntiles ; ++i )
         {
            TileCacheData *tile = &tiles [ i ] ;

            status = m_tileCache->addTile ( tile->data, tile->dataSize, DT_COMPRESSEDTILE_FREE_DATA, 0 ) ; // Add compressed tiles to tileCache

            if ( dtStatusFailed ( status ) )
            {
               dtFree ( tile->data ) ;
               tile->data = nullptr ;
               continue ;
            }
         }
      }
   }

   // Build initial meshes
   // Builds detour compatible navmesh from all tiles.
   // A tile will have to be rebuilt if something changes, eg. a temporary obstacle is placed on it.
   for ( int y = 0 ; y < m_th ; ++y )
   {
      for ( int x = 0 ; x < m_tw ; ++x )
      {
         m_tileCache->buildNavMeshTilesAt ( x, y, m_navMesh ) ; // This immediately builds the tile, without the need of a dtTileCache::update()
      }
   }

   return true ;
}

bool
OgreDetourTileCache::
SaveAll ( const Ogre::String &filename )
{
    if ( ! m_tileCache )
    {
        Ogre::LogManager::getSingleton ().logMessage ( "Error: OgreDetourTileCache::saveAll(" + filename + "). Could not save tilecache, no tilecache to save." ) ;
        return false ;
    }

    std::ofstream file ( filename, std::ios::out | std::ios::binary ) ;

   if ( ! file )
   {
      Ogre::LogManager::getSingleton ().logMessage ( "Error: OgreDetourTileCache::saveAll(" + filename + "). Could not save file." ) ;
      return false ;
   }

   const auto bytes = ToBytes () ;

   if ( bytes.empty () )
   {
      Ogre::LogManager::getSingleton ().logMessage ( "Error: OgreDetourTileCache::saveAll(" + filename + "). Could not convert to raw data." ) ;
      return false ;
   }

   file.write ( reinterpret_cast <const char*> ( bytes.data () ), bytes.size () ) ;

   return file.good () ;
}

bool
OgreDetourTileCache::
LoadAll ( const Ogre::String &filename )
{
   std::ifstream file ( filename, std::ios::in | std::ios::binary ) ;

   if ( ! file )
   {
      Ogre::LogManager::getSingletonPtr()->logMessage ( "Error: OgreDetourTileCache::loadAll("+filename+"). Could not open file." ) ;
      return false ;
   }

   std::vector <std::uint8_t> bytes = { std::istream_iterator <std::uint8_t> ( file ),
                                        std::istream_iterator <std::uint8_t> () } ;

   file.close () ;

   const bool bytes_read = FromBytes ( std::move ( bytes ) ) ;

   return bytes_read ;
}

std::vector <std::uint8_t>
OgreDetourTileCache::
ToBytes ()
{
   if ( ! m_tileCache )
   {
      return {} ;
   }

  // Store header.
  TileCacheSetHeader header ;
  header.magic = TILECACHESET_MAGIC;
  header.version = TILECACHESET_VERSION;
  header.numTiles = 0;

  for (int i = 0; i < m_tileCache->getTileCount(); ++i)
  {
     const dtCompressedTile* tile = m_tileCache->getTile(i);
     if (!tile || !tile->header || !tile->dataSize) continue;
     header.numTiles++;
  }

  memcpy ( &header.cacheParams, m_tileCache->getParams(), sizeof(dtTileCacheParams));
  memcpy ( &header.meshParams, m_navMesh->getParams(), sizeof(dtNavMeshParams));
  memcpy ( &header.recastConfig, &m_cfg, sizeof(rcConfig));

  std::vector <std::uint8_t> bytes ;

  StructToBytes ( bytes, header ) ;

  // Store tiles.
  for (int i = 0; i < m_tileCache->getTileCount(); ++i)
  {
     const dtCompressedTile* tile = m_tileCache->getTile(i);
     if (!tile || !tile->header || !tile->dataSize) continue;

     TileCacheTileHeader tileHeader;
     tileHeader.tileRef = m_tileCache->getTileRef(tile);
     tileHeader.dataSize = tile->dataSize;

     StructToBytes ( bytes, tileHeader ) ;

     DataToBytes ( bytes, tile->data, tile->dataSize ) ;
  }

  return bytes ;
}

bool
OgreDetourTileCache::
FromBytes ( const std::vector <std::uint8_t> &bytes )
{
   if ( bytes.empty () )
   {
      return false ;
   }

   auto bytes_iter = bytes.cbegin () ;

   // Read header.
   TileCacheSetHeader header;

   BytesToStruct ( bytes_iter, bytes.cend (), header ) ;

   if (header.magic != TILECACHESET_MAGIC)
   {
       Ogre::LogManager::getSingletonPtr()->logMessage("Error: File does not appear to contain valid tilecache data.");
       return false;
   }

   if (header.version != TILECACHESET_VERSION)
   {
       Ogre::LogManager::getSingletonPtr()->logMessage("Error: File contains a different version of the tilecache data format ("+Ogre::StringConverter::toString(header.version)+" instead of "+Ogre::StringConverter::toString(TILECACHESET_VERSION)+").");
       return false;
   }

   m_navMesh = dtAllocNavMesh();
   if (!m_navMesh)
   {
       Ogre::LogManager::getSingletonPtr()->logMessage("Error: Could not allocate navmesh.");
       return false;
   }

   dtStatus status = m_navMesh->init(&header.meshParams);
   if (dtStatusFailed(status))
   {
       Ogre::LogManager::getSingletonPtr()->logMessage("Error: Could not init navmesh.");
       return false;
   }

   m_tileCache = dtAllocTileCache();
   if (!m_tileCache)
   {
       Ogre::LogManager::getSingletonPtr()->logMessage("Error: Could not allocate tilecache.");
       return false;
   }

   status = m_tileCache->init(&header.cacheParams, m_talloc, m_tcomp, m_tmproc);
   if (dtStatusFailed(status))
   {
       Ogre::LogManager::getSingletonPtr()->logMessage("Error: Could not init tilecache.");
       return false;
   }

   memcpy(&m_cfg, &header.recastConfig, sizeof(rcConfig));

   // Read tiles.
   for (int i = 0; i < header.numTiles; ++i)
   {
           TileCacheTileHeader tileHeader;

           BytesToStruct ( bytes_iter, bytes.cend (), tileHeader ) ;

           if (!tileHeader.tileRef || !tileHeader.dataSize)
                   break;

           unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
           if (!data) break;
           memset(data, 0, tileHeader.dataSize);

           BytesToData ( bytes_iter, bytes.cend (), data, tileHeader.dataSize ) ;

           dtCompressedTileRef tile = 0;
           m_tileCache->addTile(data, tileHeader.dataSize, DT_COMPRESSEDTILE_FREE_DATA, &tile);

           if (tile)
                   m_tileCache->buildNavMeshTile(tile, m_navMesh);
   }

   // Init recast navmeshquery with created navmesh (in OgreRecast component)
   NavQuery.init(m_navMesh, MAX_SEARCH_NODES );

   // Config
   // TODO handle this nicer, also inputGeom is not inited, making some functions crash
   m_cellSize = m_cfg.cs;
   m_tileSize = m_cfg.tileSize;

   // cache bounding box
   const float* bmin = m_cfg.bmin;
   const float* bmax = m_cfg.bmax;

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

   // Max tiles and max polys affect how the tile IDs are caculated.
   // There are 22 bits available for identifying a tile and a polygon.
   int tileBits = rcMin((int)dtIlog2(dtNextPow2(tw*th*EXPECTED_LAYERS_PER_TILE)), 14);
   if (tileBits > 14) tileBits = 14;
   int polyBits = 22 - tileBits;
   m_maxTiles = 1 << tileBits;
   m_maxPolysPerTile = 1 << polyBits;

   // Build initial meshes
   // Builds detour compatible navmesh from all tiles.
   // A tile will have to be rebuilt if something changes, eg. a temporary obstacle is placed on it.
   for (int y = 0; y < m_th; ++y)
   {
       for (int x = 0; x < m_tw; ++x)
       {
           m_tileCache->buildNavMeshTilesAt(x,y, m_navMesh); // This immediately builds the tile, without the need of a dtTileCache::update()
       }
   }

   // InputGeometry only required when the Navmesh is being built, so we don't need to restore it

   return true;
}

void
OgreDetourTileCache::
HandleUpdate ( const float delta_time,
               const bool  until_up_to_date ) // Continue processing the tile cache obstacles until the entire navmesh is up-to-date
{
   if ( ! m_navMesh )
   {
      return ;
   }

   if ( ! m_tileCache )
   {
      return ;
   }

   if ( ! until_up_to_date )
   {
      m_tileCache->update ( delta_time, m_navMesh ) ;
   }
   else
   {
      bool up_to_date = false ;

      while ( ! up_to_date )
      {
         m_tileCache->update ( delta_time, m_navMesh, &up_to_date ) ;
      }
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

      m_tileCache->addBoxObstacle ( bmin, bmax, &result, area_id, flags ) ; // No rotation
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

      m_tileCache->addBoxObstacle ( centre_position, half_extents, y_rotation, &result, area_id, flags ) ;
   }

   return result ;
}

const dtTileCacheObstacle *
OgreDetourTileCache::
GetObstacleByRef ( dtObstacleRef ref )
{
   return m_tileCache->getObstacleByRef ( ref ) ;
}

bool
OgreDetourTileCache::
RemoveObstacle ( dtObstacleRef obstacleRef )
{
    if(m_tileCache->removeObstacle(obstacleRef) == DT_SUCCESS)
        return true;
    else
        return false;
}

int
OgreDetourTileCache::
AddConvexVolume ( std::unique_ptr <ConvexVolume> vol )
{
    //// The maximum number of convex volumes that can be added to the navmesh equals the max amount
    //// of volumes that can be added to the inputGeom it is built from.
    //if (m_volumeCount >= OgreDetourTileCache::MAX_VOLUMES)
    //    return -1;

    //m_volumes[m_volumeCount] = vol;
    //m_volumeCount++;

    //return m_volumeCount-1; // Return index of created volume

   m_volumes.push_back ( std::move ( vol ) ) ;

   return static_cast <int> ( m_volumes.size () - 1 ) ;
}

bool
OgreDetourTileCache::
DeleteConvexVolume ( int i )
{
    if(i >= static_cast<int>(m_volumes.size ()) || i < 0)
        return false;

    //m_volumeCount--;
    //m_volumes[i] = m_volumes[m_volumeCount];

    m_volumes.erase ( m_volumes.begin () + i ) ;

    return true;
}

bool
OgreDetourTileCache::
ConfigureTileCacheContext ()
{
    // Reuse OgreRecast context for tiled navmesh building

    if (!InputGeometry) {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: OgreDetourTileCache::configure: No vertices and triangles.");
        return false;
    }

    if (!InputGeometry->getChunkyMesh()) {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: OgreDetourTileCache::configure: Input mesh has no chunkyTriMesh built.");
        return false;
    }

    // Init cache bounding box
    const float* bmin = InputGeometry->getMeshBoundsMin();
    const float* bmax = InputGeometry->getMeshBoundsMax();

    // Navmesh generation params

    // Most params are taken from OgreRecast::configure, except for these:
    m_cfg.tileSize = m_tileSize;
    m_cfg.borderSize = (int) (m_cfg.walkableRadius + BORDER_PADDING); // Reserve enough padding.
    m_cfg.width = m_cfg.tileSize + m_cfg.borderSize*2;
    m_cfg.height = m_cfg.tileSize + m_cfg.borderSize*2;

    // Set mesh bounds
    rcVcopy(m_cfg.bmin, bmin);
    rcVcopy(m_cfg.bmax, bmax);

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

    return InitTileCache();
}

int
OgreDetourTileCache::
RasterizeTileLayers ( const int     tx,
                      const int     ty,
                      TileCacheData *tiles,
                      const int     maxTiles )
{
    if (!InputGeometry) {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildTile: Input mesh is not specified.");
        return 0;
    }

    if (!InputGeometry->getChunkyMesh()) {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildTile: Input mesh has no chunkyTriMesh built.");
        return 0;
    }

//TODO make these member variables?
    FastLZCompressor comp;
    RasterizationContext rc;

    const float* verts = InputGeometry->getVerts();
    const int nverts = InputGeometry->getVertCount();

    // The chunky tri mesh in the inputgeom is a simple spatial subdivision structure that allows to
    // process the vertices in the geometry relevant to this part of the tile.
    // The chunky tri mesh is a grid of axis aligned boxes that store indices to the vertices in verts
    // that are positioned in that box.
    const rcChunkyTriMesh* chunkyMesh = InputGeometry->getChunkyMesh();

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
    auto* solid = rcAllocHeightfield();
    if (!solid)
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildNavigation: Out of memory 'solid'.");
        return 0;
    }
    if (!rcCreateHeightfield(&m_ctx, *solid, tcfg.width, tcfg.height, tcfg.bmin, tcfg.bmax, tcfg.cs, tcfg.ch))
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
        rcMarkWalkableTriangles(&m_ctx, tcfg.walkableSlopeAngle,
                                verts, nverts, tris, ntris, rc.triareas);

        rcRasterizeTriangles(&m_ctx, verts, nverts, tris, rc.triareas, ntris, *solid, tcfg.walkableClimb);
    }

    // Once all geometry is rasterized, we do initial pass of filtering to
    // remove unwanted overhangs caused by the conservative rasterization
    // as well as filter spans where the character cannot possibly stand.
    rcFilterLowHangingWalkableObstacles(&m_ctx, tcfg.walkableClimb, *solid);
    rcFilterLedgeSpans(&m_ctx, tcfg.walkableHeight, tcfg.walkableClimb, *solid);
    rcFilterWalkableLowHeightSpans(&m_ctx, tcfg.walkableHeight, *solid);


    rc.chf = rcAllocCompactHeightfield();
    if (!rc.chf)
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildNavigation: Out of memory 'chf'.");
        return 0;
    }
    if (!rcBuildCompactHeightfield(&m_ctx, tcfg.walkableHeight, tcfg.walkableClimb, *solid, *rc.chf))
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildNavigation: Could not build compact data.");
        return 0;
    }

    if  ( ! m_keep_heightfield )
    {
       rcFreeHeightField ( solid ) ;
       solid = 0 ;
    }
    else
    {
       m_solid.push_back ( solid ) ;
    }

    // Erode the walkable area by agent radius.
    if (!rcErodeWalkableArea(&m_ctx, tcfg.walkableRadius, *rc.chf))
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildNavigation: Could not erode.");
        return 0;
    }

    // Mark areas of dynamically added convex polygons
    for ( const auto &vol : m_volumes )
    {
        rcMarkConvexPolyArea(&m_ctx, vol->verts, vol->nverts,
                             vol->hmin, vol->hmax,
                             (unsigned char)vol->area, *rc.chf);
    }



    // Up till this part was more or less the same as OgreRecast::NavMeshBuild()
    // The following part is specific for creating a 2D intermediary navmesh tile.

    rc.lset = rcAllocHeightfieldLayerSet();
    if (!rc.lset)
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildNavigation: Out of memory 'lset'.");
        return 0;
    }
    if (!rcBuildHeightfieldLayers(&m_ctx, *rc.chf, tcfg.borderSize, tcfg.walkableHeight, *rc.lset))
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

bool
OgreDetourTileCache::
InitTileCache ()
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

    dtFreeNavMesh(m_navMesh);

    m_navMesh = dtAllocNavMesh();
    if (! m_navMesh)
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

    status = m_navMesh->init(&params);
    if (dtStatusFailed(status))
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildTiledNavigation: Could not init navmesh.");
        return false;
    }

    // Init recast navmeshquery with created navmesh (in OgreRecast component)
    status = NavQuery.init(m_navMesh, MAX_SEARCH_NODES );
    if (dtStatusFailed(status))
    {
        Ogre::LogManager::getSingleton ().logMessage("ERROR: buildTiledNavigation: Could not init Detour navmesh query");
        return false;
    }

    return true;
}
