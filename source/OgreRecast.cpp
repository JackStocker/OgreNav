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

#include "OgreRecast.h"
#include "RecastInputGeom.h"
#include "DetourTileCacheBuilder.h"
//#include "OgreRecastNavmeshPruner.h"
#include "PlayerFlagQueryFilter.h"

OgreRecast::
OgreRecast ( OgreRecastConfigParams configParams )
    : //m_rebuildSg(false),
      mFilter(0),
      //mNavmeshPruner(0),
      m_ctx(0)
{
   // Init recast stuff in a safe state

   //m_triareas=NULL;
   //m_solid=NULL ;
   //m_chf=NULL ;
   //m_cset=NULL;
   //m_pmesh=NULL;
   //m_cfg;
   //m_dmesh=NULL ;
   //m_geom=NULL;
   m_navMesh=NULL;
   m_navQuery=NULL;
   //m_navMeshDrawFlags;
   m_ctx=NULL ;

   RecastCleanup() ; // TODO ?? don't know if I should do this prior to making any recast stuff, but the demo did.
   //m_pRecastMOPath=NULL ;

   //m_pRecastSN=m_pSceneMgr->getRootSceneNode()->createChildSceneNode("RecastSN");


   m_pLog = Ogre::LogManager::getSingletonPtr();


   // Set default size of box around points to look for nav polygons
   mExtents[0] = 32.0f; mExtents[1] = 32.0f; mExtents[2] = 32.0f;

   // Setup the default query filter
   mFilter = new PlayerFlagQueryFilter () ;
   mFilter->setIncludeFlags ( POLYFLAGS_ALL ) ;
   mFilter->setExcludeFlags ( 0 ) ;
   mFilter->setAreaCost ( POLYAREA_GRASS, 2.0f  ) ;
   mFilter->setAreaCost ( POLYAREA_WATER, 10.0f ) ;
   mFilter->setAreaCost ( POLYAREA_ROAD,  1.0f ) ;
   mFilter->setAreaCost ( POLYAREA_SAND,  4.0f  ) ;

   // Init path store. MaxVertex 0 means empty path slot
   for(int i = 0; i < MAX_PATHSLOT; i++) {
       m_PathStore[i].MaxVertex = 0;
       m_PathStore[i].Target = 0;
   }


   // Set configuration
   configure(configParams);
}


/**
 * Cleanup recast stuff, not debug manualobjects.
**/
void OgreRecast::RecastCleanup()
{
   //if(m_triareas) delete [] m_triareas;
   //m_triareas = 0;

   //rcFreeHeightField(m_solid);
   //m_solid = 0;
   //rcFreeCompactHeightfield(m_chf);
   //m_chf = 0;
   //rcFreeContourSet(m_cset);
   //m_cset = 0;
   //rcFreePolyMesh(m_pmesh);
   //m_pmesh = 0;
   //rcFreePolyMeshDetail(m_dmesh);
   //m_dmesh = 0;
   dtFreeNavMesh(m_navMesh);
   m_navMesh = 0;

   dtFreeNavMeshQuery(m_navQuery);
   m_navQuery = 0 ;

   if(m_ctx){
       delete m_ctx;
       m_ctx = 0;
   }
}


void OgreRecast::configure(OgreRecastConfigParams params)
{
    // NOTE: this is one of the most important parts to get it right!!
    // Perhaps the most important part of the above is setting the agent size with m_agentHeight and m_agentRadius,
    // and the voxel cell size used, m_cellSize and m_cellHeight. In my project 1 units is a little less than 1 meter,
    // so I've set the agent to 2.5 units high, and the cell sizes to sub-meter size.
    // This is about the same as in the original cell sizes in the Recast/Detour demo.

    // Smaller cellsizes are the most accurate at finding all the places we could go, but are also slow to generate.
    // Might be suitable for pre-generated meshes. Though it also produces a lot more polygons.

    if(m_ctx) {
        delete m_ctx;
        m_ctx = 0;
    }
    m_ctx=new rcContext(true);

    //m_cellSize = params.getCellSize();
    //m_cellHeight = params.getCellHeight();
    //m_agentMaxSlope = params.getAgentMaxSlope();
    //m_agentHeight = params.getAgentHeight();
    //m_agentMaxClimb = params.getAgentMaxClimb();
    //m_agentRadius = params.getAgentRadius();
    //m_edgeMaxLen = params.getEdgeMaxLen();
    //m_edgeMaxError = params.getEdgeMaxError();
    //m_regionMinSize = params.getRegionMinSize();
    //m_regionMergeSize = params.getRegionMergeSize();
    //m_vertsPerPoly = params.getVertsPerPoly();
    //m_detailSampleDist = params.getDetailSampleDist();
    //m_detailSampleMaxError = params.getDetailSampleMaxError();
    //m_keepInterResults = params.getKeepInterResults();

    // Init cfg object
    memset(&m_cfg, 0, sizeof(m_cfg));
    m_cfg.cs = params.getCellSize();
    m_cfg.ch = params.getCellHeight();
    m_cfg.walkableSlopeAngle = params.getAgentMaxSlope();
    m_cfg.walkableHeight = params._getWalkableheight();
    m_cfg.walkableClimb = params._getWalkableClimb();
    m_cfg.walkableRadius = params._getWalkableRadius();
    m_cfg.maxEdgeLen = params._getMaxEdgeLen();
    m_cfg.maxSimplificationError = params.getEdgeMaxError();
    m_cfg.minRegionArea = params._getMinRegionArea();
    m_cfg.mergeRegionArea = params._getMergeRegionArea();
    m_cfg.maxVertsPerPoly = params.getVertsPerPoly();
    m_cfg.detailSampleDist = (float) params._getDetailSampleDist();
    m_cfg.detailSampleMaxError = (float) params._getDetailSampleMaxError();


    // Demo specific parameters
    //m_navMeshOffsetFromGround = m_cellHeight/5;         // Distance above ground for drawing navmesh polygons
    //m_navMeshEdgesOffsetFromGround = m_cellHeight/3;    // Distance above ground for drawing edges of navmesh (should be slightly higher than navmesh polygons)
    //m_pathOffsetFromGround = m_agentHeight+m_navMeshOffsetFromGround; // Distance above ground for drawing path debug lines relative to cellheight (should be higher than navmesh polygons)

    // Colors for navmesh debug drawing
    //m_navmeshNeighbourEdgeCol= Ogre::ColourValue(0.9,0.9,0.9);   // Light Grey
    //m_navmeshOuterEdgeCol    = Ogre::ColourValue(0,0,0);         // Black
    //m_navmeshGroundPolygonCol= Ogre::ColourValue(0,0.7,0);       // Green
    //m_navmeshOtherPolygonCol = Ogre::ColourValue(0,0.175,0);     // Dark green
    //m_pathCol                = Ogre::ColourValue(1,0,0);         // Red
}



/**
 * Now for the navmesh creation function.
 * I've mostly taken this from the demo, apart from the top part where I create the triangles. Recast needs a bunch of input vertices and triangles from your map to build the navigation mesh. Where you get those verts amd triangles is up to you, my map loader was already outputing verts and triangle so it was easy to use those. Make sure the triangles wind the correct way or Recast will try to build the navmesh on the outside of your map.
 * There's some potentially groovy stuff in there that I haven't touched, like filtering and different weights for different types of zones. Also I've just gone for the simplest navmesh type, there's also other modes like tiling navmeshes which I've ignored.
 * This method is heavily based on Sample_SoloMesh::handleBuild() from the recastnavigation demo.
 *
 * Perhaps the most important part of the above is setting the agent size with m_agentHeight and m_agentRadius, and the voxel cell size used, m_cellSize and m_cellHeight. In my project 32.0 units is 1 meter, so I've set the agent to 48 units high, and the cell sizes are quite large. The original cell sizes in the Recast/Detour demo were down around 0.3.
**/
//bool OgreRecast::NavMeshBuild(std::vector<Ogre::Entity*> srcMeshes)
//{
//    if (srcMeshes.empty()) {
//        Ogre::LogManager::getSingletonPtr()->logMessage("Warning: Called NavMeshBuild without any entities. No navmesh was built.");
//        return false;
//    }
//
//    return NavMeshBuild(new InputGeom(srcMeshes));
//}

//bool OgreRecast::NavMeshBuild(InputGeom* input)
//{
//    // TODO: clean up unused variables
//
//
//   m_pLog->logMessage("NavMeshBuild Start");
//
//
//   //
//   // Step 1. Initialize build config.
//   //
//
//   // Reset build times gathering.
//   m_ctx->resetTimers();
//
//   // Start the build process.
//   m_ctx->startTimer(RC_TIMER_TOTAL);
//
//
//
//
//
//
//   //
//   // Step 2. Rasterize input polygon soup.
//   //
//
//   InputGeom *inputGeom = input;
//   rcVcopy(m_cfg.bmin, inputGeom->getMeshBoundsMin());
//   rcVcopy(m_cfg.bmax, inputGeom->getMeshBoundsMax());
//   rcCalcGridSize(m_cfg.bmin, m_cfg.bmax, m_cfg.cs, &m_cfg.width, &m_cfg.height);
//
//   int nverts = inputGeom->getVertCount();
//   int ntris = inputGeom->getTriCount();
//   Ogre::Vector3 min; FloatAToOgreVect3(inputGeom->getMeshBoundsMin(), min);
//   Ogre::Vector3 max; FloatAToOgreVect3(inputGeom->getMeshBoundsMax(), max);
//
//   //Ogre::LogManager::getSingletonPtr()->logMessage("Bounds: "+Ogre::StringConverter::toString(min) + "   "+ Ogre::StringConverter::toString(max));
//
//
//   m_pLog->logMessage("Building navigation:");
//   m_pLog->logMessage(" - " + Ogre::StringConverter::toString(m_cfg.width) + " x " + Ogre::StringConverter::toString(m_cfg.height) + " cells");
//   m_pLog->logMessage(" - " + Ogre::StringConverter::toString(nverts/1000.0f) + " K verts, " + Ogre::StringConverter::toString(ntris/1000.0f) + " K tris");
//
//   // Allocate voxel heightfield where we rasterize our input data to.
//   m_solid = rcAllocHeightfield();
//   if (!m_solid)
//   {
//      m_pLog->logMessage("ERROR: buildNavigation: Out of memory 'solid'.");
//      return false;
//   }
//   if (!rcCreateHeightfield(m_ctx, *m_solid, m_cfg.width, m_cfg.height, m_cfg.bmin, m_cfg.bmax, m_cfg.cs, m_cfg.ch))
//   {
//      m_pLog->logMessage("ERROR: buildNavigation: Could not create solid heightfield. Possibly it requires too much memory, try setting a higher cellSize and cellHeight value.");
//      return false;
//   }
//
//   // Allocate array that can hold triangle area types.
//   // If you have multiple meshes you need to process, allocate
//   // an array which can hold the max number of triangles you need to process.
//   m_triareas = new unsigned char[ntris];
//   if (!m_triareas)
//   {
//       m_pLog->logMessage("ERROR: buildNavigation: Out of memory 'm_triareas' ("+Ogre::StringConverter::toString(ntris)+").");
//      return false;
//   }
//
//   // Find triangles which are walkable based on their slope and rasterize them.
//   // If your input data is multiple meshes, you can transform them here, calculate
//   // the are type for each of the meshes and rasterize them.
//   memset(m_triareas, 0, ntris*sizeof(unsigned char));
//   rcMarkWalkableTriangles(m_ctx, m_cfg.walkableSlopeAngle, inputGeom->getVerts(), inputGeom->getVertCount(), inputGeom->getTris(), inputGeom->getTriCount(), m_triareas);
//   rcRasterizeTriangles(m_ctx, inputGeom->getVerts(), inputGeom->getVertCount(), inputGeom->getTris(), m_triareas, inputGeom->getTriCount(), *m_solid, m_cfg.walkableClimb);
//
//   if (!m_keepInterResults)
//   {
//      delete [] m_triareas;
//      m_triareas = 0;
//   }
//
//
//
//
//
//   //
//   // Step 3. Filter walkables surfaces.
//   //
//
//   // Once all geoemtry is rasterized, we do initial pass of filtering to
//   // remove unwanted overhangs caused by the conservative rasterization
//   // as well as filter spans where the character cannot possibly stand.
//   rcFilterLowHangingWalkableObstacles(m_ctx, m_cfg.walkableClimb, *m_solid);
//   rcFilterLedgeSpans(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid);
//   rcFilterWalkableLowHeightSpans(m_ctx, m_cfg.walkableHeight, *m_solid);
//
//
//
//
//
//
//
//
//   //
//   // Step 4. Partition walkable surface to simple regions.
//   //
//
//   // Compact the heightfield so that it is faster to handle from now on.
//   // This will result more cache coherent data as well as the neighbours
//   // between walkable cells will be calculated.
//   m_chf = rcAllocCompactHeightfield();
//   if (!m_chf)
//   {
//      m_pLog->logMessage("ERROR: buildNavigation: Out of memory 'chf'.");
//      return false;
//   }
//   if (!rcBuildCompactHeightfield(m_ctx, m_cfg.walkableHeight, m_cfg.walkableClimb, *m_solid, *m_chf))
//   {
//      m_pLog->logMessage("ERROR: buildNavigation: Could not build compact data.");
//      return false;
//   }
//
//   if (!m_keepInterResults)
//   {
//      rcFreeHeightField(m_solid);
//      m_solid = 0;
//   }
//
//
//   // Erode the walkable area by agent radius.
//   if (!rcErodeWalkableArea(m_ctx, m_cfg.walkableRadius, *m_chf))
//   {
//      m_pLog->logMessage("ERROR: buildNavigation: Could not erode walkable areas.");
//      return false;
//   }
//
//// TODO implement
//   // (Optional) Mark areas.
//   const ConvexVolume * const *vols = inputGeom->getConvexVolumes();
//   for (int i  = 0; i < inputGeom->getConvexVolumeCount(); ++i)
//      rcMarkConvexPolyArea(m_ctx, vols[i]->verts, vols[i]->nverts, vols[i]->hmin, vols[i]->hmax, (unsigned char)vols[i]->area, *m_chf);
//
//
//   // Prepare for region partitioning, by calculating distance field along the walkable surface.
//   if (!rcBuildDistanceField(m_ctx, *m_chf))
//   {
//      m_pLog->logMessage("ERROR: buildNavigation: Could not build distance field.");
//      return false;
//   }
//
//   // Partition the walkable surface into simple regions without holes.
//   if (!rcBuildRegions(m_ctx, *m_chf, m_cfg.borderSize, m_cfg.minRegionArea, m_cfg.mergeRegionArea))
//   {
//      m_pLog->logMessage("ERROR: buildNavigation: Could not build regions.");
//      return false;
//   }
//
//
//
//
//
//
//
//
//   //
//   // Step 5. Trace and simplify region contours.
//   //
//
//   // Create contours.
//   m_cset = rcAllocContourSet();
//   if (!m_cset)
//   {
//      m_pLog->logMessage("ERROR: buildNavigation: Out of memory 'cset'.");
//      return false;
//   }
//   if (!rcBuildContours(m_ctx, *m_chf, m_cfg.maxSimplificationError, m_cfg.maxEdgeLen, *m_cset))
//   {
//      m_pLog->logMessage("ERROR: buildNavigation: Could not create contours.");
//      return false;
//   }
//
//   if (m_cset->nconts == 0)
//   {
//       // In case of errors see: http://groups.google.com/group/recastnavigation/browse_thread/thread/a6fbd509859a12c8
//       // You should probably tweak the parameters
//           m_pLog->logMessage("ERROR: No contours created (Recast)!");
//    }
//
//
//
//
//
//   //
//   // Step 6. Build polygons mesh from contours.
//   //
//
//   // Build polygon navmesh from the contours.
//   m_pmesh = rcAllocPolyMesh();
//   if (!m_pmesh)
//   {
//      m_pLog->logMessage("ERROR: buildNavigation: Out of memory 'pmesh'.");
//      return false;
//   }
//   if (!rcBuildPolyMesh(m_ctx, *m_cset, m_cfg.maxVertsPerPoly, *m_pmesh))
//   {
//       // Try modifying the parameters. I experienced this error when setting agentMaxClimb too high.
//      m_pLog->logMessage("ERROR: buildNavigation: Could not triangulate contours.");
//      return false;
//   }
//
//
//
//
//
//
//
//
//
//   //
//   // Step 7. Create detail mesh which allows to access approximate height on each polygon.
//   //
//
//   m_dmesh = rcAllocPolyMeshDetail();
//   if (!m_dmesh)
//   {
//      m_pLog->logMessage("ERROR: buildNavigation: Out of memory 'pmdtl'.");
//      return false;
//   }
//
//   if (!rcBuildPolyMeshDetail(m_ctx, *m_pmesh, *m_chf, m_cfg.detailSampleDist, m_cfg.detailSampleMaxError, *m_dmesh))
//   {
//      m_pLog->logMessage("ERROR: buildNavigation: Could not build detail mesh.");
//      return false;
//   }
//
//   if (!m_keepInterResults)
//   {
//      rcFreeCompactHeightfield(m_chf);
//      m_chf = 0;
//      rcFreeContourSet(m_cset);
//      m_cset = 0;
//   }
//
//   // At this point the navigation mesh data is ready, you can access it from m_pmesh.
//   // See duDebugDrawPolyMesh or dtCreateNavMeshData as examples how to access the data.
//
//
//
//
//
//
//
//
//   //
//   // (Optional) Step 8. Create Detour data from Recast poly mesh.
//   //
//
//   // The GUI may allow more max points per polygon than Detour can handle.
//   // Only build the detour navmesh if we do not exceed the limit.
//
//
//   if (m_cfg.maxVertsPerPoly <= DT_VERTS_PER_POLYGON)
//   {
//      m_pLog->logMessage("Detour 1000");
//
//      unsigned char* navData = 0;
//      int navDataSize = 0;
//
//
//      // Update poly flags from areas.
//      for (int i = 0; i < m_pmesh->npolys; ++i)
//      {
//         if (m_pmesh->areas[i] == RC_WALKABLE_AREA)
//         {
//            m_pmesh->areas[i] = POLYAREA_GRASS;
//            m_pmesh->flags[i] = POLYFLAGS_WALK;
//         }
//      }
//
//
//      // Set navmesh params
//      dtNavMeshCreateParams params;
//      memset(&params, 0, sizeof(params));
//      params.verts = m_pmesh->verts;
//      params.vertCount = m_pmesh->nverts;
//      params.polys = m_pmesh->polys;
//      params.polyAreas = m_pmesh->areas;
//      params.polyFlags = m_pmesh->flags;
//      params.polyCount = m_pmesh->npolys;
//      params.nvp = m_pmesh->nvp;
//      params.detailMeshes = m_dmesh->meshes;
//      params.detailVerts = m_dmesh->verts;
//      params.detailVertsCount = m_dmesh->nverts;
//      params.detailTris = m_dmesh->tris;
//      params.detailTriCount = m_dmesh->ntris;
//
//      // no off mesh connections yet
//      //m_offMeshConCount=0 ;
//      params.offMeshConVerts  = nullptr;//m_offMeshConVerts ;
//      params.offMeshConRad    = nullptr;//m_offMeshConRads ;
//      params.offMeshConDir    = nullptr;//m_offMeshConDirs ;
//      params.offMeshConAreas  = nullptr;//m_offMeshConAreas ;
//      params.offMeshConFlags  = nullptr;//m_offMeshConFlags ;
//      params.offMeshConUserID = nullptr;//m_offMeshConId ;
//      params.offMeshConCount  = 0;//m_offMeshConCount ;
//
//      params.walkableHeight = m_agentHeight;
//      params.walkableRadius = m_agentRadius;
//      params.walkableClimb = m_agentMaxClimb;
//      rcVcopy(params.bmin, m_pmesh->bmin);
//      rcVcopy(params.bmax, m_pmesh->bmax);
//      params.cs = m_cfg.cs;
//      params.ch = m_cfg.ch;
//
//
//      m_pLog->logMessage("Detour 2000");
//
//      if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
//      {
//         m_pLog->logMessage("ERROR: Could not build Detour navmesh.");
//         return false;
//      }
//
//      m_pLog->logMessage("Detour 3000");
//
//      m_navMesh = dtAllocNavMesh();
//      if (!m_navMesh)
//      {
//         dtFree(navData);
//         m_pLog->logMessage("ERROR: Could not create Detour navmesh");
//         return false;
//      }
//
//      m_pLog->logMessage("Detour 4000");
//
//      dtStatus status;
//
//      status = m_navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
//      if (dtStatusFailed(status))
//      {
//         dtFree(navData);
//         m_pLog->logMessage("ERROR: Could not init Detour navmesh");
//         return false;
//      }
//
//      m_pLog->logMessage("Detour 5000");
//
//      m_navQuery = dtAllocNavMeshQuery();
//      status = m_navQuery->init(m_navMesh, 2048);
//
//      m_pLog->logMessage("Detour 5500");
//
//      if (dtStatusFailed(status))
//      {
//         m_pLog->logMessage("ERROR: Could not init Detour navmesh query");
//         return false;
//      }
//
//      m_pLog->logMessage("Detour 6000");
//   }
//
//   m_ctx->stopTimer(RC_TIMER_TOTAL);
//
//
//   ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//   // cleanup stuff we don't need
////   delete [] rc_verts ;
////   delete [] rc_tris ;
////   delete [] rc_trinorms ;
//
//   //CreateRecastPolyMesh(*m_pmesh) ;   // Debug render it
//
//   m_pLog->logMessage("NavMeshBuild End");
//   return true;
//}






#include <math.h>


/**
 * Now for the pathfinding code.
 * This takes a start point and an end point and, if possible, generates a list of lines in a path. It might fail if the start or end points aren't near any navmesh polygons, or if the path is too long, or it can't make a path, or various other reasons. So far I've not had problems though.
 *
 * nTarget: The index number for the slot in which the found path is to be stored
 * nPathSlot: Number identifying the target the path leads to
 *
 * Return codes:
 *  0   found path
 *  -1  Couldn't find polygon nearest to start point
 *  -2  Couldn't find polygon nearest to end point
 *  -3  Couldn't create a path
 *  -4  Couldn't find a path
 *  -5  Couldn't create a straight path
 *  -6  Couldn't find a straight path
**/
int OgreRecast::FindPath(float* pStartPos, float* pEndPos, int nPathSlot, int nTarget, const unsigned int include_flags, const unsigned int exclude_flags )
{
   dtStatus status ;
   dtPolyRef StartPoly ;
   float StartNearest[3] ;
   dtPolyRef EndPoly ;
   float EndNearest[3] ;
   dtPolyRef PolyPath[MAX_PATHPOLY] ;
   int nPathCount=0 ;
   float StraightPath[MAX_PATHVERT*3] ;
   int nVertCount=0 ;

   mFilter->setIncludeFlags ( include_flags ) ;
   mFilter->setExcludeFlags ( exclude_flags ) ;

   // find the start polygon
   status=m_navQuery->findNearestPoly(pStartPos, mExtents, mFilter, &StartPoly, StartNearest) ;
   if((status&DT_FAILURE) || (status&DT_STATUS_DETAIL_MASK)) return -1 ; // couldn't find a polygon

   // find the end polygon
   status=m_navQuery->findNearestPoly(pEndPos, mExtents, mFilter, &EndPoly, EndNearest) ;
   if((status&DT_FAILURE) || (status&DT_STATUS_DETAIL_MASK)) return -2 ; // couldn't find a polygon

   status=m_navQuery->findPath(StartPoly, EndPoly, StartNearest, EndNearest, mFilter, PolyPath, &nPathCount, MAX_PATHPOLY) ;

   if ( status & DT_PARTIAL_RESULT &&
        nPathCount > 0 )
   {
      auto new_start = PolyPath [ nPathCount -1 ] ;

      status=m_navQuery->findPath(new_start, EndPoly, StartNearest, EndNearest, mFilter, PolyPath, &nPathCount, MAX_PATHPOLY) ;
   }

   if((status&DT_FAILURE) || (status&DT_STATUS_DETAIL_MASK)) return -3 ; // couldn't create a path
   if(nPathCount==0) return -4 ; // couldn't find a path

   //status=m_navQuery->findStraightPath(StartNearest, EndNearest, PolyPath, nPathCount, StraightPath, NULL, NULL, &nVertCount, MAX_PATHVERT, DT_STRAIGHTPATH_ALL_CROSSINGS) ;
   status=m_navQuery->findStraightPath(StartNearest, EndNearest, PolyPath, nPathCount, StraightPath, NULL, NULL, &nVertCount, MAX_PATHVERT, DT_STRAIGHTPATH_AREA_CROSSINGS) ;
   if((status&DT_FAILURE) || (status&DT_STATUS_DETAIL_MASK)) return -5 ; // couldn't create a path
   if(nVertCount==0) return -6 ; // couldn't find a path

   // At this point we have our path.  Copy it to the path store
   int nIndex=0 ;
   for(int nVert=0 ; nVert<nVertCount ; nVert++)
   {
      m_PathStore[nPathSlot].PosX[nVert]=StraightPath[nIndex++] ;
      m_PathStore[nPathSlot].PosY[nVert]=StraightPath[nIndex++] ;
      m_PathStore[nPathSlot].PosZ[nVert]=StraightPath[nIndex++] ;

      //sprintf(m_chBug, "Path Vert %i, %f %f %f", nVert, m_PathStore[nPathSlot].PosX[nVert], m_PathStore[nPathSlot].PosY[nVert], m_PathStore[nPathSlot].PosZ[nVert]) ;
      //m_pLog->logMessage(m_chBug);
   }
   m_PathStore[nPathSlot].MaxVertex=nVertCount ;
   m_PathStore[nPathSlot].Target=nTarget ;

   return nVertCount ;

}

int OgreRecast::FindPath(Ogre::Vector3 startPos, Ogre::Vector3 endPos, int nPathSlot, int nTarget, const unsigned int include_flags, const unsigned int exclude_flags)
{
    float start[3];
    float end[3];
    OgreVect3ToFloatA(startPos, start);
    OgreVect3ToFloatA(endPos, end);

    return FindPath(start,end,nPathSlot,nTarget, include_flags, exclude_flags);
}

std::vector<Ogre::Vector3> OgreRecast::getPath(int pathSlot)
{
    std::vector<Ogre::Vector3> result;
    if(pathSlot < 0 || pathSlot >= MAX_PATHSLOT || m_PathStore[pathSlot].MaxVertex <= 0)
        return result;

    PATHDATA *path = &(m_PathStore[pathSlot]);
    result.reserve(path->MaxVertex);
    for(int i = 0; i < path->MaxVertex; i++) {
        result.push_back(Ogre::Vector3(path->PosX[i], path->PosY[i], path->PosZ[i]));
    }

    return result;
}


//int OgreRecast::getTarget(int pathSlot)
//{
//    if(pathSlot < 0 || pathSlot >= MAX_PATHSLOT)
//        return 0;
//
//    return m_PathStore[pathSlot].Target;
//}




/**
 * Debug drawing functionality:
**/

//void OgreRecast::drawNavMesh()
//{
//    if(m_pmesh)
//        drawPolyMesh(*m_pmesh);
//}

//void OgreRecast::drawPolyMesh(const struct rcPolyMesh &mesh, bool colorRegions)
//{
//    const int nvp = mesh.nvp;
//    const float cs = mesh.cs;
//    const float ch = mesh.ch;
//    const float* orig = mesh.bmin;
//
//    const unsigned short* verts = mesh.verts;
//    const unsigned short* polys = mesh.polys;
//    const unsigned char* areas = mesh.areas;
//    const unsigned short* regions = mesh.regs;
//    const int nverts = mesh.nverts;
//    const int npolys = mesh.npolys;
//    const int maxpolys = mesh.maxpolys;
//
//
//    //CreateRecastPolyMesh("SingleNavmesh", verts, nverts, polys, npolys, areas, maxpolys, regions, nvp, cs, ch, orig, colorRegions);
//}

// TODO make this only create an ogre entity, put the demo specific drawing in a separate DebugDrawing class to separate it from the reusable recast wrappers
//void OgreRecast::CreateRecastPolyMesh(const Ogre::String name, const unsigned short *verts, const int nverts, const unsigned short *polys, const int npolys, const unsigned char *areas, const int maxpolys, const unsigned short *regions, const int nvp, const float cs, const float ch, const float *orig, bool colorRegions)
//{
//   //Ogre::ManualObject *new_walk_tile      = NULL ;
//   //Ogre::ManualObject *new_walk_neighbour = NULL ;
//   //Ogre::ManualObject *new_walk_boundary  = NULL ;
//
//   DebugTile new_walk_tile ;
//   DebugTile new_walk_neighbour ;
//   DebugTile new_walk_boundary ;
//
//    //m_flDataX=npolys ;
//    //m_flDataY=nverts ;
//    //return ;
//   // When drawing regions choose different random colors for each region
//// TODO maybe cache generated colors so when rebuilding tiles the same colors can be reused? If possible
//   Ogre::ColourValue region_colour ( 0.2f, 0.8f, 1.0f ) ;
//
//   int nIndex = 0 ;
//   auto m_nAreaCount = npolys ;
//   auto m_navMeshOffsetFromGround = 0.1f;
//
//   if ( m_nAreaCount )
//   {
//      // start defining the manualObject with the navmesh planes
//      //new_walk_tile = new Ogre::ManualObject ( "RecastMOTile_" + name ) ;
//      new_walk_tile.Name = "RecastMOTile_" + name ;
//      new_walk_tile.RenderType = Ogre::RenderOperation::OT_TRIANGLE_LIST ;
//
//      //m_pRecastMOWalk = m_pSceneMgr->createManualObject("RecastMOWalk_"+name);
//      //new_walk_tile->begin("recastdebug", Ogre::RenderOperation::OT_TRIANGLE_LIST) ;
//
//      for (int i = 0; i < npolys; ++i) {    // go through all polygons
//         //if (areas[i] == POLYAREA_GRASS || areas[i] == DT_TILECACHE_WALKABLE_AREA)
//         {
//            const unsigned short* p = &polys[i*nvp*2];
//
//            unsigned short vi[3];
//            for (int j = 2; j < nvp; ++j) // go through all verts in the polygon
//            {
//               if (p[j] == RC_MESH_NULL_IDX) break;
//               vi[0] = p[0];
//               vi[1] = p[j-1];
//               vi[2] = p[j];
//
//               std::array<Ogre::Vector3, 3> triangle ;
//
//               for (int k = 0; k < 3; ++k) // create a 3-vert triangle for each 3 verts in the polygon.
//               {
//                  const unsigned short* v = &verts[vi[k]*3];
//                  const float x = orig[0] + v[0]*cs;
//                  const float y = orig[1] + (v[1]/*+1*/)*ch;
//                  const float z = orig[2] + v[2]*cs;
//
//                  triangle [ k ] = Ogre::Vector3 ( x, y+m_navMeshOffsetFromGround, z ) ;
//
//                  //new_walk_tile.push_back ( Ogre::Vector3 ( x, y+m_navMeshOffsetFromGround, z ) ) ;
//                  /*new_walk_tile->position(x, y+m_navMeshOffsetFromGround, z);
//                  if(colorRegions) {
//                      new_walk_tile->colour(region_colour);  // Assign vertex color
//                  } else {
//                      if (areas[i] == SAMPLE_POLYAREA_GROUND)
//                         new_walk_tile->colour(m_navmeshGroundPolygonCol);
//                      else
//                         new_walk_tile->colour(m_navmeshOtherPolygonCol);
//                  }*/
//
//               }
//
//               new_walk_tile.Triangles.push_back ( triangle ) ;
//
//               //new_walk_tile->triangle(nIndex, nIndex+1, nIndex+2) ;
//               nIndex+=3 ;
//            }
//         }
//     }
//      //new_walk_tile->end() ;
//
//
//
//      // Define manualObject with the navmesh edges between neighbouring polygons
//      //new_walk_neighbour = new Ogre::ManualObject ( "RecastMONeighbour_" + name ) ;
//      new_walk_neighbour.Name = "RecastMONeighbour_" + name ;
//      new_walk_neighbour.RenderType = Ogre::RenderOperation::OT_LINE_LIST ;
//
//      //m_pRecastMONeighbour = m_pSceneMgr->createManualObject("RecastMONeighbour_"+name);
//      //new_walk_neighbour->begin("recastdebug", Ogre::RenderOperation::OT_LINE_LIST) ;
//
//      for (int i = 0; i < npolys; ++i)
//      {
//         const unsigned short* p = &polys[i*nvp*2];
//         for (int j = 0; j < nvp; ++j)
//         {
//            if (p[j] == RC_MESH_NULL_IDX) break;
//            if (p[nvp+j] == RC_MESH_NULL_IDX) continue;
//            int vi[2];
//            vi[0] = p[j];
//            if (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX)
//               vi[1] = p[0];
//            else
//               vi[1] = p[j+1];
//
//            std::array<Ogre::Vector3, 3> triangle ;
//            triangle [ 2 ] = Ogre::Vector3 ( -1,-1,-1 ) ;
//
//            for (int k = 0; k < 2; ++k)
//            {
//               const unsigned short* v = &verts[vi[k]*3];
//               const float x = orig[0] + v[0]*cs;
//               const float y = orig[1] + (v[1]/*+1*/)*ch /*+ 0.1f*/;
//               const float z = orig[2] + v[2]*cs;
//               //dd->vertex(x, y, z, coln);
//               //new_walk_neighbour->position(x, y+m_navMeshEdgesOffsetFromGround, z) ;
//               //new_walk_neighbour->colour(m_navmeshNeighbourEdgeCol) ;
//
//               //new_walk_neighbour.push_back ( Ogre::Vector3 ( x, y+m_navMeshEdgesOffsetFromGround, z ) ) ;
//
//               triangle [ k ] = Ogre::Vector3 ( x, y+m_navMeshOffsetFromGround, z ) ;
//            }
//
//            new_walk_neighbour.Triangles.push_back ( triangle ) ;
//         }
//      }
//      //new_walk_neighbour->end() ;
//
//
//      // Define manualObject with navmesh outer edges (boundaries)
//      //new_walk_boundary = new Ogre::ManualObject ( "RecastMOBoundary_" + name ) ;
//      new_walk_boundary.Name = "RecastMOBoundary_" + name ;
//      new_walk_neighbour.RenderType = Ogre::RenderOperation::OT_LINE_LIST ;
//
//      //m_pRecastMOBoundary = m_pSceneMgr->createManualObject("RecastMOBoundary_"+name);
//      //new_walk_boundary->begin("recastdebug", Ogre::RenderOperation::OT_LINE_LIST) ;
//
//      for (int i = 0; i < npolys; ++i)
//      {
//         const unsigned short* p = &polys[i*nvp*2];
//         for (int j = 0; j < nvp; ++j)
//         {
//            if (p[j] == RC_MESH_NULL_IDX) break;
//            if (p[nvp+j] != RC_MESH_NULL_IDX) continue;
//            int vi[2];
//            vi[0] = p[j];
//            if (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX)
//               vi[1] = p[0];
//            else
//               vi[1] = p[j+1];
//
//            std::array<Ogre::Vector3, 3> triangle ;
//
//            triangle [ 2 ] = Ogre::Vector3 ( -1,-1,-1 ) ;
//
//            for (int k = 0; k < 2; ++k)
//            {
//               const unsigned short* v = &verts[vi[k]*3];
//               const float x = orig[0] + v[0]*cs;
//               const float y = orig[1] + (v[1]/*+1*/)*ch /*+ 0.1f*/;
//               const float z = orig[2] + v[2]*cs;
//               //dd->vertex(x, y, z, colb);
//
//               //new_walk_boundary->position(x, y+m_navMeshEdgesOffsetFromGround, z) ;
//               //new_walk_boundary->colour(m_navmeshOuterEdgeCol);
//
//               //new_walk_boundary.push_back ( Ogre::Vector3 ( x, y+m_navMeshEdgesOffsetFromGround, z ) ) ;
//
//               triangle [ k ] = Ogre::Vector3 ( x, y+m_navMeshOffsetFromGround, z ) ;
//            }
//
//            new_walk_boundary.Triangles.push_back ( triangle ) ;
//         }
//      }
//
//      //new_walk_boundary->end() ;
//   }// end areacount
//
//   bool tile_assigned  = false ;
//   bool neigh_assigned = false ;
//   bool bound_assigned = false ;
//
//   for ( int tile_index = 0 ; tile_index < ( signed )DebugTileList.size () ; ++tile_index )
//   {
//      //TODO: Memory leak with old tiles
//
//      if ( new_walk_tile.Triangles.size () > 0 &&
//         DebugTileList [ tile_index ].Name == new_walk_tile.Name )
//      {
//         DebugTileList [ tile_index ] = new_walk_tile ;
//
//         tile_assigned = true ;
//      }
//      else if (new_walk_neighbour.Triangles.size () > 0 &&
//         DebugTileList [ tile_index ].Name == new_walk_neighbour.Name )
//      {
//         DebugTileList [ tile_index ] = new_walk_neighbour ;
//
//         neigh_assigned = true ;
//      }
//      else if ( new_walk_boundary.Triangles.size () > 0 &&
//         DebugTileList [ tile_index ].Name == new_walk_boundary.Name )
//      {
//         DebugTileList [ tile_index ] = new_walk_boundary ;
//
//         bound_assigned = true ;
//      }
//   }
//
//   if ( ! tile_assigned &&
//        new_walk_tile.Triangles.size () > 0 )
//   {
//      DebugTileList.push_back ( new_walk_tile ) ;
//   }
//
//   if ( ! neigh_assigned &&
//        new_walk_neighbour.Triangles.size () > 0 )
//   {
//      DebugTileList.push_back ( new_walk_neighbour ) ;
//   }
//
//   if ( ! bound_assigned &&
//        new_walk_boundary.Triangles.size () > 0 )
//   {
//      DebugTileList.push_back ( new_walk_boundary ) ;
//   }
//}

//float OgreRecast::getAgentRadius()
//{
//    return m_agentRadius;
//}
//
//float OgreRecast::getAgentHeight()
//{
//    return m_agentHeight;
//}

//float OgreRecast::getPathOffsetFromGround()
//{
//    return m_pathOffsetFromGround;
//}

//float OgreRecast::getNavmeshOffsetFromGround()
//{
//    return m_navMeshOffsetFromGround;
//}

//rcConfig OgreRecast::getConfig()
//{
//    return m_cfg;
//}

//const std::vector <DebugTile> &
//OgreRecast::
//GetDebugTileList () const
//{
//   return DebugTileList ;
//}

//rcCompactHeightfield&
//OgreRecast::
//GetCompactHeightField () const
//{
//   return *m_chf ;
//}

//std::vector<Ogre::Vector3> OgreRecast::getManualObjectVertices(Ogre::ManualObject *manual)
//{
//    std::vector<Ogre::Vector3> returnVertices;
//    unsigned long thisSectionStart = 0;
//    for (size_t i=0; i < manual->getNumSections(); i++)
//    {
//        Ogre::ManualObject::ManualObjectSection * section = manual->getSection(i);
//        Ogre::RenderOperation * renderOp = section->getRenderOperation();
//
//        //Collect the vertices
//        {
//            const Ogre::VertexElement * vertexElement = renderOp->vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
//            Ogre::HardwareVertexBufferSharedPtr vertexBuffer = renderOp->vertexData->vertexBufferBinding->getBuffer(vertexElement->getSource());
//
//            char * verticesBuffer = (char*)vertexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY);
//            float * positionArrayHolder;
//
//            thisSectionStart = returnVertices.size();
//
//            returnVertices.reserve(returnVertices.size() + renderOp->vertexData->vertexCount);
//
//            for (unsigned int j=0; j<renderOp->vertexData->vertexCount; j++)
//            {
//                vertexElement->baseVertexPointerToElement(verticesBuffer + j * vertexBuffer->getVertexSize(), &positionArrayHolder);
//                Ogre::Vector3 vertexPos = Ogre::Vector3(positionArrayHolder[0],
//                                                        positionArrayHolder[1],
//                                                        positionArrayHolder[2]);
//
//                //vertexPos = (orient * (vertexPos * scale)) + position;
//
//                returnVertices.push_back(vertexPos);
//            }
//
//            vertexBuffer->unlock();
//        }
//    }
//
//    return returnVertices;
//}

/**
  * Helpers
  **/

void OgreRecast::OgreVect3ToFloatA(const Ogre::Vector3 vect, float* result)
{
    result[0] = vect[0];
    result[1] = vect[1];
    result[2] = vect[2];
};

void OgreRecast::FloatAToOgreVect3(const float* vect, Ogre::Vector3 &result)
{
    result.x = vect[0];
    result.y = vect[1];
    result.z = vect[2];
}

/**
  * Random number generator implementation used by getRandomNavMeshPoint method.
  **/
//static float frand()
//{
//        return (float)rand()/(float)RAND_MAX;
//}

//Ogre::Vector3 OgreRecast::getRandomNavMeshPoint()
//{
//    float resultPoint[3];
//    dtPolyRef resultPoly;
//    m_navQuery->findRandomPoint(mFilter, frand, &resultPoly, resultPoint);
//
//    return Ogre::Vector3(resultPoint[0], resultPoint[1], resultPoint[2]);
//}

//dtQueryFilter OgreRecast::getFilter()
//{
//    return *mFilter;    // Copy-on-return
//}

//void OgreRecast::setFilter(const dtQueryFilter filter)
//{
//    *mFilter = filter;    // Copy
//        // TODO will this work? As I'm making a shallow copy of a class that contains pointers
//}

//Ogre::Vector3 OgreRecast::getPointExtents()
//{
//    Ogre::Vector3 result;
//    FloatAToOgreVect3(mExtents, result);
//    return result;
//}

//void OgreRecast::setPointExtents(Ogre::Vector3 extents)
//{
//    OgreVect3ToFloatA(extents, mExtents);
//}

//Ogre::Vector3 OgreRecast::getRandomNavMeshPointInCircle(Ogre::Vector3 center, Ogre::Real radius)
//{
//    // First find nearest navmesh poly to center
//    float pt[3];
//    OgreVect3ToFloatA(center, pt);
//    float rPt[3];
//    dtPolyRef navmeshPoly;
//    dtStatus status=m_navQuery->findNearestPoly(pt, mExtents, mFilter, &navmeshPoly, rPt);
//    if((status&DT_FAILURE) || (status&DT_STATUS_DETAIL_MASK))
//        return center; // couldn't find a polygon
//
//
//    // Then start searching at this poly for a random point within specified radius
//    dtPolyRef resultPoly;
//    m_navQuery->findRandomPointAroundCircle(navmeshPoly, pt, radius, mFilter, frand, &resultPoly, rPt);
//    Ogre::Vector3 resultPt;
//    FloatAToOgreVect3(rPt, resultPt);
//
//    return resultPt;
//}

//Ogre::String OgreRecast::getPathFindErrorMsg(int errorCode)
//{
//    Ogre::String code = Ogre::StringConverter::toString(errorCode);
//    switch(errorCode) {
//        case 0:
//                return code +" -- No error.";
//        case -1:
//                return code +" -- Couldn't find polygon nearest to start point.";
//        case -2:
//                return code +" -- Couldn't find polygon nearest to end point.";
//        case -3:
//                return code +" -- Couldn't create a path.";
//        case -4:
//                return code +" -- Couldn't find a path.";
//        case -5:
//                return code +" -- Couldn't create a straight path.";
//        case -6:
//                return code +" -- Couldn't find a straight path.";
//        default:
//                return code + " -- Unknown detour error code.";
//    }
//}

bool OgreRecast::findNearestPointOnNavmesh(Ogre::Vector3 position, const unsigned int include_flags, const unsigned int exclude_flags, Ogre::Vector3 &resultPt)
{
    dtPolyRef navmeshPoly;
    return findNearestPolyOnNavmesh(position, include_flags, exclude_flags, resultPt, navmeshPoly);
}

bool OgreRecast::findNearestPolyOnNavmesh(Ogre::Vector3 position, const unsigned int include_flags, const unsigned int exclude_flags, Ogre::Vector3 &resultPt, dtPolyRef &resultPoly)
{
   mFilter->setIncludeFlags ( include_flags ) ;
   mFilter->setExcludeFlags ( exclude_flags ) ;

    float pt[3];
    OgreVect3ToFloatA(position, pt);
    float rPt[3];
    dtStatus status=m_navQuery->findNearestPoly(pt, mExtents, mFilter, &resultPoly, rPt);
    if((status&DT_FAILURE) || (status&DT_STATUS_DETAIL_MASK))
        return false; // couldn't find a polygon

    FloatAToOgreVect3(rPt, resultPt);
    return true;
}

//OgreRecastNavmeshPruner* OgreRecast::getNavmeshPruner()
//{
//    // Store singleton instance
//    if (!mNavmeshPruner)
//        mNavmeshPruner = new OgreRecastNavmeshPruner(this, m_navMesh);
//            // TODO does m_navMesh object not change when navmesh is rebuilt?
//
//    // Return navmesh pruner for this recast mesh
//    return mNavmeshPruner;
//}
