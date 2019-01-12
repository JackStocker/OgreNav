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
#include "InputGeom.h"
#include "DetourTileCacheBuilder.h"
#include "PlayerFlagQueryFilter.h"

#include <math.h>

OgreRecast::
OgreRecast ( OgreRecastConfigParams configParams )
    : mFilter(0),
      m_ctx(0)
{
   // Init recast stuff in a safe state
   m_navMesh=NULL;
   m_navQuery=NULL;
   m_ctx=NULL ;

   RecastCleanup() ; // TODO ?? don't know if I should do this prior to making any recast stuff, but the demo did.

   // Set default size of box around points to look for nav polygons
   mExtents[0] = 32.0f;
   mExtents[1] = 32.0f;
   mExtents[2] = 32.0f;

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
}

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

/**
  * Helpers
  **/
void OgreRecast::OgreVect3ToFloatA(const Ogre::Vector3 vect, float* result)
{
    result[0] = vect[0];
    result[1] = vect[1];
    result[2] = vect[2];
}

void OgreRecast::FloatAToOgreVect3(const float* vect, Ogre::Vector3 &result)
{
    result.x = vect[0];
    result.y = vect[1];
    result.z = vect[2];
}

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
