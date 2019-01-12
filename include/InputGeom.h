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

#ifndef RECASTINPUTGEOM_H
#define RECASTINPUTGEOM_H

#include <Ogre.h>
#include <OgreTerrain.h>
#include <OgreTerrainGroup.h>
#include "ConvexVolume.h"


/**
  * One node or chunk of the chunky tri mesh.
  * Contains a 2D xz plane bounding box.
  * n is the number of tris contained in this chunk.
  * i is the starting index of the tris contained in this chunk.
  * The actual tris are in the chunkyMesh object itself, in linear
  * order per node, so that each node only needs the begin position
  * and tri count to reference its tris.
  **/
struct rcChunkyTriMeshNode
{
        float bmin[2], bmax[2];
        int i, n;
};

/**
  * Spatial subdivision structure that structures triangles
  * in axis-aligned boxes of a fixed size.
  * This allows to quickly retrieve the triangles in a specific box,
  * at the cost of a small pre-process step and extra memory usage.
  **/
struct rcChunkyTriMesh
{
        inline rcChunkyTriMesh() : nodes(0), tris(0) {};
        inline ~rcChunkyTriMesh() { if(nodes) delete [] nodes; if(tris) delete [] tris; }

        rcChunkyTriMeshNode* nodes;
        int nnodes;
        int* tris;
        int ntris;
        int maxTrisPerChunk;
};

/// Creates partitioned triangle mesh (AABB tree),
/// where each node contains at max trisPerChunk triangles.
bool rcCreateChunkyTriMesh(const float* verts, const int* tris, int ntris,
                                                   int trisPerChunk, rcChunkyTriMesh* cm);

/// Returns the chunk indices which overlap the input rectable.
int rcGetChunksOverlappingRect(const rcChunkyTriMesh* cm, float bmin[2], float bmax[2], int* ids, const int maxIds);

/**
  * Helper class to manage input geometry used as input for recast
  * to build a navmesh.
  * Adds extra features such as creating off-mesh connections between
  * points in the geometry, raycasting to polygon level and bounding box
  * intersection tests.
  * It also allows to add or remove temporary obstacles to the geometry and
  * add convex shapes as extra obstacles.
  *
  * This class handles the conversion of Ogre::Entities to recast compatible
  * input format.
  **/
class InputGeom
{
public:
    /**
      * Create recast compatible inputgeom from the specified entities. The entities have to be added to the
      * scene before this call, as we need to calculate the world coordinates of the entity.
      * Vertices and faces of the specified source entities are stored in this inputGeom, individual entity
      * grouping and origin points are lost.
      **/
    InputGeom(std::vector<Ogre::Entity*> srcMeshes);
    ~InputGeom();

    /**
      * Retrieves the vertices stored within this inputGeom. The verts are an array of floats in which each
      * subsequent three floats are in order the x, y and z coordinates of a vert. The size of this array is
      * always a multiple of three and is exactly 3*getVertCount().
      **/
    const float* getVerts(void) const;

    /**
      * The number of vertices stored in this inputGeom.
      **/
    int getVertCount(void) const;

    /**
      * Retrieves the tris stored in this inputGeom.
      * A tri is defined by a sequence of three indexes which refer to an index position in the getVerts() array.
      * Similar to getVerts, the size of this array is a multitude of 3 and is exactly 3*getTriCount().
      **/
    const int* getTris(void) const;

    /**
      * The number of triangles stored in this inputGeom.
      **/
    int getTriCount(void) const;

    /**
      * Retrieve the normals calculated for this inputGeom. Note that the normals are not exact and are not meant for rendering,
      * but they are good enough for navmesh calculation. Each normal corresponds to one vertex from getVerts() with the same index.
      * The size of the normals array is 3*getVertCount().
      **/
    const float* getNormals(void) const;

    /**
      * The axis aligned bounding box minimum of this input Geom.
      **/
    const float* getMeshBoundsMin(void) const;

    /**
      * The axis aligned bounding box maximum of this input Geom.
      **/
    const float* getMeshBoundsMax(void) const;

    /**
      * Retrieve vertex data from a mesh
      * From http://www.ogre3d.org/tikiwiki/RetrieveVertexData
      *
      * This example is taken from monster's OgreODE project. The full source can be found under ogreaddons/ogreode in the Ogre SVN.
      * It has been adopted, so that it can be used separately. Just copy/paste it into your own project.
      *
      * Note that this code assumes sizeof(long) == sizeof(uint32_t), which is not true on AMD64 Linux.
     **/
    static void getMeshInformation(const Ogre::MeshPtr mesh,
                            size_t &vertex_count,
                            Ogre::Vector3* &vertices,
                            size_t &index_count,
                            unsigned long* &indices,
                            const Ogre::Vector3 &position = Ogre::Vector3::ZERO,
                            const Ogre::Quaternion &orient = Ogre::Quaternion::IDENTITY,
                            const Ogre::Vector3 &scale = Ogre::Vector3::UNIT_SCALE);

    /**
      * getMeshInformation for manual meshes.
      **/
    static void getManualMeshInformation(const Ogre::ManualObject *manual,
                            size_t &vertex_count,
                            Ogre::Vector3* &vertices,
                            size_t &index_count,
                            unsigned long* &indices,
                            const Ogre::Vector3 &position = Ogre::Vector3::ZERO,
                            const Ogre::Quaternion &orient = Ogre::Quaternion::IDENTITY,
                            const Ogre::Vector3 &scale = Ogre::Vector3::UNIT_SCALE);

    /**
      * The chunky tri mesh generated for this inputGeom.
      * Chunky tri meshes are only used when building tiled navmeshes, and are not essential,
      * just more optimized.
      **/
    inline const rcChunkyTriMesh* getChunkyMesh() const { return m_chunkyMesh; }

    /**
      * Raycast this inputGeometry.
      **/
    //bool raycastMesh(float* src, float* dst, float& tmin);

    /**
      * See OgreDetourTileCache::hitTestObstacle, but here it serves for
      * finding convexVolumes.
      **/
    //int hitTestConvexVolume(const float* sp, const float* sq);

    /**
      * Retrieve the convex volume obstacle with specified index from this inputGeom.
      **/
    ConvexVolume* getConvexVolume(int volIdx);

    /// @name Box Volumes.
    ///@{
    int getConvexVolumeCount() const { return m_volumeCount; }
    const ConvexVolume* const* getConvexVolumes() const { return m_volumes; }
    int addConvexVolume(ConvexVolume *vol);
    bool deleteConvexVolume(int i, ConvexVolume** = NULL);
    // Not implemented
    void drawConvexVolumes(struct duDebugDraw* dd, bool hilight = false);
    int getConvexVolumeId(ConvexVolume *convexHull);
    ///@}

private:
    /**
      * Calculate max and min bounds of this geometry.
      **/
    void calculateExtents(void);

    /**
      * Build chunky tri mesh.
      * Only needed for building tiled navmeshes.
      **/
    void buildChunkyTriMesh(void);

    /**
      * Convert triangles and verticies of ogre entities
      * to recast internal geometry format.
      **/
    void convertOgreEntities(void);

    /**
      * Recast input vertices
      **/
    float* verts;

    /**
      * Number of verts
      * The actual size of the verts array is 3*nverts
      **/
    int nverts;

    /**
      * Recast input tris
      * Tris are index references to verts array
      **/
    int* tris;

    /**
      * The number of tris
      * The actual size of the tris array is 3*ntris
      **/
    int ntris;

    /**
      * Normals calculated for verts
      * Normals are not entirely accurate but good enough for recast use.
      * Size of the normals array is 3*nverts
      **/
    float* normals;

    /**
      * Axis aligned bounding box of this inputGeom minimum.
      **/
    float* bmin;

    /**
      * Axis aligned bounding box of this inputGeom maximum.
      **/
    float* bmax;

    /**
      * Ogre entities this inputGeom was constructed from.
      **/
    std::vector<Ogre::Entity*> mSrcMeshes;

    /**
      * Reference node to which the absolute coordinates of the verts in this inputGeom was calculated.
      * Is usually the scene rootnode.
      **/
    Ogre::SceneNode *mReferenceNode;

    /**
      * Optimized structures that stores triangles in axis aligned boxes of uniform size
      * (tiles). Allows quick access to a part of the geometry, but requires more memory to store.
      **/
    rcChunkyTriMesh *m_chunkyMesh;

    /**
      * Maximum number of convex volume obstacles that can be added to this inputGeom.
      **/
    static const int MAX_VOLUMES = 1024;

    /// @name Convex Volumes (temporary) added to this geometry.
    ConvexVolume* m_volumes[MAX_VOLUMES];
    int m_volumeCount;

};

#endif // RECASTINPUTGEOM_H
