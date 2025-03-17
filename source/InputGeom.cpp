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

#include "InputGeom.h"
#include "OgreRecast.h"
#include <OgreStreamSerialiser.h>
#include <cstdio>
#include <OgreItem.h>
#include <OgreMesh2.h>
#include <OgreSubMesh2.h>
#include "OgreBitwise.h"
#include "Vao/OgreAsyncTicket.h"

InputGeom::InputGeom(std::vector<Ogre::ManualObject*> srcMeshes)
    : mSrcMeshes(srcMeshes),
      nverts(0),
      ntris(0),
      mReferenceNode(0),
      bmin(0),
      bmax(0),
      normals(0),
      verts(0),
      tris(0)
{
    if (srcMeshes.empty())
        return;

    // Convert Ogre::Item source meshes to a format that recast understands

    //set the reference node
    auto* ent = srcMeshes[0];
    mReferenceNode = ent->getParentSceneNode()->getCreator()->getRootSceneNode();

    // Set the area where the navigation mesh will be build.
    // Using bounding box of source mesh and specified cell size
    calculateExtents();

    // Convert ogre geometry (vertices, triangles and normals)
    convertOgreEntities();

    // You don't need to build this in single navmesh mode
    buildChunkyTriMesh();
}

void InputGeom::buildChunkyTriMesh()
{
    m_chunkyMesh = std::make_unique <rcChunkyTriMesh> () ;

    if (!m_chunkyMesh)
    {
        Ogre::LogManager::getSingletonPtr()->logMessage("buildTiledNavigation: Out of memory 'm_chunkyMesh'.");
        return;
    }
    if (!rcCreateChunkyTriMesh(getVerts(), getTris(), getTriCount(), 256, m_chunkyMesh.get ()))
    {
        Ogre::LogManager::getSingletonPtr()->logMessage("buildTiledNavigation: Failed to build chunky mesh.");
        return;
    }
}

// TODO make sure I don't forget destructing some members
InputGeom::~InputGeom()
{
    if(verts)
        delete[] verts;
    if(normals)
        delete[] normals;
    if(tris)
        delete[] tris;
    if(bmin)
        delete[] bmin;
    if(bmax)
        delete[] bmax;
}

void InputGeom::convertOgreEntities()
{
    //Convert all vertices and triangles to recast format
    const int numNodes = mSrcMeshes.size();
    size_t *meshVertexCount = new size_t[numNodes];
    size_t *meshIndexCount = new size_t[numNodes];
    Ogre::Vector3 **meshVertices = new Ogre::Vector3*[numNodes];
    Ogre::uint32 **meshIndices = new Ogre::uint32*[numNodes];

    nverts = 0;
    ntris = 0;
    size_t i = 0;
    for(auto iter = mSrcMeshes.begin(); iter != mSrcMeshes.end(); iter++) {
        getManualMeshInformation((*iter), meshVertexCount[i], meshVertices[i], meshIndexCount[i], meshIndices[i]);
        //total number of verts
        nverts += meshVertexCount[i];
        //total number of indices
        ntris += meshIndexCount[i];

        i++;
    }

    // DECLARE RECAST DATA BUFFERS USING THE INFO WE GRABBED ABOVE
    verts = new float[nverts*3];// *3 as verts holds x,y,&z for each verts in the array
    tris = new int[ntris];// tris in recast is really indices like ogre

    //convert index count into tri count
    ntris = ntris/3; //although the tris array are indices the ntris is actual number of triangles, eg. indices/3;

    //copy all meshes verticies into single buffer and transform to world space relative to parentNode
    int vertsIndex = 0;
    int prevVerticiesCount = 0;
    int prevIndexCountTotal = 0;
    i = 0;
    for (auto iter = mSrcMeshes.begin(); iter != mSrcMeshes.end(); iter++) {
        auto *ent = *iter;
        //find the transform between the reference node and this node
        Ogre::Matrix4 transform = mReferenceNode->_getFullTransformUpdated().inverse() * ent->getParentSceneNode()->_getFullTransformUpdated();
        Ogre::Vector3 vertexPos;
        for (size_t j = 0 ; j < meshVertexCount[i] ; j++)
        {
            vertexPos = transform*meshVertices[i][j];
            verts[vertsIndex] = vertexPos.x;
            verts[vertsIndex+1] = vertexPos.y;
            verts[vertsIndex+2] = vertexPos.z;
            vertsIndex+=3;
        }

        for (size_t j = 0 ; j < meshIndexCount[i] ; j++)
        {
            tris[prevIndexCountTotal+j] = meshIndices[i][j]+prevVerticiesCount;
        }
        prevIndexCountTotal += meshIndexCount[i];
        prevVerticiesCount += meshVertexCount[i];

        i++;
    }

    //delete tempory arrays
    //TODO These probably could member varibles, this would increase performance slightly
    delete[] meshVertices;
    delete[] meshIndices;
    delete[] meshVertexCount;
    delete[] meshIndexCount;

    // calculate normals data for Recast - im not 100% sure where this is required
    // but it is used, Ogre handles its own Normal data for rendering, this is not related
    // to Ogre at all ( its also not correct lol )
    // TODO : fix this
    normals = new float[ntris*3];
    for (int i = 0; i < ntris*3; i += 3)
    {
        const float* v0 = &verts[tris[i]*3];
        const float* v1 = &verts[tris[i+1]*3];
        const float* v2 = &verts[tris[i+2]*3];
        float e0[3], e1[3];
        for (int j = 0; j < 3; ++j)
        {
            e0[j] = (v1[j] - v0[j]);
            e1[j] = (v2[j] - v0[j]);
        }
        float* n = &normals[i];
        n[0] = ((e0[1]*e1[2]) - (e0[2]*e1[1]));
        n[1] = ((e0[2]*e1[0]) - (e0[0]*e1[2]));
        n[2] = ((e0[0]*e1[1]) - (e0[1]*e1[0]));

        float d = sqrtf(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
        if (d > 0)
        {
            d = 1.0f/d;
            n[0] *= d;
            n[1] *= d;
            n[2] *= d;
        }
    }
}

void InputGeom::calculateExtents()
{
    auto* ent = mSrcMeshes[0];
    Ogre::Aabb srcMeshBB = ent->getWorldAabbUpdated ();
    Ogre::Matrix4 transform = mReferenceNode->_getFullTransformUpdated ().inverse() * ent->getParentSceneNode()->_getFullTransformUpdated();
    srcMeshBB.transformAffine(transform);
    Ogre::Vector3 min = srcMeshBB.getMinimum();
    Ogre::Vector3 max = srcMeshBB.getMaximum();

    // Calculate min and max from all entities
    for(auto iter = mSrcMeshes.begin(); iter != mSrcMeshes.end(); iter++) {
        auto* ent = *iter;

        //find the transform between the reference node and this node
        transform = mReferenceNode->_getFullTransformUpdated().inverse() * ent->getParentSceneNode()->_getFullTransformUpdated();

        Ogre::Aabb srcMeshBB = ent->getWorldAabbUpdated ();
        srcMeshBB.transformAffine(transform);
        Ogre::Vector3 min2 = srcMeshBB.getMinimum();
        if(min2.x < min.x)
            min.x = min2.x;
        if(min2.y < min.y)
            min.y = min2.y;
        if(min2.z < min.z)
            min.z = min2.z;

        Ogre::Vector3 max2 = srcMeshBB.getMaximum();
        if(max2.x > max.x)
            max.x = max2.x;
        if(max2.y > max.y)
            max.y = max2.y;
        if(max2.z > max.z)
            max.z = max2.z;
    }

    if(!bmin)
        bmin = new float[3];
    if(!bmax)
        bmax = new float[3];
    OgreRecast::OgreVect3ToFloatA(min, bmin);
    OgreRecast::OgreVect3ToFloatA(max, bmax);
}

const float* InputGeom::getMeshBoundsMax() const
{
    return bmax;
}

const float* InputGeom::getMeshBoundsMin() const
{
    return bmin;
}

int InputGeom::getVertCount() const
{
    return nverts;
}

int InputGeom::getTriCount() const
{
    return ntris;
}

const int* InputGeom::getTris() const
{
    return tris;
}

const float* InputGeom::getVerts() const
{
    return verts;
}

const float* InputGeom::getNormals() const
{
   return normals;
}

void InputGeom::getMeshInformation(const Ogre::MeshPtr    mesh,
                                   size_t                 &vertex_count,
                                   Ogre::Vector3*         &vertices,
                                   size_t                 &index_count,
                                   Ogre::uint32*          &indices,
                                   const Ogre::Vector3    &position,
                                   const Ogre::Quaternion &orient,
                                   const Ogre::Vector3    &scale )
{
   // First, we compute the total number of vertices and indices and init the buffers.
   unsigned int numVertices = 0;
   unsigned int numIndices = 0;

   auto subMeshIterator = mesh->getSubMeshes ().begin () ;

   while (subMeshIterator != mesh->getSubMeshes().end())
   {
      Ogre::SubMesh *subMesh = *subMeshIterator;
      numVertices += subMesh->mVao[0][0]->getVertexBuffers()[0]->getNumElements();
      numIndices += subMesh->mVao[0][0]->getIndexBuffer()->getNumElements();

      subMeshIterator++;
   }

   vertices = new Ogre::Vector3[numVertices];
   indices  = new Ogre::uint32[numIndices];

   vertex_count = numVertices ;
   index_count = numIndices ;

   unsigned int addedIndices  = 0 ;
   unsigned int index_offset  = 0 ;
   unsigned int subMeshOffset = 0 ;

   // Read Submeshes
   subMeshIterator = mesh->getSubMeshes ().begin () ;

   while ( subMeshIterator != mesh->getSubMeshes ().end () )
   {
      auto*      subMesh = *subMeshIterator;
      const auto &vaos   = subMesh->mVao[0];

      if (!vaos.empty())
      {
         // Get the first LOD level
         auto* vao       = vaos [ 0 ] ;
         bool  indices32 = ( vao->getIndexBuffer ()->getIndexType () == Ogre::IndexBufferPacked::IT_32BIT ) ;

         const auto &vertexBuffers = vao->getVertexBuffers () ;
         auto*      indexBuffer    = vao->getIndexBuffer () ;

         // Request async read from buffer
         Ogre::VertexArrayObject::ReadRequestsArray requests ;
         requests.push_back ( Ogre::VertexArrayObject::ReadRequests ( Ogre::VES_POSITION ) ) ;
         requests.push_back ( Ogre::VertexArrayObject::ReadRequests ( Ogre::VES_TEXTURE_COORDINATES ) ) ;

         vao->readRequests ( requests ) ;
         vao->mapAsyncTickets ( requests ) ;

         unsigned int subMeshVerticiesNum = requests [ 0 ].vertexBuffer->getNumElements () ;

         if ( requests [ 0 ].type == Ogre::VET_HALF4 )
         {
            for ( size_t i = 0 ; i < subMeshVerticiesNum ; ++i )
            {
               const Ogre::uint16* pos = reinterpret_cast<const Ogre::uint16*>(requests[0].data);

               Ogre::Vector3 vec;
               vec.x = Ogre::Bitwise::halfToFloat(pos[0]);
               vec.y = Ogre::Bitwise::halfToFloat(pos[1]);
               vec.z = Ogre::Bitwise::halfToFloat(pos[2]);

               requests[0].data += requests[0].vertexBuffer->getBytesPerElement();
               vertices[i + subMeshOffset] = (orient * (vec * scale)) + position;
            }
         }
         else if (requests[0].type == Ogre::VET_FLOAT3)
         {
            for (size_t i = 0; i < subMeshVerticiesNum; ++i)
            {
               const float* pos = reinterpret_cast<const float*>(requests[0].data);
               Ogre::Vector3 vec;
               vec.x = *pos++;
               vec.y = *pos++;
               vec.z = *pos++;
               requests[0].data += requests[0].vertexBuffer->getBytesPerElement();
               vertices[i + subMeshOffset] = (orient * (vec * scale)) + position;
            }
         }
         else
         {
            //lprint("Error: Vertex Buffer type not recognised in MeshTools::getMeshInformation");
         }

         //UV COORDINATES
         if (requests[1].type == Ogre::VET_HALF2)
         {
            for (size_t i = 0; i < subMeshVerticiesNum; ++i)
            {
               //const float* pos = reinterpret_cast<const float*>(requests[1].data);
               const Ogre::uint16* pos = reinterpret_cast<const Ogre::uint16*>(requests[1].data);
               Ogre::Vector2 vec;
               vec.x = Ogre::Bitwise::halfToFloat(pos[0]);
               vec.y = Ogre::Bitwise::halfToFloat(pos[1]);
               requests[1].data += requests[1].vertexBuffer->getBytesPerElement();
            }
         }

         subMeshOffset += subMeshVerticiesNum;
         vao->unmapAsyncTickets(requests);

         ////Read index data
         if (indexBuffer)
         {
            Ogre::AsyncTicketPtr asyncTicket = indexBuffer->readRequest(0, indexBuffer->getNumElements());
            unsigned int *pIndices = nullptr;

            if( indices32 )
            {
               pIndices = (unsigned*)(asyncTicket->map());
            }
            else
            {
               unsigned short *pShortIndices = (unsigned short*)(asyncTicket->map());
               pIndices = new unsigned int[indexBuffer->getNumElements()];
               for (size_t k = 0; k < indexBuffer->getNumElements(); k++)
               {
                  pIndices[k] = static_cast<unsigned int>(pShortIndices[k]);
               }
            }

            unsigned int bufferIndex = 0;

            for (size_t i = addedIndices; i < addedIndices + indexBuffer->getNumElements(); i++)
            {
               indices[i] = pIndices[bufferIndex] + index_offset;
               bufferIndex++;
            }

            addedIndices += indexBuffer->getNumElements();

            if (!indices32)
            {
               delete[] pIndices;
            }

            asyncTicket->unmap();
         }

         index_offset += vertexBuffers[0]->getNumElements();
      }

      subMeshIterator++;
   }
}

void InputGeom::getManualMeshInformation(const Ogre::ManualObject *manual,
                                         size_t                   &vertex_count,
                                         Ogre::Vector3*           &vertices,
                                         size_t                   &index_count,
                                         Ogre::uint32*            &indices,
                                         const Ogre::Vector3      &position,
                                         const Ogre::Quaternion   &orient,
                                         const Ogre::Vector3      &scale)
{
   // First, we compute the total number of vertices and indices and init the buffers.
   unsigned int numVertices = 0;
   unsigned int numIndices = 0;

   for ( unsigned int i = 0, size = manual->getNumSections () ; i < size ; ++i )
   {
      auto* section = manual->getSection ( i ) ;

      for ( const auto* vao : section->getVaos ( Ogre::VertexPass::VpNormal ) )
      {
         numVertices += vao->getVertexBuffers()[0]->getNumElements();
         numIndices += vao->getIndexBuffer()->getNumElements();
      }
   }

   vertices = new Ogre::Vector3[numVertices];
   indices  = new Ogre::uint32[numIndices];

   vertex_count = numVertices ;
   index_count = numIndices ;

   unsigned int addedIndices  = 0 ;
   unsigned int index_offset  = 0 ;
   unsigned int subMeshOffset = 0 ;

   // Read Submeshes
   for ( unsigned int i = 0, size = manual->getNumSections () ; i < size ; ++i )
   {
      auto* section = manual->getSection ( i ) ;

      for ( auto* vao : section->getVaos ( Ogre::VertexPass::VpNormal ) )
      {
         // Get the first LOD level
         bool  indices32 = ( vao->getIndexBuffer ()->getIndexType () == Ogre::IndexBufferPacked::IT_32BIT ) ;

         const auto &vertexBuffers = vao->getVertexBuffers () ;
         auto*      indexBuffer    = vao->getIndexBuffer () ;

         // Request async read from buffer
         Ogre::VertexArrayObject::ReadRequestsArray requests ;
         requests.push_back ( Ogre::VertexArrayObject::ReadRequests ( Ogre::VES_POSITION ) ) ;
         requests.push_back ( Ogre::VertexArrayObject::ReadRequests ( Ogre::VES_TEXTURE_COORDINATES ) ) ;

         vao->readRequests ( requests ) ;
         vao->mapAsyncTickets ( requests ) ;

         unsigned int subMeshVerticiesNum = requests [ 0 ].vertexBuffer->getNumElements () ;

         if ( requests [ 0 ].type == Ogre::VET_HALF4 )
         {
            for ( size_t i = 0 ; i < subMeshVerticiesNum ; ++i )
            {
               const Ogre::uint16* pos = reinterpret_cast<const Ogre::uint16*>(requests[0].data);

               Ogre::Vector3 vec;
               vec.x = Ogre::Bitwise::halfToFloat(pos[0]);
               vec.y = Ogre::Bitwise::halfToFloat(pos[1]);
               vec.z = Ogre::Bitwise::halfToFloat(pos[2]);

               requests[0].data += requests[0].vertexBuffer->getBytesPerElement();
               vertices[i + subMeshOffset] = (orient * (vec * scale)) + position;
            }
         }
         else if (requests[0].type == Ogre::VET_FLOAT3)
         {
            for (size_t i = 0; i < subMeshVerticiesNum; ++i)
            {
               const float* pos = reinterpret_cast<const float*>(requests[0].data);
               Ogre::Vector3 vec;
               vec.x = *pos++;
               vec.y = *pos++;
               vec.z = *pos++;
               requests[0].data += requests[0].vertexBuffer->getBytesPerElement();
               vertices[i + subMeshOffset] = (orient * (vec * scale)) + position;
            }
         }
         else
         {
            //lprint("Error: Vertex Buffer type not recognised in MeshTools::getMeshInformation");
         }

         //UV COORDINATES
         if (requests[1].type == Ogre::VET_HALF2)
         {
            for (size_t i = 0; i < subMeshVerticiesNum; ++i)
            {
               //const float* pos = reinterpret_cast<const float*>(requests[1].data);
               const Ogre::uint16* pos = reinterpret_cast<const Ogre::uint16*>(requests[1].data);
               Ogre::Vector2 vec;
               vec.x = Ogre::Bitwise::halfToFloat(pos[0]);
               vec.y = Ogre::Bitwise::halfToFloat(pos[1]);
               requests[1].data += requests[1].vertexBuffer->getBytesPerElement();
            }
         }

         subMeshOffset += subMeshVerticiesNum;
         vao->unmapAsyncTickets(requests);

         ////Read index data
         if (indexBuffer)
         {
            Ogre::AsyncTicketPtr asyncTicket = indexBuffer->readRequest(0, indexBuffer->getNumElements());
            unsigned int *pIndices = nullptr;

            if( indices32 )
            {
               pIndices = (unsigned*)(asyncTicket->map());
            }
            else
            {
               unsigned short *pShortIndices = (unsigned short*)(asyncTicket->map());
               pIndices = new unsigned int[indexBuffer->getNumElements()];
               for (size_t k = 0; k < indexBuffer->getNumElements(); k++)
               {
                  pIndices[k] = static_cast<unsigned int>(pShortIndices[k]);
               }
            }

            unsigned int bufferIndex = 0;

            for (size_t i = addedIndices; i < addedIndices + indexBuffer->getNumElements(); i++)
            {
               indices[i] = pIndices[bufferIndex] + index_offset;
               bufferIndex++;
            }

            addedIndices += indexBuffer->getNumElements();

            if (!indices32)
            {
               delete[] pIndices;
            }

            asyncTicket->unmap();
         }

         index_offset += vertexBuffers[0]->getNumElements();
      }
   }

   /*
    std::vector<Ogre::Vector3> returnVertices;
    std::vector<unsigned long> returnIndices;
    unsigned long thisSectionStart = 0;
    for (size_t i=0; i < manual->getNumSections(); i++)
    {
        Ogre::v1::ManualObject::ManualObjectSection * section = manual->getSection(i);
        Ogre::v1::RenderOperation renderOp;

        section->getRenderOperation(renderOp, false);

        std::vector<Ogre::Vector3> pushVertices;
        //Collect the vertices
        {
            const Ogre::v1::VertexElement * vertexElement = renderOp.vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
            Ogre::v1::HardwareVertexBufferSharedPtr vertexBuffer = renderOp.vertexData->vertexBufferBinding->getBuffer(vertexElement->getSource());

            char * verticesBuffer = (char*)vertexBuffer->lock(Ogre::v1::HardwareBuffer::HBL_READ_ONLY);
            float * positionArrayHolder;

            thisSectionStart = pushVertices.size();

            pushVertices.reserve(renderOp.vertexData->vertexCount);

            for (unsigned int j=0; j<renderOp.vertexData->vertexCount; j++)
            {
                vertexElement->baseVertexPointerToElement(verticesBuffer + j * vertexBuffer->getVertexSize(), &positionArrayHolder);
                Ogre::Vector3 vertexPos = Ogre::Vector3(positionArrayHolder[0],
                                                        positionArrayHolder[1],
                                                        positionArrayHolder[2]);

                vertexPos = (orient * (vertexPos * scale)) + position;

                pushVertices.push_back(vertexPos);
            }

            vertexBuffer->unlock();
        }
        //Collect the indices
        {
            if (renderOp.useIndexes)
            {
                Ogre::v1::HardwareIndexBufferSharedPtr indexBuffer = renderOp.indexData->indexBuffer;

                if (! indexBuffer || renderOp.operationType != Ogre::OperationType::OT_TRIANGLE_LIST)
                {
                    //No triangles here, so we just drop the collected vertices and move along to the next section.
                    continue;
                }
                else
                {
                    returnVertices.reserve(returnVertices.size() + pushVertices.size());
                    returnVertices.insert(returnVertices.end(), pushVertices.begin(), pushVertices.end());
                }

                unsigned int * pLong = (unsigned int*)indexBuffer->lock(Ogre::v1::HardwareBuffer::HBL_READ_ONLY);
                unsigned short * pShort = (unsigned short*)pLong;

                returnIndices.reserve(returnIndices.size() + renderOp.indexData->indexCount);

                for (size_t j = 0; j < renderOp.indexData->indexCount; j++)
                {
                    unsigned long index;
                    //We also have got to remember that for a multi section object, each section has
                    //different vertices, so the indices will not be correct. To correct this, we
                    //have to add the position of the first vertex in this section to the index

                    //(At least I think so...)
                    if (indexBuffer->getType() == Ogre::v1::HardwareIndexBuffer::IT_32BIT)
                        index = (unsigned long)pLong[j] + thisSectionStart;
                    else
                        index = (unsigned long)pShort[j] + thisSectionStart;

                    returnIndices.push_back(index);
                }

                indexBuffer->unlock();
            }
        }
    }

    //Now we simply return the data.
    index_count = returnIndices.size();
    vertex_count = returnVertices.size();
    vertices = new Ogre::Vector3[vertex_count];
    for (unsigned long i = 0; i<vertex_count; i++)
    {
        vertices[i] = returnVertices[i];
    }
    indices = new unsigned long[index_count];
    for (unsigned long i = 0; i<index_count; i++)
    {
        indices[i] = returnIndices[i];
    }

    //All done.
    return;*/
}

// ChunkyTriMesh

struct BoundsItem
{
    float bmin[2];
    float bmax[2];
    int i;
};

static int compareItemX(const void* va, const void* vb)
{
    const BoundsItem* a = (const BoundsItem*)va;
    const BoundsItem* b = (const BoundsItem*)vb;
    if (a->bmin[0] < b->bmin[0])
        return -1;
    if (a->bmin[0] > b->bmin[0])
        return 1;
    return 0;
}

static int compareItemY(const void* va, const void* vb)
{
    const BoundsItem* a = (const BoundsItem*)va;
    const BoundsItem* b = (const BoundsItem*)vb;
    if (a->bmin[1] < b->bmin[1])
        return -1;
    if (a->bmin[1] > b->bmin[1])
        return 1;
    return 0;
}

static void calcExtends(const BoundsItem* items, const int /*nitems*/,
                        const int imin, const int imax,
                        float* bmin, float* bmax)
{
    bmin[0] = items[imin].bmin[0];
    bmin[1] = items[imin].bmin[1];

    bmax[0] = items[imin].bmax[0];
    bmax[1] = items[imin].bmax[1];

    for (int i = imin+1; i < imax; ++i)
    {
        const BoundsItem& it = items[i];
        if (it.bmin[0] < bmin[0]) bmin[0] = it.bmin[0];
        if (it.bmin[1] < bmin[1]) bmin[1] = it.bmin[1];

        if (it.bmax[0] > bmax[0]) bmax[0] = it.bmax[0];
        if (it.bmax[1] > bmax[1]) bmax[1] = it.bmax[1];
    }
}

inline int longestAxis(float x, float y)
{
    return y > x ? 1 : 0;
}

static void subdivide(BoundsItem* items, int nitems, int imin, int imax, int trisPerChunk,
                      int& curNode, rcChunkyTriMeshNode* nodes, const int maxNodes,
                      int& curTri, int* outTris, const int* inTris)
{
    int inum = imax - imin;
    int icur = curNode;

    if (curNode > maxNodes)
        return;

    rcChunkyTriMeshNode& node = nodes[curNode++];

    if (inum <= trisPerChunk)
    {
        // Leaf
        calcExtends(items, nitems, imin, imax, node.bmin, node.bmax);

        // Copy triangles.
        node.i = curTri;
        node.n = inum;

        for (int i = imin; i < imax; ++i)
        {
            const int* src = &inTris[items[i].i*3];
            int* dst = &outTris[curTri*3];
            curTri++;
            dst[0] = src[0];
            dst[1] = src[1];
            dst[2] = src[2];
        }
    }
    else
    {
        // Split
        calcExtends(items, nitems, imin, imax, node.bmin, node.bmax);

        int	axis = longestAxis(node.bmax[0] - node.bmin[0],
                                   node.bmax[1] - node.bmin[1]);

        if (axis == 0)
        {
            // Sort along x-axis
            qsort(items+imin, inum, sizeof(BoundsItem), compareItemX);
        }
        else if (axis == 1)
        {
            // Sort along y-axis
            qsort(items+imin, inum, sizeof(BoundsItem), compareItemY);
        }

        int isplit = imin+inum/2;

        // Left
        subdivide(items, nitems, imin, isplit, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);
        // Right
        subdivide(items, nitems, isplit, imax, trisPerChunk, curNode, nodes, maxNodes, curTri, outTris, inTris);

        int iescape = curNode - icur;
        // Negative index means escape.
        node.i = -iescape;
    }
}

bool rcCreateChunkyTriMesh(const float* verts, const int* tris, int ntris,
                           int trisPerChunk, rcChunkyTriMesh* cm)
{
    int nchunks = (ntris + trisPerChunk-1) / trisPerChunk;

    cm->nodes = new rcChunkyTriMeshNode[nchunks*4];
    if (!cm->nodes)
        return false;

    cm->tris = new int[ntris*3];
    if (!cm->tris)
        return false;

    cm->ntris = ntris;

    // Build tree
    BoundsItem* items = new BoundsItem[ntris];
    if (!items)
        return false;

    for (int i = 0; i < ntris; i++)
    {
        const int* t = &tris[i*3];
        BoundsItem& it = items[i];
        it.i = i;
        // Calc triangle XZ bounds.
        it.bmin[0] = it.bmax[0] = verts[t[0]*3+0];
        it.bmin[1] = it.bmax[1] = verts[t[0]*3+2];
        for (int j = 1; j < 3; ++j)
        {
            const float* v = &verts[t[j]*3];
            if (v[0] < it.bmin[0]) it.bmin[0] = v[0];
            if (v[2] < it.bmin[1]) it.bmin[1] = v[2];

            if (v[0] > it.bmax[0]) it.bmax[0] = v[0];
            if (v[2] > it.bmax[1]) it.bmax[1] = v[2];
        }
    }

    int curTri = 0;
    int curNode = 0;
    subdivide(items, ntris, 0, ntris, trisPerChunk, curNode, cm->nodes, nchunks*4, curTri, cm->tris, tris);

    delete [] items;

    cm->nnodes = curNode;

    // Calc max tris per node.
    cm->maxTrisPerChunk = 0;
    for (int i = 0; i < cm->nnodes; ++i)
    {
        rcChunkyTriMeshNode& node = cm->nodes[i];
        const bool isLeaf = node.i >= 0;
        if (!isLeaf) continue;
        if (node.n > cm->maxTrisPerChunk)
            cm->maxTrisPerChunk = node.n;
    }

    return true;
}


inline bool checkOverlapRect(const float amin[2], const float amax[2],
                             const float bmin[2], const float bmax[2])
{
    bool overlap = true;
    overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
    overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
    return overlap;
}

int rcGetChunksOverlappingRect(const rcChunkyTriMesh* cm,
                               float bmin[2], float bmax[2],
                               int* ids, const int maxIds)
{
    // Traverse tree
    int i = 0;
    int n = 0;
    while (i < cm->nnodes)
    {
        const rcChunkyTriMeshNode* node = &cm->nodes[i];
        const bool overlap = checkOverlapRect(bmin, bmax, node->bmin, node->bmax);
        const bool isLeafNode = node->i >= 0;

        if (isLeafNode && overlap)
        {
            if (n < maxIds)
            {
                ids[n] = i;
                n++;
            }
        }

        if (overlap || isLeafNode)
            i++;
        else
        {
            const int escapeIndex = -node->i;
            i += escapeIndex;
        }
    }

    return n;
}
