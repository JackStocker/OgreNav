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

#include "ConvexVolume.h"
#include "InputGeom.h"
#include "Recast.h"
#include "OgreRecastDefinitions.h"
#include "OgreRecast.h"

// Returns true if 'a' is more lower-left than 'b'.
bool cmppt(const Real* a, const Real* b)
{
    if (a[0] < b[0]) return true;
    if (a[0] > b[0]) return false;
    if (a[2] < b[2]) return true;
    if (a[2] > b[2]) return false;
    return false;
}

// Returns true if 'c' is left of line 'a'-'b'.
bool left(const Real* a, const Real* b, const Real* c)
{
    const Real u1 = b[0] - a[0];
    const Real v1 = b[2] - a[2];
    const Real u2 = c[0] - a[0];
    const Real v2 = c[2] - a[2];
    return u1 * v2 - v1 * u2 < Real ( 0);
}

ConvexVolume::ConvexVolume(RealVector3 boundingBoxMin, RealVector3 boundingBoxMax, int new_area, Real offset)
{
   RealVector3 max = boundingBoxMax;
    RealVector3 min = boundingBoxMin;

    // Offset bounding box (except height)
    if(offset > Real ( 0.01f)) {
        max = max + offset * RealVector3(1,0,1);
        min = min - offset * RealVector3(1,0,1);
    }

    // Create box verts (in clockwise fashion!!)
    verts[0]= min.x; verts[1]= min.y; verts[2]= max.z;
    verts[3]= max.x; verts[4]= max.y; verts[5]= max.z;
    verts[6]= max.x; verts[7]= max.y; verts[8]= min.z;
    verts[9]= min.x; verts[10]= min.y; verts[11]= min.z;
    nverts = 4; // For rcMarkConvexPoly the verts of the shape need to be in clockwise order

    // Set bounding box limits
    OgreRecast::OgreVect3ToReal(min, bmin);
    OgreRecast::OgreVect3ToReal(max, bmax);

    // Set height limits
    hmin = min.y;
    hmax = max.y;

    area = new_area;   // You can choose whatever flag you assing to the poly area
}

ConvexVolume::
ConvexVolume ( std::vector <RealVector3> verts, int new_area, Real offset )
{
   RealVector3 min ;
   RealVector3 max ;

   // Offset bounding box (except height)
   if ( offset > Real ( 0.01f) )
   {
      max = max + offset * RealVector3 ( 1, 0, 1 ) ;
      min = min - offset * RealVector3 ( 1, 0, 1 ) ;
   }

   // Create box verts (in clockwise fashion!!)
   verts [ 0 ] = min.x ; verts [ 1 ]  = min.y ; verts [ 2 ]  = max.z ;
   verts [ 3 ] = max.x ; verts [ 4 ]  = max.y ; verts [ 5 ]  = max.z ;
   verts [ 6 ] = max.x ; verts [ 7 ]  = max.y ; verts [ 8 ]  = min.z ;
   verts [ 9 ] = min.x ; verts [ 10 ] = min.y ; verts [ 11 ] = min.z ;
   nverts = 4 ; // For rcMarkConvexPoly the verts of the shape need to be in clockwise order

   // Set bounding box limits
   OgreRecast::OgreVect3ToReal ( min, bmin ) ;
   OgreRecast::OgreVect3ToReal ( max, bmax ) ;

   // Set height limits
   hmin = Real (min.y) ;
   hmax = Real (max.y) ;

   area = new_area ; // You can choose whatever flag you assing to the poly area
}
