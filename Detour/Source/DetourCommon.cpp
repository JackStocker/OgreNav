//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include "DetourCommon.h"
#include "DetourMath.h"

//////////////////////////////////////////////////////////////////////////////////////////

void dtClosestPtPointTriangle(Real* closest, const Real* p,
							  const Real* a, const Real* b, const Real* c)
{
	// Check if P in vertex region outside A
	Real ab[3], ac[3], ap[3];
	dtVsub(ab, b, a);
	dtVsub(ac, c, a);
	dtVsub(ap, p, a);
	Real d1 = dtVdot(ab, ap);
	Real d2 = dtVdot(ac, ap);
	if (d1 <= Real ( 0.0f) && d2 <= Real ( 0.0f))
	{
		// barycentric coordinates (1,0,0)
		dtVcopy(closest, a);
		return;
	}
	
	// Check if P in vertex region outside B
	Real bp[3];
	dtVsub(bp, p, b);
	Real d3 = dtVdot(ab, bp);
	Real d4 = dtVdot(ac, bp);
	if (d3 >= Real ( 0.0f) && d4 <= d3)
	{
		// barycentric coordinates (0,1,0)
		dtVcopy(closest, b);
		return;
	}
	
	// Check if P in edge region of AB, if so return projection of P onto AB
	Real vc = d1*d4 - d3*d2;
	if (vc <= Real ( 0.0f) && d1 >= Real ( 0.0f) && d3 <= Real ( 0.0f))
	{
		// barycentric coordinates (1-v,v,0)
		Real v = d1 / (d1 - d3);
		closest[0] = a[0] + v * ab[0];
		closest[1] = a[1] + v * ab[1];
		closest[2] = a[2] + v * ab[2];
		return;
	}
	
	// Check if P in vertex region outside C
	Real cp[3];
	dtVsub(cp, p, c);
	Real d5 = dtVdot(ab, cp);
	Real d6 = dtVdot(ac, cp);
	if (d6 >= Real ( 0.0f) && d5 <= d6)
	{
		// barycentric coordinates (0,0,1)
		dtVcopy(closest, c);
		return;
	}
	
	// Check if P in edge region of AC, if so return projection of P onto AC
	Real vb = d5*d2 - d1*d6;
	if (vb <= Real ( 0.0f) && d2 >= Real ( 0.0f) && d6 <= Real ( 0.0f))
	{
		// barycentric coordinates (1-w,0,w)
		Real w = d2 / (d2 - d6);
		closest[0] = a[0] + w * ac[0];
		closest[1] = a[1] + w * ac[1];
		closest[2] = a[2] + w * ac[2];
		return;
	}
	
	// Check if P in edge region of BC, if so return projection of P onto BC
	Real va = d3*d6 - d5*d4;
	if (va <= Real ( 0.0f) && (d4 - d3) >= Real ( 0.0f) && (d5 - d6) >= Real ( 0.0f))
	{
		// barycentric coordinates (0,1-w,w)
		Real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		closest[0] = b[0] + w * (c[0] - b[0]);
		closest[1] = b[1] + w * (c[1] - b[1]);
		closest[2] = b[2] + w * (c[2] - b[2]);
		return;
	}
	
	// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
	Real denom = Real ( 1.0f) / (va + vb + vc);
	Real v = vb * denom;
	Real w = vc * denom;
	closest[0] = a[0] + ab[0] * v + ac[0] * w;
	closest[1] = a[1] + ab[1] * v + ac[1] * w;
	closest[2] = a[2] + ab[2] * v + ac[2] * w;
}

bool dtIntersectSegmentPoly2D(const Real* p0, const Real* p1,
							  const Real* verts, int nverts,
							  Real& tmin, Real& tmax,
							  int& segMin, int& segMax)
{
	static const Real EPS = Real ( 0.000001f);
	
	tmin = Real(0);
	tmax = Real(1);
	segMin = -1;
	segMax = -1;
	
	Real dir[3];
	dtVsub(dir, p1, p0);
	
	for (int i = 0, j = nverts-1; i < nverts; j=i++)
	{
		Real edge[3], diff[3];
		dtVsub(edge, &verts[i*3], &verts[j*3]);
		dtVsub(diff, p0, &verts[j*3]);
		const Real n = dtVperp2D(edge, diff);
		const Real d = dtVperp2D(dir, edge);
		if (dtAbs(d) < EPS)
		{
			// S is nearly parallel to this edge
			if (n < Real ( 0))
				return false;
			else
				continue;
		}
		const Real t = n / d;
		if (d < Real ( 0))
		{
			// segment S is entering across this edge
			if (t > tmin)
			{
				tmin = t;
				segMin = j;
				// S enters after leaving polygon
				if (tmin > tmax)
					return false;
			}
		}
		else
		{
			// segment S is leaving across this edge
			if (t < tmax)
			{
				tmax = t;
				segMax = j;
				// S leaves before entering polygon
				if (tmax < tmin)
					return false;
			}
		}
	}
	
	return true;
}

Real dtDistancePtSegSqr2D(const Real* pt, const Real* p, const Real* q, Real& t)
{
	Real pqx = q[0] - p[0];
	Real pqz = q[2] - p[2];
	Real dx = pt[0] - p[0];
	Real dz = pt[2] - p[2];
	Real d = pqx*pqx + pqz*pqz;
	t = pqx*dx + pqz*dz;
	if (d > Real(0)) t /= d;
	if (t < Real(0)) t = Real ( 0);
	else if (t > Real ( 1)) t = Real ( 1);
	dx = p[0] + t*pqx - pt[0];
	dz = p[2] + t*pqz - pt[2];
	return dx*dx + dz*dz;
}

void dtCalcPolyCenter(Real* tc, const unsigned short* idx, int nidx, const Real* verts)
{
	tc[0] = Real(0.0f);
	tc[1] = Real(0.0f);
	tc[2] = Real(0.0f);
	for (int j = 0; j < nidx; ++j)
	{
		const Real* v = &verts[idx[j]*3];
		tc[0] += v[0];
		tc[1] += v[1];
		tc[2] += v[2];
	}
	const Real s = Real ( 1.0f) / nidx;
	tc[0] *= s;
	tc[1] *= s;
	tc[2] *= s;
}

bool dtClosestHeightPointTriangle(const Real* p, const Real* a, const Real* b, const Real* c, Real& h)
{
	const Real EPS = Real ( 1e-6f);
	Real v0[3], v1[3], v2[3];

	dtVsub(v0, c, a);
	dtVsub(v1, b, a);
	dtVsub(v2, p, a);

	// Compute scaled barycentric coordinates
	Real denom = v0[0] * v1[2] - v0[2] * v1[0];
	if (dtAbs(denom) < EPS)
		return false;

	Real u = v1[2] * v2[0] - v1[0] * v2[2];
	Real v = v0[0] * v2[2] - v0[2] * v2[0];

	if (denom < Real ( 0)) {
		denom = -denom;
		u = -u;
		v = -v;
	}

	// If point lies inside the triangle, return interpolated ycoord.
	if (u >= Real ( 0.0f) && v >= Real ( 0.0f) && (u + v) <= denom) {
		h = a[1] + (v0[1] * u + v1[1] * v) / denom;
		return true;
	}
	return false;
}

/// @par
///
/// All points are projected onto the xz-plane, so the y-values are ignored.
bool dtPointInPolygon(const Real* pt, const Real* verts, const int nverts)
{
	// TODO: Replace pnpoly with triArea2D tests?
	int i, j;
	bool c = false;
	for (i = 0, j = nverts-1; i < nverts; j = i++)
	{
		const Real* vi = &verts[i*3];
		const Real* vj = &verts[j*3];
		if (((vi[2] > pt[2]) != (vj[2] > pt[2])) &&
			(pt[0] < (vj[0]-vi[0]) * (pt[2]-vi[2]) / (vj[2]-vi[2]) + vi[0]) )
			c = !c;
	}
	return c;
}

bool dtDistancePtPolyEdgesSqr(const Real* pt, const Real* verts, const int nverts,
							  Real* ed, Real* et)
{
	// TODO: Replace pnpoly with triArea2D tests?
	int i, j;
	bool c = false;
	for (i = 0, j = nverts-1; i < nverts; j = i++)
	{
		const Real* vi = &verts[i*3];
		const Real* vj = &verts[j*3];
		if (((vi[2] > pt[2]) != (vj[2] > pt[2])) &&
			(pt[0] < (vj[0]-vi[0]) * (pt[2]-vi[2]) / (vj[2]-vi[2]) + vi[0]) )
			c = !c;
		ed[j] = dtDistancePtSegSqr2D(pt, vj, vi, et[j]);
	}
	return c;
}

static void projectPoly(const Real* axis, const Real* poly, const int npoly,
						Real& rmin, Real& rmax)
{
	rmin = rmax = dtVdot2D(axis, &poly[0]);
	for (int i = 1; i < npoly; ++i)
	{
		const Real d = dtVdot2D(axis, &poly[i*3]);
		rmin = dtMin(rmin, d);
		rmax = dtMax(rmax, d);
	}
}

inline bool overlapRange(const Real amin, const Real amax,
						 const Real bmin, const Real bmax,
						 const Real eps)
{
	return ((amin+eps) > bmax || (amax-eps) < bmin) ? false : true;
}

/// @par
///
/// All vertices are projected onto the xz-plane, so the y-values are ignored.
bool dtOverlapPolyPoly2D(const Real* polya, const int npolya,
						 const Real* polyb, const int npolyb)
{
	const Real eps = Real ( 1e-4f);
	
	for (int i = 0, j = npolya-1; i < npolya; j=i++)
	{
		const Real* va = &polya[j*3];
		const Real* vb = &polya[i*3];
		const Real n[3] = { vb[2]-va[2], Real ( 0), -(vb[0]-va[0]) };
		Real amin,amax,bmin,bmax;
		projectPoly(n, polya, npolya, amin,amax);
		projectPoly(n, polyb, npolyb, bmin,bmax);
		if (!overlapRange(amin,amax, bmin,bmax, eps))
		{
			// Found separating axis
			return false;
		}
	}
	for (int i = 0, j = npolyb-1; i < npolyb; j=i++)
	{
		const Real* va = &polyb[j*3];
		const Real* vb = &polyb[i*3];
		const Real n[3] = { vb[2]-va[2], Real ( 0), -(vb[0]-va[0]) };
		Real amin,amax,bmin,bmax;
		projectPoly(n, polya, npolya, amin,amax);
		projectPoly(n, polyb, npolyb, bmin,bmax);
		if (!overlapRange(amin,amax, bmin,bmax, eps))
		{
			// Found separating axis
			return false;
		}
	}
	return true;
}

// Returns a random point in a convex polygon.
// Adapted from Graphics Gems article.
void dtRandomPointInConvexPoly(const Real* pts, const int npts, Real* areas,
							   const Real s, const Real t, Real* out)
{
	// Calc triangle araes
	Real areasum = Real ( 0.0f);
	for (int i = 2; i < npts; i++) {
		areas[i] = dtTriArea2D(&pts[0], &pts[(i-1)*3], &pts[i*3]);
		areasum += dtMax( Real ( 0.001f), areas[i]);
	}
	// Find sub triangle weighted by area.
	const Real thr = s*areasum;
	Real acc = Real ( 0.0f);
	Real u = Real ( 1.0f);
	int tri = npts - 1;
	for (int i = 2; i < npts; i++) {
		const Real dacc = areas[i];
		if (thr >= acc && thr < (acc+dacc))
		{
			u = (thr - acc) / dacc;
			tri = i;
			break;
		}
		acc += dacc;
	}
	
	Real v = dtMathSqrtf(t);
	
	const Real a = Real(1) - v;
	const Real b = (Real(1) - u) * v;
	const Real c = u * v;
	const Real* pa = &pts[0];
	const Real* pb = &pts[(tri-1)*3];
	const Real* pc = &pts[tri*3];
	
	out[0] = a*pa[0] + b*pb[0] + c*pc[0];
	out[1] = a*pa[1] + b*pb[1] + c*pc[1];
	out[2] = a*pa[2] + b*pb[2] + c*pc[2];
}

inline Real vperpXZ(const Real* a, const Real* b) { return a[0]*b[2] - a[2]*b[0]; }

bool dtIntersectSegSeg2D(const Real* ap, const Real* aq,
						 const Real* bp, const Real* bq,
						 Real& s, Real& t)
{
	Real u[3], v[3], w[3];
	dtVsub(u,aq,ap);
	dtVsub(v,bq,bp);
	dtVsub(w,ap,bp);
	Real d = vperpXZ(u,v);
	if (dtAbs(d) < std::numeric_limits<Real>::epsilon()) return false;
	s = vperpXZ(v,w) / d;
	t = vperpXZ(u,w) / d;
	return true;
}

