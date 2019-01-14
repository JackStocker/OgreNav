#pragma once

/**
  * Configuration parameters for recast navmesh building.
  * A lot of the descripions of the parameters are not mine but come from the very
  * useful CritterAI page (http://www.critterai.org/nmgen_config).
  * For more detail and pictures, have a look there.
  * Some other descriptions come from stevesp's doxygen API docs for recast itself
  * (http://www.stevefsp.org/projects/rcndoc/prod/structrcConfig.html).
  *
  * Some settings are derived from easier to set parameters, those are denoted by
  * the _ that follows their setter and getter. The easier to set parameters correspond
  * to the settings available in the demo that comes with the recast library.
  * You can overwrite those values by calling the setters preceeded with _.
  * Otherwise it suffices to set a value for each setter without a preceding _.
  **/
class OgreRecastConfigParams
{
public:
    /**
      * Initialize some default recast parameters
      **/
    OgreRecastConfigParams(void)
        : cellSize(0.3f),
          cellHeight(0.2f),
          agentMaxSlope(20.0f),
          agentHeight(2.5f),
          agentMaxClimb(1.0f),
          agentRadius(0.0f),//0.5),
          edgeMaxLen(12.0f),
          edgeMaxError(1.3f),
          regionMinSize(50.0f),
          regionMergeSize(20.0f),
          vertsPerPoly(DT_VERTS_PER_POLYGON),   // (=6)
          detailSampleDist(6.0f),
          detailSampleMaxError(1.0f),
          keepInterResults(false)
    { eval(); }


///////////////////////
// GUI SETTINGS:
///////////////////////

    /*****************
      * Rasterization
     *****************/
    /**
      * @see{cellSize}
      **/
    inline void setCellSize(Ogre::Real cellSize) { this->cellSize = cellSize; eval(); }
    /**
      * @see{cellHeight}
      **/
    inline void setCellHeight(Ogre::Real cellHeight) { this->cellHeight = cellHeight; eval(); }
    /*****************
      * Agent
     *****************/
    /**
      * @see{agentHeight}
      **/
    inline void setAgentHeight(Ogre::Real agentHeight) { this->agentHeight = agentHeight; eval(); }
    /**
      * @see{agentRadius}
      **/
    inline void setAgentRadius(Ogre::Real agentRadius) { this->agentRadius = agentRadius; eval(); }
    /**
      * @see{agentMaxClimb}
      **/
    inline void setAgentMaxClimb(Ogre::Real agentMaxClimb) { this->agentMaxClimb = agentMaxClimb; eval(); }
    /**
      * @see{agentMaxSlope}
      **/
    inline void setAgentMaxSlope(Ogre::Real agentMaxSlope) { this->agentMaxSlope = agentMaxSlope; }
    /*****************
      * Region
     *****************/
    /**
      * @see{regionMinSize}
      **/
    inline void setRegionMinSize(Ogre::Real regionMinSize) { this->regionMinSize = regionMinSize; eval(); }
    /**
      * @see{regionMergeSize}
      **/
    inline void setRegionMergeSize(Ogre::Real regionMergeSize) { this->regionMergeSize = regionMergeSize; eval(); }
// TODO Add "monotone partitioning" option to call rcBuildRegionsMonotone in single navmesh building.

    /*****************
      * Polygonization
     *****************/
    /**
      * @see{edgeMaxLen}
      **/
    inline void setEdgeMaxLen(Ogre::Real edgeMaxLength) { this->edgeMaxLen = edgeMaxLength; eval(); }
    /**
      * @see{edgeMaxError}
      **/
    inline void setEdgeMaxError(Ogre::Real edgeMaxError) { this->edgeMaxError = edgeMaxError;}
    /**
      * @see{vertsPerPoly}
      **/
    inline void setVertsPerPoly(int vertsPerPoly) { this->vertsPerPoly = vertsPerPoly; }
    /*****************
      * Detail mesh
     *****************/
    /**
      * @see{detailSampleDist}
      **/
    inline void setDetailSampleDist(Ogre::Real detailSampleDist) { this->detailSampleDist = detailSampleDist; eval(); }
    /**
      * @see{detailSampleMaxError}
      **/
    inline void setDetailSampleMaxError(Ogre::Real detailSampleMaxError) { this->detailSampleMaxError = detailSampleMaxError; eval(); }

    /**
      * @see{keepInterResults}
      **/
    inline void setKeepInterResults(bool keepInterResults) { this->keepInterResults = keepInterResults; }

    /**
      * @see{_walkableHeight}
      **/
    inline void _setWalkableHeight(int walkableHeight) { this->_walkableHeight = walkableHeight; }
    /**
      * @see{_walkableClimb}
      **/
    inline void _setWalkableClimb(int walkableClimb) { this->_walkableClimb = walkableClimb; }
    /**
      * @see{_walkableRadius}
      **/
    inline void _setWalkableRadius(int walkableRadius) { this->_walkableRadius = walkableRadius; }
    /**
      * @see{_maxEdgeLen}
      **/
    inline void _setMaxEdgeLen(int maxEdgeLen) { this->_maxEdgeLen = maxEdgeLen; }
    /**
      * @see{_minRegionArea}
      **/
    inline void _setMinRegionArea(int minRegionArea) { this->_minRegionArea = minRegionArea; }
    /**
      * @see{_mergeRegionArea}
      **/
    inline void _setMergeRegionArea(int mergeRegionArea) { this->_mergeRegionArea = mergeRegionArea; }
    /**
      * @see{_detailSampleDist}
      **/
    inline void _setDetailSampleDist(Ogre::Real detailSampleDist) { this->_detailSampleDist = detailSampleDist; }
    /**
      * @see{_detailSampleMaxError}
      **/
    inline void _setDetailSampleMaxError(Ogre::Real detailSampleMaxError) { this->_detailSampleMaxError = detailSampleMaxError; }



    /**
      * @see{cellSize}
      **/
    inline Ogre::Real getCellSize(void) const { return cellSize; }
    /**
      * @see{cellHeight}
      **/
    inline Ogre::Real getCellHeight(void) const { return cellHeight; }
    /**
      * @see{agentMaxSlope}
      **/
    inline Ogre::Real getAgentMaxSlope(void) const { return agentMaxSlope; }
    /**
      * @see{agentHeight}
      **/
    inline Ogre::Real getAgentHeight(void) const { return agentHeight; }
    /**
      * @see{agentMaxClimb}
      **/
    inline Ogre::Real getAgentMaxClimb(void) const { return agentMaxClimb; }
    /**
      * @see{agentRadius}
      **/
    inline Ogre::Real getAgentRadius(void) const { return agentRadius; }
    /**
      * @see{edgeMaxLen}
      **/
    inline Ogre::Real getEdgeMaxLen(void) const { return edgeMaxLen; }
    /**
      * @see{edgeMaxError}
      **/
    inline Ogre::Real getEdgeMaxError(void) const { return edgeMaxError; }
    /**
      * @see{regionMinSize}
      **/
    inline Ogre::Real getRegionMinSize(void) const { return regionMinSize; }
    /**
      * @see{regionMergeSize}
      **/
    inline Ogre::Real getRegionMergeSize(void) const { return regionMergeSize; }
    /**
      * @see{vertsPerPoly}
      **/
    inline int getVertsPerPoly(void) const { return vertsPerPoly; }
    /**
      * @see{detailSampleDist}
      **/
    inline Ogre::Real getDetailSampleDist(void) const { return detailSampleDist; }
    /**
      * @see{detailSampleMaxError}
      **/
    inline Ogre::Real getDetailSampleMaxError(void) const { return detailSampleMaxError; }

    /**
      * @see{keepInterResults}
      **/
    inline bool getKeepInterResults(void) const { return keepInterResults; }

    /**
      * @see{_walkableHeight}
      **/
    inline int _getWalkableheight(void) const { return _walkableHeight; }
    /**
      * @see{_walkableClimb}
      **/
    inline int _getWalkableClimb(void) const { return _walkableClimb; }
    /**
      * @see{_walkableRadius}
      **/
    inline int _getWalkableRadius(void) const { return _walkableRadius; }
    /**
      * @see{_maxEdgeLen}
      **/
    inline int _getMaxEdgeLen(void) const { return _maxEdgeLen; }
    /**
      * @see{_minRegionArea}
      **/
    inline int _getMinRegionArea(void) const { return _minRegionArea; }
    /**
      * @see{_mergeRegionArea}
      **/
    inline int _getMergeRegionArea(void) const { return _mergeRegionArea; }
    /**
      * @see{_detailSampleDist}
      **/
    inline int _getDetailSampleDist(void) const { return (int)_detailSampleDist; }
    /**
      * @see{detailSampleMaxError}
      **/
    inline int _getDetailSampleMaxError(void) const { return (int)_detailSampleMaxError; }

private:

    /**
      * Derive non-directly set parameters
      * This is the default behaviour and these parameters can be overridden using
      * _ setters.
      **/
    inline void eval(void) {
        _walkableHeight = (int)ceilf(agentHeight / cellHeight);
        _walkableClimb = (int)floorf(agentMaxClimb / cellHeight);
        _walkableRadius = (int)ceilf(agentRadius / cellSize);
        _maxEdgeLen = (int)(edgeMaxLen / cellSize);
        _minRegionArea = (int)rcSqr(regionMinSize);      // Note: area = size*size
        _mergeRegionArea = (int)rcSqr(regionMergeSize);   // Note: area = size*size
        _detailSampleDist = detailSampleDist < 0.9f ? 0 : cellSize * detailSampleDist;
        _detailSampleMaxError = cellHeight * detailSampleMaxError;
    }

    unsigned int MaxNumObstacles ;
    unsigned int TileSize ;

    /**
      * Cellsize (cs) is the width and depth resolution used when sampling the source geometry.
      * The width and depth of the cell columns that make up voxel fields.
      * Cells are laid out on the width/depth plane of voxel fields. Width is associated with the x-axis of the source geometry. Depth is associated with the z-axis.
      * A lower value allows for the generated meshes to more closely match the source geometry, but at a higher processing and memory cost.
      *
      * The xz-plane cell size to use for fields. [Limit: > 0] [Units: wu].
      * cs and ch define voxel/grid/cell size. So their values have significant side effects on all parameters defined in voxel units.
      * The minimum value for this parameter depends on the platform's floating point accuracy, with the practical minimum usually around 0.05.
      **/
    Ogre::Real cellSize;

    /**
      * Cellheight (ch) is the height resolution used when sampling the source geometry. The height of the voxels in voxel fields.
      * Height is associated with the y-axis of the source geometry.
      * A smaller value allows for the final meshes to more closely match the source geometry at a potentially higher processing cost.
      * (Unlike cellSize, using a lower value for cellHeight does not significantly increase memory use.)
      *
      * The y-axis cell size to use for fields. [Limit: > 0] [Units: wu].
      * cs and ch define voxel/grid/cell size. So their values have significant side effects on all parameters defined in voxel units.
      * The minimum value for this parameter depends on the platform's floating point accuracy, with the practical minimum usually around 0.05.
      *
      * Setting ch lower will result in more accurate detection of areas the agent can still pass under, as min walkable height is discretisized
      * in number of cells. Also walkableClimb's precision is affected by ch in the same way, along with some other parameters.
      **/
    Ogre::Real cellHeight;

    /**
      * The maximum slope that is considered traversable (in degrees).
      * [Limits: 0 <= value < 90]
      * The practical upper limit for this parameter is usually around 85 degrees.
      *
      * Also called maxTraversableSlope
      **/
    Ogre::Real agentMaxSlope;

    /**
      * The height of an agent. Defines the minimum height that
      * agents can walk under. Parts of the navmesh with lower ceilings
      * will be pruned off.
      *
      * This parameter serves at setting walkableHeight (minTraversableHeight) parameter, precision of this parameter is determined by cellHeight (ch).
      **/
    Ogre::Real agentHeight;

    /**
      * The Maximum ledge height that is considered to still be traversable.
      * This parameter serves at setting walkableClimb (maxTraversableStep) parameter, precision of this parameter is determined by cellHeight (ch).
      * [Limit: >=0]
      * Allows the mesh to flow over low lying obstructions such as curbs and up/down stairways. The value is usually set to how far up/down an agent can step.
      **/
    Ogre::Real agentMaxClimb;

    /**
      * The radius on the xz (ground) plane of the circle that describes the agent (character) size.
      * Serves at setting walkableRadius (traversableAreaBorderSize) parameter, the precision of walkableRadius is affected by cellSize (cs).
      *
      * This parameter is also used by DetourCrowd to determine the area other agents have to avoid in order not to collide with an agent.
      * The distance to erode/shrink the walkable area of the heightfield away from obstructions.
      * [Limit: >=0]
      *
      * In general, this is the closest any part of the final mesh should get to an obstruction in the source geometry. It is usually set to the maximum agent radius.
      * While a value of zero is legal, it is not recommended and can result in odd edge case issues.
      *
      **/
    Ogre::Real agentRadius;

    /**
      * The maximum allowed length for contour edges along the border of the mesh.
      * [Limit: >=0]
      * Extra vertices will be inserted as needed to keep contour edges below this length. A value of zero effectively disables this feature.
      * Serves at setting maxEdgeLen, the precision of maxEdgeLen is affected by cellSize (cs).
      **/
    Ogre::Real edgeMaxLen;

    /**
      * The maximum distance a simplfied contour's border edges should deviate the original raw contour. (edge matching)
      * [Limit: >=0] [Units: wu]
      * The effect of this parameter only applies to the xz-plane.
      *
      * Also called maxSimplificationError or edgeMaxDeviation
      * The maximum distance the edges of meshes may deviate from the source geometry.
      * A lower value will result in mesh edges following the xz-plane geometry contour more accurately at the expense of an increased triangle count.
      * A value to zero is not recommended since it can result in a large increase in the number of polygons in the final meshes at a high processing cost.
      **/
    Ogre::Real edgeMaxError;

    /**
      * The minimum number of cells allowed to form isolated island areas (size).
      * [Limit: >=0]
      * Any regions that are smaller than this area will be marked as unwalkable. This is useful in removing useless regions that can sometimes form on geometry such as table tops, box tops, etc.
      * Serves at setting minRegionArea, which will be set to the square of this value (the regions are square, thus area=size*size)
      **/
    Ogre::Real regionMinSize;

    /**
      * Any regions with a span count smaller than this value will, if possible, be merged with larger regions.
      * [Limit: >=0] [Units: vx]
      * Serves at setting MergeRegionArea, which will be set to the square of this value (the regions are square, thus area=size*size)
      **/
    Ogre::Real regionMergeSize;

    /**
      * The maximum number of vertices allowed for polygons generated during the contour to polygon conversion process.
      * [Limit: >= 3]
      * If the mesh data is to be used to construct a Detour navigation mesh, then the upper limit is limited to <= DT_VERTS_PER_POLYGON (=6).
      *
      * Also called maxVertsPerPoly
      * The maximum number of vertices per polygon for polygons generated during the voxel to polygon conversion process.
      * Higher values increase processing cost, but can also result in better formed polygons in the final meshes. A value of around 6 is generally adequate with diminishing returns for higher values.
      **/
    int vertsPerPoly;

    /**
      * Sets the sampling distance to use when generating the detail mesh.
      * (For height detail only.) [Limits: 0 or >= 0.9] [Units: wu]
      *
      * Also called contourSampleDistance
      * Sets the sampling distance to use when matching the detail mesh to the surface of the original geometry.
      * Impacts how well the final detail mesh conforms to the surface contour of the original geometry. Higher values result in a detail mesh which conforms more closely to the original geometry's surface at the cost of a higher final triangle count and higher processing cost.
      * Setting this argument to less than 0.9 disables this functionality.
      **/
    Ogre::Real detailSampleDist;

    /**
      * The maximum distance the detail mesh surface should deviate from heightfield data.
      * (For height detail only.) [Limit: >=0] [Units: wu]
      *
      * Also called contourMaxDeviation
      * The maximum distance the surface of the detail mesh may deviate from the surface of the original geometry.
      * The accuracy is impacted by contourSampleDistance.
      * The value of this parameter has no meaning if contourSampleDistance is set to zero.
      * Setting the value to zero is not recommended since it can result in a large increase in the number of triangles in the final detail mesh at a high processing cost.
      * Stronly related to detailSampleDist (contourSampleDistance).
      **/
    Ogre::Real detailSampleMaxError;

    /**
      * Determines whether intermediary results are stored in OgreRecast class or whether they are removed after navmesh creation.
      **/
    bool keepInterResults;


    /**
      * Minimum height in number of (voxel) cells that the ceiling needs to be
      * for an agent to be able to walk under. Related to cellHeight (ch) and
      * agentHeight.
      *
      * Minimum floor to 'ceiling' height that will still allow the floor area to be considered walkable.
      * [Limit: >= 3] [Units: vx]
      * Permits detection of overhangs in the source geometry that make the geometry below un-walkable. The value is usually set to the maximum agent height.
      *
      * Also called minTraversableHeight
      * This value should be at least two times the value of cellHeight in order to get good results.
      **/
    int _walkableHeight;

    /**
      * Maximum ledge height that is considered to still be traversable, in number of cells (height).
      * [Limit: >=0] [Units: vx].
      * Allows the mesh to flow over low lying obstructions such as curbs and up/down stairways. The value is usually set to how far up/down an agent can step.
      *
      * Also called maxTraversableStep
      * Represents the maximum ledge height that is considered to still be traversable.
      * Prevents minor deviations in height from improperly showing as obstructions. Permits detection of stair-like structures, curbs, etc.
      **/
    int _walkableClimb;

    /**
      * The distance to erode/shrink the walkable area of the heightfield away from obstructions, in cellsize units.
      * [Limit: >=0] [Units: vx]
      * In general, this is the closest any part of the final mesh should get to an obstruction in the source geometry. It is usually set to the maximum agent radius.
      * While a value of zero is legal, it is not recommended and can result in odd edge case issues.
      *
      * Also called traversableAreaBorderSize
      * Represents the closest any part of a mesh can get to an obstruction in the source geometry.
      * Usually this value is set to the maximum bounding radius of agents utilizing the meshes for navigation decisions.
      *
      * This value must be greater than the cellSize to have an effect.
      * The actual border will be larger around ledges if ledge clipping is enabled. See the clipLedges parameter for more information.
      * The actual border area will be larger if smoothingTreshold is > 0. See the smoothingThreshold parameter for more information.
      **/
    int _walkableRadius;


    /**
      * The maximum allowed length for contour edges along the border of the mesh.
      * [Limit: >=0] [Units: vx].
      * Extra vertices will be inserted as needed to keep contour edges below this length. A value of zero effectively disables this feature.
      *
      * Also called maxEdgeLength
      * The maximum length of polygon edges that represent the border of meshes.
      * More vertices will be added to border edges if this value is exceeded for a particular edge.
      * In certain cases this will reduce the number of long thin triangles.
      * A value of zero will disable this feature.
      **/
    int _maxEdgeLen;

    /**
      * The minimum number of cells allowed to form isolated island areas.
      * [Limit: >=0] [Units: vx].
      * Any regions that are smaller than this area will be marked as unwalkable.
      * This is useful in removing useless regions that can sometimes form on geometry such as table tops, box tops, etc.
      *
      * Also called minUnconnectedRegionSize
      * The minimum region size for unconnected (island) regions.
      * The value is in voxels.
      * Regions that are not connected to any other region and are smaller than this size will be culled before mesh generation. I.e. They will no longer be considered traversable.
      **/
    int _minRegionArea;

    /**
      * Any regions with a span count smaller than this value will, if possible, be merged with larger regions.
      * [Limit: >=0] [Units: vx]
      *
      * Also called mergeRegionSize or mergeRegionArea
      * Any regions smaller than this size will, if possible, be merged with larger regions.
      * Value is in voxels.
      * Helps reduce the number of small regions. This is especially an issue in diagonal path regions where inherent faults in the region generation algorithm can result in unnecessarily small regions.
      * Small regions are left unchanged if they cannot be legally merged with a neighbor region. (E.g. Merging will result in a non-simple polygon.)
      **/
    int _mergeRegionArea;

    /**
      * Sets the sampling distance to use when generating the detail mesh.
      * (For height detail only.) [Limits: 0 or >= 0.9] [Units: wu]
      *
      * Also called contourSampleDistance
      * Sets the sampling distance to use when matching the detail mesh to the surface of the original geometry.
      * Impacts how well the final detail mesh conforms to the surface contour of the original geometry. Higher values result in a
      * detail mesh which conforms more closely to the original geometry's surface at the cost of a higher final triangle count and higher processing cost.
      * Setting this argument to less than 0.9 disables this functionality.
      *
      * The difference between this parameter and edge matching (edgeMaxError) is that this parameter operates on the height rather than the xz-plane.
      * It also matches the entire detail mesh surface to the contour of the original geometry. Edge matching only matches edges of meshes to the contour of the original geometry.
      **/
    Ogre::Real _detailSampleDist;

    /**
      * The maximum distance the detail mesh surface should deviate from heightfield data.
      * (For height detail only.) [Limit: >=0] [Units: wu]
      *
      * Also called contourMaxDeviation
      * The maximum distance the surface of the detail mesh may deviate from the surface of the original geometry.
      * The accuracy is impacted by contourSampleDistance (detailSampleDist).
      * The value of this parameter has no meaning if contourSampleDistance is set to zero.
      * Setting the value to zero is not recommended since it can result in a large increase in the number of triangles in the final detail mesh at a high processing cost.
      * This parameter has no impact if contourSampleDistance is set to zero.
      **/
    Ogre::Real _detailSampleMaxError;
};
