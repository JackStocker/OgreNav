#include "include/ConvexShapeObstacle.h"

ConvexShapeObstacle::ConvexShapeObstacle(Ogre::Vector3 position, Ogre::Real offset, OgreDetourTileCache *detourTileCache)
    : Obstacle(detourTileCache),
      mPosition(position),
      mEnt(0),
      mNode(0),
      mObstacleId(-1),
      mConvexHullDebug(0),
      mInputGeom(0)
{
    // Randomly place a box or a pot as obstacle
    mNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
    if (Ogre::Math::RangeRandom(0,2) < 1.5) {
        mEnt = mSceneMgr->createEntity("Box.mesh");
    } else {
        // For more complex entities, convex hull building will consider only MAX_CONVEXVOL_PTS vertices and the result can be sub-optimal.
        // A more robust convex hull building algorithm might be preferred.
        mEnt = mSceneMgr->createEntity("Pot.mesh");
        mNode->setScale(0.3, 0.3, 0.3);
    }
    mNode->attachObject(mEnt);
    mNode->setPosition(mPosition);

    // Transfer entitiy geometry to recast compatible format
// TODO I want to use the other constructor for one entity here!!
    std::vector<Ogre::Entity*> ents;
    ents.push_back(mEnt);
    // Note that it is important to first add your entity to the scene before creating an inputGeom from it.
    // This is so that it can calculate the world space coordinates for the object, which are needed for recast.
    mInputGeom = new InputGeom(ents);

    // Create convex area obstacle in the detourTileCache
    // Create convex hull with agent radios offset around the object (this is important so agents don't walk through the edges of the obstacle!)
    mConvexHull = mInputGeom->getConvexHull(offset);
    // WARNING: Watch out for memory leaks here! ConvexVolume objects are not managed by any system (except this class).
    mConvexHull->area = RC_NULL_AREA;   // Set area described by convex polygon to "unwalkable"
    // Add convex hull to detourTileCache as obstacle
    mObstacleId = mDetourTileCache->addConvexShapeObstacle(mConvexHull);

    mName = "ConvexObstacle_"+ Ogre::StringConverter::toString(mObstacleId);

    // Debug draw convex hull
// TODO add debug flag, grey lines around boxes should disappear when disabling debug drawing
    mConvexHullDebug = InputGeom::drawConvexVolume(mConvexHull, mSceneMgr);    // Debug convex volume
    Ogre::LogManager::getSingletonPtr()->logMessage("Adding obstacle "+mName);

    //if(mObstacleId == -1)
        // TODO exception when something goes wrong!

    //mEnt->setVisible(false);       // TODO maybe make boxes semi-transparent in debug draw mode
}

ConvexShapeObstacle::~ConvexShapeObstacle()
{
    // Removing obstacle from DetourTileCache is done by the application class (maybe clean this up in the future)

    mNode->removeAllChildren();
    mNode->getParentSceneNode()->removeChild(mNode);
    mSceneMgr->destroyEntity(mEnt);
    mSceneMgr->destroySceneNode(mNode);

    mNode = NULL;
    mEnt = NULL;

    mConvexHullDebug->detachFromParent();
    mSceneMgr->destroyManualObject(mConvexHullDebug);

    delete mInputGeom;

    mConvexHullDebug = NULL;
}


void ConvexShapeObstacle::update(long time)
{
}