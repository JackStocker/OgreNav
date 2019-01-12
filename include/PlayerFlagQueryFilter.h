#include "DetourNavMeshQuery.h"

class PlayerFlagQueryFilter : public dtQueryFilter
{
public:
   PlayerFlagQueryFilter () {}
   virtual ~PlayerFlagQueryFilter () {}

   /// Returns true if the polygon can be visited.  (I.e. Is traversable.)
   ///  @param[in]		ref		The reference id of the polygon test.
   ///  @param[in]		tile	The tile containing the polygon.
   ///  @param[in]		poly  The polygon to test.
   virtual bool
   passFilter ( const dtPolyRef  ref,
                const dtMeshTile *tile,
                const dtPoly     *poly ) const override ;

} ;
