#include "PlayerFlagQueryFilter.h"
#include "OgreRecastDefinitions.h" // For POLYFLAGS

bool
PlayerFlagQueryFilter::
passFilter ( const dtPolyRef  ref,
             const dtMeshTile *tile,
             const dtPoly     *poly ) const
{
   if ( poly->getArea () == POLYAREA_GATE )
   {
      // Separate player and movement bits so that they can be compared independently,
      // i.e. passing one does not automatically pass the other.
      const unsigned short player_flags   = ( poly->flags & POLYFLAGS_ALL_PLAYERS ) ;
      const unsigned short movement_flags = ( poly->flags & POLYFLAGS_MOVE_MASK ) ;

      return ( ( ( player_flags   & getIncludeFlags () ) != 0 ) && // Any of the flags match
               ( ( movement_flags & getIncludeFlags () ) != 0 ) && // Any of the flags match
               ( ( player_flags   & getExcludeFlags () ) == 0 ) && // All of the flags do not match
               ( ( movement_flags & getExcludeFlags () ) == 0 ) ) ; // All of the flags do not match
   }
   else
   {
      // All non gate areas do not care about the player flags
      const unsigned short movement_flags = ( poly->flags & POLYFLAGS_MOVE_MASK ) ;

      return ( ( ( movement_flags & getIncludeFlags () ) != 0 ) && // Any of the flags match
               ( ( movement_flags & getExcludeFlags () ) == 0 ) ) ; // All of the flags do not match
   }
}
