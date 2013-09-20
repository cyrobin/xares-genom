/**
 ** xaresExecCodels.cc
 **
 ** Codels called by execution task xaresExec
 **
 ** Author: cyril.robin@laas.fr
 ** Date: Fri Sep 20 2013
 **
 **/

#include <portLib.h>

#include "server/xaresHeader.h"
#include "xares/xares.hpp"

#include <iostream>
#include <cstdlib>
#include <sys/time.h>

/*------------------------------------------------------------------------
 * Local structures and functions
 */
struct coords {//{{{
  //TODO take "North up" into account
  double xScale;
  double yScale;
  double xOrigin;
  double yOrigin;
};//}}}

coords local_coords;

static inline
gladys::point_xy_t to_local_coord(double x, double y) {//{{{
  return gladys::point_xy_t { (x - local_coords.xOrigin) / local_coords.xScale,
                          (y - local_coords.yOrigin) / local_coords.yScale };
}//}}}

static inline
gladys::point_xy_t to_local_coord( const POM_POS& pos ) {//{{{
  return to_local_coord(  pos.mainToOrigin.euler.x,
                          pos.mainToOrigin.euler.y);
}//}}}

static inline
GENPOS_CART_CONFIG from_local_coord( gladys::point_xy_t p ) {//{{{
  GENPOS_CART_CONFIG res;
  res.x = p[0] * local_coords.xScale + local_coords.xOrigin;
  res.y = p[1] * local_coords.yScale + local_coords.yOrigin;
  res.theta = 0;

  return res;
}//}}}

/*------------------------------------------------------------------------
 * Local variables
 */
// poster IDs that are initialized with PosterFind requests
static POSTER_ID robotPos_PosterID ;
//static POSTER_ID teammatePos_PosterID	;

// Used objects
static xares::xares xp ; // the exploration planner


/*------------------------------------------------------------------------
 * Init
 *
 * Description: 
 *
 * Reports:      OK
 *              S_xares_CANNOT_CONNECT
 *              S_xares_FAILED_CREATION
 */

/* xaresInitMain  -  codel EXEC of Init
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
xaresInitMain(xaresInitParams *initParams, int *report)
{//{{{
  /* Connect to Posters (POM) */
  // Robot pom Pos
  if (posterFind(initParams->pomPosterName, &robotPos_PosterID) == ERROR) {
    std::cerr << "#EEE# xares : cannot find pom poster : " 
              << initParams->pomPosterName << std::endl;
    (*report) = S_xares_CANNOT_CONNECT ;
    return ETHER;
  }

  /*  Here you get some xp, cause you're going on an adventure ! ;-) */
  // TODO try/catch ?
  xp = xares::xares(    initParams->regionMapPath,
                        initParams->robotModelPath );

  //if ( xp == NULL ) {
    //(*report) = S_xares_FAILED_CREATION ;
    //return FAIL;
  //}

  // set local reference
  std::array<double,4> transform = xp.get_transform() ;
  local_coords.xScale   = transform[0];
  local_coords.yScale   = transform[1];
  local_coords.xOrigin  = transform[2];
  local_coords.yOrigin  = transform[3];

  // Poster Init
  memset(&SDI_F->path, 0, sizeof(SDI_F->path));

  //end
  (*report) = OK ;
  return ETHER;
}///}}}

/*------------------------------------------------------------------------
 * UpdateModels
 *
 * Description: 
 *
 * Reports:      OK
 *              S_xares_FAILED_CREATION
 */

/* xaresUpdateMain  -  codel EXEC of UpdateModels
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
xaresUpdateMain(xaresInitParams *initParams, int *report)
{//{{{
  /* Load the new models */
  // TODO try/catch ?
  xp.load(    initParams->regionMapPath,
              initParams->robotModelPath );

  (*report) = OK ;
  return ETHER;
}//}}}

/*------------------------------------------------------------------------
 * FindGoal
 *
 * Description: 
 *
 * Reports:      OK
 *              S_xares_NO_FRONTIER
 *              S_xares_CANNOT_READ_POM
 *              S_xares_CANNOT_UPDATE_POSTER
 */

/* xaresFindGoalMain  -  codel EXEC of FindGoal
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
xaresFindGoalMain(int *report)
{//{{{
  /* Init */
  // Read pom posters
  POM_POS robotPos;
  if ( posterRead( robotPos_PosterID, 0, &robotPos, sizeof(POM_POS) ) == ERROR) {
    std::cerr << "#EEE# xares : can not read pom poster." << std::endl;
    (*report) = S_xares_CANNOT_READ_POM;
    return FAIL;
  }

  // Transform POM_POS into gladys::points_t
  gladys::points_t r_pos ; 
  r_pos.push_back( gladys::point_xy_t { robotPos.mainToOrigin.euler.x, 
                                        robotPos.mainToOrigin.euler.y } );
  /* Plan */
  struct timeval tv0, tv1;
  gettimeofday(&tv0, NULL);
  xp.plan( r_pos );
  gettimeofday(&tv1, NULL);

  std::cerr << "[xares] Plan computed in " 
            << (tv1.tv_sec -  tv0.tv_sec) * 1000 +
            (tv1.tv_usec - tv0.tv_usec) / 1000 << " ms\n";

  /* and Post */
  // get plan and transform it into GENPOS_TRAJ_POINTS ;
  const gladys::path_t& path = xp.get_path() ;

  if ( path.size() == 0 )
    return ETHER;

  gladys::path_t::const_iterator it;
  SDI_F->path.nbPts = 0;
  double dist = 0.0;

  gladys::point_xy_t curr = path.front() ;

  for ( it = path.begin();
        it != path.end() and SDI_F->path.nbPts < GENPOS_MAX_TRAJECTORY_SIZE ;
        ++it)
  {
    dist += gladys::distance( curr, *it );
    curr = *it;

    if ( dist > 6.0 ) {
      SDI_F->path.points[SDI_F->path.nbPts] = from_local_coord( *it );
      SDI_F->path.nbPts++;
      dist = 0.0;
    }
  }

  /* Always add the last point to the path */
  SDI_F->path.points[SDI_F->path.nbPts] = from_local_coord(curr);
  SDI_F->path.nbPts++;

  SDI_F->path.numRef++;

  return ETHER;
}//}}}

