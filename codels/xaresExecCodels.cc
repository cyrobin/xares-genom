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
#include <posterLib.h>
//#include <dtmStruct.h>

#include <ostream>
#include <cstdlib>
#include <limits>
#include <sys/time.h>

#include "server/xaresHeader.h"
#include "xares/xares.hpp"
#include "gladys/weight_map.hpp"

//#include <dtmStruct.h>

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

static inline
GENPOS_CART_CONFIG fake_from_local_coord( gladys::point_xy_t p ) {//{{{
  GENPOS_CART_CONFIG res;
  res.x = p[0] ;
  res.y = p[1] ;
  res.theta = 0;

  return res;
}//}}}

void fill_gdal(gladys::gdal::raster& gdal, const DTM_LABEL_POSTER* poster){//{{{
	int nbLines = poster->nbLines;
	int nbCols = poster->nbCols;

	for (size_t i = 0; i < nbLines; ++i) 
		for (size_t j = 0; j < nbCols; ++j)
		{
			switch (poster->state[i][j]) {
				case DTM_NO_LABEL:
					//gdal[i * nbCols + j] = 100.0;
					gdal[i + nbLines * j] = 100.0;
					break;
				case DTM_LABEL_TRAVERSABLE:
					//gdal[i * nbCols + j] = 3.14;
					gdal[i + nbLines * j] = 3.14;
					break;
				case DTM_LABEL_OBSTACLE:
					//gdal[i * nbCols + j] = std::numeric_limits<float>::infinity();
					gdal[i + nbLines * j] = std::numeric_limits<float>::infinity();
					break;
			}
		}
}//}}}

/*------------------------------------------------------------------------
 * Local variables
 */
// poster IDs that are initialized with PosterFind requests
static POSTER_ID robotPos_PosterID ;
//static POSTER_ID teammatePos_PosterID	;
static POSTER_ID dtm_PosterID ;

//gladys::weight_map wm;
//DTM_LABEL_POSTER* poster ;
//gladys::gdal::raster gdal ;

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

  // dtm label poster
  if (posterFind(initParams->dtmPosterName, &dtm_PosterID) == ERROR) {
    std::cerr << "#EEE# xares : cannot find dtm poster : " 
              << initParams->dtmPosterName << std::endl;
    (*report) = S_xares_CANNOT_CONNECT ;
    return ETHER;
  }

  std::cerr << "[xares] Init -- posters found." << std::endl;

//  // link the dtm with the weight map
//  DTM_LABEL_POSTER* poster = (DTM_LABEL_POSTER*)posterAddr(dtm_PosterID);
//  posterTake(dtm_PosterID, POSTER_READ);
//
//  gdal = wm.setup_weight_band(poster->nbLines, poster->nbCols);

  // Poster Init
  memset(&SDI_F->path, 0, sizeof(SDI_F->path));

  std::cerr << "[xares] Init done." << std::endl;

  //end
  (*report) = OK ;
  return ETHER;
}///}}}

/*------------------------------------------------------------------------
 * FindGoal
 *
 * Description: 
 *
 * Reports:      OK
 *              S_xares_NO_FRONTIER
 *              S_xares_CANNOT_READ_POSTER
 *              S_xares_CANNOT_UPDATE_POSTER
 */

/* xaresFindGoalMain  -  codel EXEC of FindGoal
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
xaresFindGoalMain(int *report)
{//{{{
  struct timeval tv0, tv1;

  std::cerr << "[xares] Init -- read posters." << std::endl;

  /* Init */
  // Read pom posters
  POM_POS robotPos;
  if ( posterRead( robotPos_PosterID, 0, &robotPos, sizeof(POM_POS) ) == ERROR) {
    std::cerr << "#EEE# xares : can not read pom poster." << std::endl;
    (*report) = S_xares_CANNOT_READ_POSTER;
    return ETHER;
  }

  // read dtm poster ; link the dtm with the weight map
  const DTM_LABEL_POSTER* poster = (const DTM_LABEL_POSTER*)posterAddr(dtm_PosterID);
  posterTake(dtm_PosterID, POSTER_READ);

  gladys::weight_map wm;
  gladys::gdal::raster& gdal = wm.setup_weight_band(poster->nbLines, poster->nbCols);

  std::cerr << "[xares] (x0,y0,xS,yS) = ("
            << poster->xOrigin << ","
            << poster->yOrigin << ","
            << poster->xScale << ","
            << poster->yScale << ")"
            << std::endl;
  wm.get_map().set_transform(  poster->xOrigin,
                               poster->yOrigin,
                               poster->xScale,
                               poster->yScale );

  // Update the weight map with the dtm label poster
  std::cerr << "[xares] Fill gdal with dtm poster." << std::endl;
  gettimeofday(&tv0, NULL);
  fill_gdal(gdal, poster);
  gettimeofday(&tv1, NULL);

  //for (auto& i : gdal)
      //std::cerr << i << " " ;
  //std::cerr << std::endl;

  //wm.get_map().save("/tmp/weight_map_003-genom.tif");
  //gladys::gdal tg ;
  //tg.load("/tmp/weight_map_003-genom.tif");
  //if (wm.get_map()==tg)
      //puts(" Oh yeah !");
  //else
      //puts("try again, fools!");

  posterGive(dtm_PosterID);

  std::cerr << "[xares] dtm data loaded (" 
            << (tv1.tv_sec -  tv0.tv_sec) * 1000 +
            (tv1.tv_usec - tv0.tv_usec) / 1000 << " ms)." << std::endl;

  /* load the planner */
  gettimeofday(&tv0, NULL);
  xares::xares xp( wm );
  gettimeofday(&tv1, NULL);

  std::cerr << "[xares] planner loaded ("
            << (tv1.tv_sec -  tv0.tv_sec) * 1000 +
            (tv1.tv_usec - tv0.tv_usec) / 1000 << " ms)." << std::endl;

  /* set local reference */
  std::array<double,4> transform = xp.get_transform() ;
  local_coords.xScale   = transform[0];
  local_coords.yScale   = transform[1];
  local_coords.xOrigin  = transform[2];
  local_coords.yOrigin  = transform[3];
  std::cerr << "[xares] local reference loaded." << std::endl;

  /* Transform POM_POS into gladys::points_t */
  gladys::points_t r_pos ; 
  r_pos.push_back( gladys::point_xy_t { robotPos.mainToOrigin.euler.x, 
                                        robotPos.mainToOrigin.euler.y } );
  std::cerr << "[xares] seed loaded." << std::endl;

  /* Plan */
  gettimeofday(&tv0, NULL);
  xp.plan( r_pos );
  gettimeofday(&tv1, NULL);

  std::cerr << "[xares] Plan computed (" 
            << (tv1.tv_sec -  tv0.tv_sec) * 1000 +
            (tv1.tv_usec - tv0.tv_usec) / 1000 << " ms)." << std::endl;

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
      //SDI_F->path.points[SDI_F->path.nbPts] = from_local_coord( *it );
      SDI_F->path.points[SDI_F->path.nbPts] = fake_from_local_coord( *it );
      SDI_F->path.nbPts++;
      dist = 0.0;
    }
  }

  /* Always add the last point to the path */
  //SDI_F->path.points[SDI_F->path.nbPts] = from_local_coord(curr);
  SDI_F->path.points[SDI_F->path.nbPts] = fake_from_local_coord(curr);
  SDI_F->path.nbPts++;

  SDI_F->path.numRef++;

  return ETHER;
}//}}}

