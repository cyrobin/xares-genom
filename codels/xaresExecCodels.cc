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

#include <ostream>
#include <sstream>
#include <cstdlib>
#include <limits>
#include <sys/time.h>
#include "boost/format.hpp"

#include "server/xaresHeader.h"
#include "xares/xares.hpp"
#include "gladys/weight_map.hpp"
#include "gdalwrap/gdal.hpp"

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

void fill_gdal(gdalwrap::raster& gdal, const DTM_LABEL_POSTER* poster){//{{{
	int nbLines = poster->nbLines;
	int nbCols = poster->nbCols;
	const unsigned char* pu = poster->state;

	for (size_t i = 0; i < nbLines; ++i) 
		for (size_t j = 0; j < nbCols; ++j, ++pu)
		{
			switch (*pu) {
				case DTM_NO_LABEL:
					gdal[i + nbLines * j] = -100.0;
					break;
				case DTM_LABEL_TRAVERSABLE:
					gdal[i + nbLines * j] = 3.14;
					break;
				case DTM_LABEL_OBSTACLE:
					gdal[i + nbLines * j] = std::numeric_limits<float>::infinity();
					break;
				default:
					assert(false);
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
int dump_cnt = 0;
gladys::points_t last_goals ;

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

  // Set internal parameters
  SDI_F->internalParams.max_nf   = initParams->max_nf;
  SDI_F->internalParams.min_size = initParams->min_size;
  SDI_F->internalParams.min_dist = initParams->min_dist;
  SDI_F->internalParams.max_dist = initParams->max_dist;

  SDI_F->internalParams.x_origin   = initParams->x_origin  ;
  SDI_F->internalParams.y_origin   = initParams->y_origin  ;
  SDI_F->internalParams.height_max = initParams->height_max;
  SDI_F->internalParams.width_max  = initParams->width_max ;

  SDI_F->dump = GEN_FALSE;

  strncpy(SDI_F->logDir,initParams->logDir,XARES_MAX_LENGTH) ;

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
 *              S_xares_UNEXPECTED_DATA
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
  gdalwrap::raster& gdal = wm.setup_weight_band(poster->nbLines, poster->nbCols);

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
  gettimeofday(&tv0, NULL);
  fill_gdal(gdal, poster);
  gettimeofday(&tv1, NULL);

  posterGive(dtm_PosterID);

  std::cerr << "[xares] dtm data loaded (" 
            << (tv1.tv_sec -  tv0.tv_sec) * 1000 +
            (tv1.tv_usec - tv0.tv_usec) / 1000 << " ms)." << std::endl;

  /* load the planner */
  gettimeofday(&tv0, NULL);
  xares::xares xp( wm,
         SDI_F->internalParams.x_origin,
         SDI_F->internalParams.y_origin,
         SDI_F->internalParams.height_max,
         SDI_F->internalParams.width_max);
  gettimeofday(&tv1, NULL);

  std::cerr << "[xares] planner loaded ("
            << (tv1.tv_sec -  tv0.tv_sec) * 1000 +
            (tv1.tv_usec - tv0.tv_usec) / 1000 << " ms)." << std::endl;

  /* Set internal parameters for xares*/
  xp.set_params(
      SDI_F->internalParams.max_nf,
      SDI_F->internalParams.min_size,
      SDI_F->internalParams.min_dist,
      SDI_F->internalParams.max_dist
  );

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

  double yaw ;
  /* Get the desired cape */
  if ( last_goals.size() > 1 ) {
    // if there is past goals, try to keep the same direction
    yaw = gladys::yaw_angle_y_inv( last_goals[0], last_goals[1] );
  }
  else { 
    // else, try to keep the current yaw of the robot
    yaw = robotPos.mainToOrigin.euler.yaw ;
  }

  std::cerr << "[xares] seed (" 
            << r_pos[0][0] << "," << r_pos[0][1] 
            << ") (yaw = " << yaw << ") loaded." << std::endl;

  if ( SDI_F->dump ) {
    // open dump file
    std::ostringstream oss, oss2;
    oss << SDI_F->logDir << "/dump-xares-";
    oss << boost::format("%04s") % dump_cnt << ".log";
    std::ofstream dump_file( oss.str() );

    // dump r_pos
    dump_file << "r_pos " << r_pos.size() << " ";
    for (auto& pos : r_pos )
        dump_file << pos[0] << " " << pos[1];
    dump_file << std::endl;
    // dump yaw
    dump_file << "yaw " << yaw << std::endl;

    // dump internal params
    dump_file << "internal_params "
              << SDI_F->internalParams.max_nf   << " "
              << SDI_F->internalParams.min_size << " "
              << SDI_F->internalParams.min_dist << " "
              << SDI_F->internalParams.max_dist << " "
              << std::endl;

    // dump weight_map
    oss2 << SDI_F->logDir << "/dump-xares-" ;
    oss2 << boost::format("%04s") % dump_cnt << "-weight-map.tif";
    wm.save( oss2.str() );
    dump_file << oss2.str() << std::endl;

    // dump bounding of the area to explore
    dump_file << "bounded_area "
              << SDI_F->internalParams.x_origin << " "
              << SDI_F->internalParams.y_origin << " "
              << SDI_F->internalParams.height_max<< " "
              << SDI_F->internalParams.width_max << " "
              << std::endl;

    // dump weight_map
    // close file
    dump_file.close();
    dump_cnt++;
  }

  /* Plan */
  gettimeofday(&tv0, NULL);
  xares::xares::return_value rv = xp.plan( r_pos, yaw );
  gettimeofday(&tv1, NULL);

  if ( rv == xares::xares::XARES_FAILURE ) {
    std::cerr << "#EEE# xares : unexpected data ; unable to compute frontiers :'( " << std::endl;
    (*report) = S_xares_UNEXPECTED_DATA;
    return ETHER;
  }

  if ( rv == xares::xares::XARES_NO_FRONTIER ) {
    std::cerr << "#EEE# xares : no valuable frontier detected." << std::endl;
    (*report) = S_xares_NO_FRONTIER;
    return ETHER;
  }

  // From here, there should be a valid plan
  std::cerr << "[xares] Plan computed (" 
            << (tv1.tv_sec -  tv0.tv_sec) * 1000 +
            (tv1.tv_usec - tv0.tv_usec) / 1000 << " ms)" << std::endl;

  /* and Post */
  // get plan 
  const gladys::path_t& path = xp.get_goal().path ;

  for (unsigned int i = 0 ; i < path.size() ; i++)
    std::cerr   << "[xares] ----waypoint #" << i 
                << " = (" << path[i][0]<< "," << path[i][1] 
                <<")"<<std::endl;

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

  /* remember this goal for the next time (yaw) */
  last_goals.push_back( curr );

  return ETHER;
}//}}}

/*------------------------------------------------------------------------
 * ReinitYaw
 *
 * Description: 
 *
 * Reports:      OK
 */

/* xaresReinitYawCtrl  -  codel EXEC of ReinitYaw
   Returns:  EXEC END ETHER FAIL ZOMBIE */
ACTIVITY_EVENT
xaresReinitYawCtrl(int *report)
{//{{{
  /* forget about past goals */
  last_goals.clear() ;
  return ETHER;
}//}}}

