#ifndef XARES_STRUCT_H
#define XARES_STRUCT_H

#define XARES_POM_POSTER_NAME "pomPos"
#define XARES_DTM_POSTER_NAME "dtm"

#define XARES_MAX_NF 20
#define XARES_MIN_SIZE 2.0
#define XARES_MIN_DIST 2.5
#define XARES_MAX_DIST 50.0
#define XARES_X0_AREA -300.0
#define XARES_Y0_AREA -300.0
#define XARES_HEIGHT_MAX 600.0
#define XARES_WIDTH_MAX 600.0

#define XARES_DEFAULT_LOGDIR "/tmp"

#define XARES_MAX_LENGTH 1024

typedef struct xaresInternalParams {

  unsigned int max_nf;
  double min_size;
  double min_dist;
  double max_dist;
  double x0_area ;
  double y0_area ;
  double height_max ;
  double width_max ;
} xaresInternalParams;

typedef struct xaresInitParams {
  char dtmPosterName[XARES_MAX_LENGTH];
  char pomPosterName[XARES_MAX_LENGTH];

  unsigned int max_nf;
  double min_size;
  double min_dist;
  double max_dist;
  double x0_area ;
  double y0_area ;
  double height_max ;
  double width_max ;

  char logDir[XARES_MAX_LENGTH];
} xaresInitParams;

#endif //XARES_STRUCT_H
