#ifndef XARES_STRUCT_H
#define XARES_STRUCT_H

#define XARES_REGION_PATH "Region Geotiff model (gladys/gdal)"
#define XARES_ROBOTMDL_PATH "Robot json model (gladys/nav_graph)"
#define XARES_POM_POSTER_NAME "pomPos"
#define XARES_DEFAULT_LOGDIR "/tmp"
#define XARES_MAX_LENGTH 1024
#define XARES_DEFAULT_LOADMAP ""

typedef struct xaresInitParams {
	char regionMapPath[XARES_MAX_LENGTH];
	char robotModelPath[XARES_MAX_LENGTH];
	char pomPosterName[XARES_MAX_LENGTH];
} xaresInitParams;

#endif //XARES_STRUCT_H
