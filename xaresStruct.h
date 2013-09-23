#ifndef XARES_STRUCT_H
#define XARES_STRUCT_H

#define XARES_POM_POSTER_NAME "pomPos"
#define XARES_DTM_POSTER_NAME "dtm"
#define XARES_DEFAULT_LOGDIR "/tmp"
#define XARES_MAX_LENGTH 1024
#define XARES_DEFAULT_LOADMAP ""

typedef struct xaresInitParams {
	char dtmPosterName[XARES_MAX_LENGTH];
	char pomPosterName[XARES_MAX_LENGTH];
} xaresInitParams;

#endif //XARES_STRUCT_H
