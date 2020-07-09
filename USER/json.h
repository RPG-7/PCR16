#ifndef _JSON_H
#define _JSON_H

#include <jansson.h>
#include "includes.h"

void jansson_init(void);
//int CreateTemp_Jsonfile(const char *path);
//int AnalysisTemp_Jsonfile(const char *path);
int CreateLab_Jsonfile(const char *path);
int AnalysisLab_Jsonfile(const char *path);
#endif

