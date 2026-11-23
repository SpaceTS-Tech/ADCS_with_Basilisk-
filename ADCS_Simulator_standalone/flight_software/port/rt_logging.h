/*
 * rt_logging.h -- standalone replacement for MATLAB's rtw/c/src/rt_logging.h
 *
 * The generated model was built with MAT-file logging enabled, so
 * ADCS_Software_v1.c unconditionally references RTWLogInfo, the rtliSet*
 * macros, rt_StartDataLoggingWithStartTime() and rt_UpdateTXYLogVars().  The
 * MathWorks implementation (rt_logging.c) writes .mat files through stdio and
 * the heap and pulls in matrix.h / rtw_matlogging.h from the MATLAB install.
 *
 * On the flight processor nothing is logged, so this header supplies the
 * same names as no-ops.  It replaces the MATLAB header and rt_logging.c; no
 * MATLAB installation is needed to compile the package.
 *
 * Remove this file once the model is regenerated with
 *   Configuration Parameters > Code Generation > Interface > MAT-file logging = off
 * (the generated code then no longer references any of these names).
 */
#ifndef rt_logging_h
#define rt_logging_h

#include <stddef.h>
#include "rtwtypes.h"

/* Only the field the generated code writes directly is kept. */
typedef struct {
    const void *loggingInterval;
} RTWLogInfo;

#define rtliSetLogXSignalInfo(li, v)      ((void)(li), (void)(v))
#define rtliSetLogXSignalPtrs(li, v)      ((void)(li), (void)(v))
#define rtliSetLogT(li, v)                ((void)(li), (void)(v))
#define rtliSetLogX(li, v)                ((void)(li), (void)(v))
#define rtliSetLogXFinal(li, v)           ((void)(li), (void)(v))
#define rtliSetLogVarNameModifier(li, v)  ((void)(li), (void)(v))
#define rtliSetLogFormat(li, v)           ((void)(li), (void)(v))
#define rtliSetLogMaxRows(li, v)          ((void)(li), (void)(v))
#define rtliSetLogDecimation(li, v)       ((void)(li), (void)(v))
#define rtliSetLogY(li, v)                ((void)(li), (void)(v))
#define rtliSetLogYSignalInfo(li, v)      ((void)(li), (void)(v))
#define rtliSetLogYSignalPtrs(li, v)      ((void)(li), (void)(v))

#define rt_StartDataLoggingWithStartTime(li, t0, tf, h, err) \
    ((void)(li), (void)(t0), (void)(tf), (void)(h), (void)(err))
#define rt_StartDataLogging(li, tf, h, err) \
    ((void)(li), (void)(tf), (void)(h), (void)(err))
#define rt_UpdateTXYLogVars(li, tPtr)     ((void)(li), (void)(tPtr))
#define rt_StopDataLogging(file, li)      ((void)(file), (void)(li))

#endif /* rt_logging_h */
