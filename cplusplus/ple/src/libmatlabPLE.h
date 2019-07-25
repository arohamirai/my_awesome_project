//
// MATLAB Compiler: 6.6 (R2018a)
// Date: Tue Jun 25 10:30:08 2019
// Arguments:
// "-B""macro_default""-W""cpplib:libmatlabPLE""-T""link:lib""ple_algorithm.m"
//

#ifndef __libmatlabPLE_h
#define __libmatlabPLE_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" {
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_libmatlabPLE_C_API 
#define LIB_libmatlabPLE_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_libmatlabPLE_C_API 
bool MW_CALL_CONV libmatlabPLEInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_libmatlabPLE_C_API 
bool MW_CALL_CONV libmatlabPLEInitialize(void);

extern LIB_libmatlabPLE_C_API 
void MW_CALL_CONV libmatlabPLETerminate(void);

extern LIB_libmatlabPLE_C_API 
void MW_CALL_CONV libmatlabPLEPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_libmatlabPLE_C_API 
bool MW_CALL_CONV mlxPle_algorithm(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_libmatlabPLE
#define PUBLIC_libmatlabPLE_CPP_API __declspec(dllexport)
#else
#define PUBLIC_libmatlabPLE_CPP_API __declspec(dllimport)
#endif

#define LIB_libmatlabPLE_CPP_API PUBLIC_libmatlabPLE_CPP_API

#else

#if !defined(LIB_libmatlabPLE_CPP_API)
#if defined(LIB_libmatlabPLE_C_API)
#define LIB_libmatlabPLE_CPP_API LIB_libmatlabPLE_C_API
#else
#define LIB_libmatlabPLE_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_libmatlabPLE_CPP_API void MW_CALL_CONV ple_algorithm();

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
