//
// MATLAB Compiler: 6.6 (R2018a)
// Date: Tue Jun 25 10:30:08 2019
// Arguments:
// "-B""macro_default""-W""cpplib:libmatlabPLE""-T""link:lib""ple_algorithm.m"
//

#include <stdio.h>
#define EXPORTING_libmatlabPLE 1
#include "libmatlabPLE.h"

static HMCRINSTANCE _mcr_inst = NULL;

#ifdef __cplusplus
extern "C" {
#endif

static int mclDefaultPrintHandler(const char *s)
{
    return mclWrite(1 /* stdout */, s, sizeof(char)*strlen(s));
}

#ifdef __cplusplus
} /* End extern "C" block */
#endif

#ifdef __cplusplus
extern "C" {
#endif

static int mclDefaultErrorHandler(const char *s)
{
    int written = 0;
    size_t len = 0;
    len = strlen(s);
    written = mclWrite(2 /* stderr */, s, sizeof(char)*len);
    if (len > 0 && s[ len-1 ] != '\n')
        written += mclWrite(2 /* stderr */, "\n", sizeof(char));
    return written;
}

#ifdef __cplusplus
} /* End extern "C" block */
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_libmatlabPLE_C_API
#define LIB_libmatlabPLE_C_API /* No special import/export declaration */
#endif

LIB_libmatlabPLE_C_API 
bool MW_CALL_CONV libmatlabPLEInitializeWithHandlers(
    mclOutputHandlerFcn error_handler,
    mclOutputHandlerFcn print_handler)
{
    int bResult = 0;
    if (_mcr_inst != NULL)
        return true;
    if (!mclmcrInitialize())
        return false;
    {
        mclCtfStream ctfStream = 
            mclGetEmbeddedCtfStream((void *)(libmatlabPLEInitializeWithHandlers));
        if (ctfStream) {
            bResult = mclInitializeComponentInstanceEmbedded(&_mcr_inst,
                                                             error_handler, 
                                                             print_handler,
                                                             ctfStream);
            mclDestroyStream(ctfStream);
        } else {
            bResult = 0;
        }
    }  
    if (!bResult)
    return false;
    return true;
}

LIB_libmatlabPLE_C_API 
bool MW_CALL_CONV libmatlabPLEInitialize(void)
{
    return libmatlabPLEInitializeWithHandlers(mclDefaultErrorHandler, 
                                            mclDefaultPrintHandler);
}

LIB_libmatlabPLE_C_API 
void MW_CALL_CONV libmatlabPLETerminate(void)
{
    if (_mcr_inst != NULL)
        mclTerminateInstance(&_mcr_inst);
}

LIB_libmatlabPLE_C_API 
void MW_CALL_CONV libmatlabPLEPrintStackTrace(void) 
{
    char** stackTrace;
    int stackDepth = mclGetStackTrace(&stackTrace);
    int i;
    for(i=0; i<stackDepth; i++)
    {
        mclWrite(2 /* stderr */, stackTrace[i], sizeof(char)*strlen(stackTrace[i]));
        mclWrite(2 /* stderr */, "\n", sizeof(char)*strlen("\n"));
    }
    mclFreeStackTrace(&stackTrace, stackDepth);
}


LIB_libmatlabPLE_C_API 
bool MW_CALL_CONV mlxPle_algorithm(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[])
{
    return mclFeval(_mcr_inst, "ple_algorithm", nlhs, plhs, nrhs, prhs);
}

LIB_libmatlabPLE_CPP_API 
void MW_CALL_CONV ple_algorithm()
{
    mclcppMlfFeval(_mcr_inst, "ple_algorithm", 0, 0, 0);
}

