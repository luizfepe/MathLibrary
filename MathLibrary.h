// MathLibrary.h - Contains declarations of math functions
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <k4a/k4a.h>
#include <k4abt.h>

#ifdef MATHLIBRARY_EXPORTS
#define MATHLIBRARY_API __declspec(dllexport)
#else
#define MATHLIBRARY_API __declspec(dllimport)
#endif

    struct Teste
    {
        int t[2] = {1, 2};
    };

extern "C" MATHLIBRARY_API int retornateste();

extern "C" MATHLIBRARY_API void KinectInit();
extern "C" MATHLIBRARY_API uint32_t GetNumBodies();

extern "C" MATHLIBRARY_API void KinectFinish();
extern "C" MATHLIBRARY_API k4abt_body_t* CaptureFrame();
extern "C" MATHLIBRARY_API Teste* Testa();
#ifdef __cplusplus
}
#endif