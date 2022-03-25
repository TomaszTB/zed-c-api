#ifndef __ZED_MC_CONTROLLER_H__
#define __ZED_MC_CONTROLLER_H__


#include <sl/Camera.hpp>
#include <sl_mc/MultiCamera.hpp>
#ifdef _WIN32
#include <Windows.h>
#endif
#include <cuda.h>
#include "cuda_runtime.h"
#include "sl/c_api/types_c.h"

static std::mutex mglobalmutex;

class ZEDMultiController {
public:
	ZEDMultiController();
    ~ZEDMultiController();

    static ZEDMultiController* get() {
        if (!instance) // Only allow one instance of class to be generated.
            instance = new ZEDMultiController();

        return instance;
    }

    static void destroyInstance() {
        if (!instance) // Only allow one instance of class to be generated.
            delete instance;
        instance = nullptr;

    }

    static bool isNotCreated(int i) {
        return (instance == nullptr);
    }

 

	int open(struct SL_InitMultiCameraParameters *params);
	void close();
	int enableObjectDetectionFusion(struct SL_ObjectDetectionFusionParameters* params);
	void disableObjectDetectionFusion();
	
 
	sl::ERROR_CODE addCameraFromID(unsigned int id, struct SL_CameraIdentifier* uuid, struct SL_Vector3* position, struct SL_Vector3* rotation, struct SL_InitCameraParameters init_camera_param);
	sl::ERROR_CODE addCameraFromSN(unsigned int serial_number, struct SL_CameraIdentifier* uuid, struct SL_Vector3* position, struct SL_Vector3* rotation, struct SL_InitCameraParameters init_camera_param);
	sl::ERROR_CODE addCameraFromSVO(const char* path_svo, struct SL_CameraIdentifier* uuid, struct SL_Vector3* position, struct SL_Vector3* rotation, struct SL_InitCameraParameters init_camera_param);
	sl::ERROR_CODE addCameraFromStreaming(const char* ip, unsigned short port, struct SL_CameraIdentifier* uuid, struct SL_Vector3* position, struct SL_Vector3* rotation, struct SL_InitCameraParameters init_camera_param);


	int grabAll(struct SL_RuntimeMultiCameraParameters* rt_params);

	sl::ERROR_CODE retrieveFusedObjectDetectionData(struct SL_Objects* data);

	sl::ERROR_CODE removeCamera(struct SL_CameraIdentifier* uuid);
	void destroy();

private:
	sl::MultiCameraHandler multi_camera;
	sl::ObjectDetectionFusionParameters OD_fusion_init_params;
    static ZEDMultiController* instance;

 

};

#endif
