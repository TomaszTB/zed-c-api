#ifndef __ZED_MC_CONTROLLER_H__
#define __ZED_MC_CONTROLLER_H__


#include <sl/Camera.hpp>
#include <sl/MultiCamera.hpp>
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

    static bool isNotCreated() {
        return (instance == nullptr);
    }


	void close();

    SL_ERROR_CODE process();

    SL_ERROR_CODE subscribe(struct SL_CameraIdentifier* uuid);

    /////////////////////////////////////////////////////////////////////
    ///////////////////// Object Detection Fusion ///////////////////////
    /////////////////////////////////////////////////////////////////////

    ///
    /// \brief enables Object detection fusion module
    /// \param [in] parameters defined by \ref sl::ObjectDetectionFusionParameters
    /// \return
    ///
	SL_ERROR_CODE enableObjectDetectionFusion(struct SL_ObjectDetectionFusionParameters* params);

	void disableObjectDetectionFusion();
	
    ///
    /// \brief retrieves a list of objects (in sl::Objects class type) seen by all cameras and merged as if it was seen by a single super-camera.
    /// \note Internal calls retrieveObjects() for all listed cameras, then merged into a single sl::Objects
    /// \param [out] objs: list of objects seen by all available cameras
    /// \note Only the 3d informations is available in the returned object.
    /// \n For this version, a person is detected if at least it is seen by 2 cameras.
    ///
    SL_ERROR_CODE retrieveFusedObjects(struct SL_Objects* objs, struct SL_ObjectDetectionFusionRuntimeParameters* rt);

	void destroy();

private:
	sl::Fusion fusion;
	sl::ObjectDetectionFusionParameters OD_fusion_init_params;
    static ZEDMultiController* instance;

};

#endif
