#include "ZEDMultiController.hpp"
#include <algorithm>
#include <cmath>

#include <sl/Camera.hpp>

#define TIMEOUT_MAX 100

ZEDMultiController* ZEDMultiController::instance = nullptr;

ZEDMultiController::ZEDMultiController() {
   
}

ZEDMultiController::~ZEDMultiController() {
     destroy();
}

void ZEDMultiController::destroy() {
	multi_camera.close();
}


int ZEDMultiController::open(SL_InitMultiCameraParameters *params)
{
	sl::InitMultiCameraParameters init_multi_cam_parameters;
	init_multi_cam_parameters.max_input_fps = params->max_input_fps;
	init_multi_cam_parameters.depth_maximum_distance = params->depth_maximum_distance;
	init_multi_cam_parameters.camera_resolution = (sl::RESOLUTION)params->camera_resolution;
	init_multi_cam_parameters.depth_mode = (sl::DEPTH_MODE)params->depth_mode;
	init_multi_cam_parameters.coordinate_system = (sl::COORDINATE_SYSTEM)params->coordinate_system;
	init_multi_cam_parameters.coordinate_units = (sl::UNIT)params->coordinate_units;
	return (int)multi_camera.init(init_multi_cam_parameters);
}

void ZEDMultiController::close() {
	multi_camera.close();
}

int ZEDMultiController::enableObjectDetectionFusion(SL_ObjectDetectionFusionParameters* params)
{
	OD_fusion_init_params.detection_model = (sl::DETECTION_MODEL) params->detection_model;
	OD_fusion_init_params.body_format = (sl::BODY_FORMAT)params->body_format;
	return (int) multi_camera.enableObjectDetectionFusion(OD_fusion_init_params);
}

void ZEDMultiController::disableObjectDetectionFusion() {
	multi_camera.disableObjectDetectionFusion();
}

sl::ERROR_CODE ZEDMultiController::addCameraFromID(unsigned int id, SL_CameraIdentifier* uuid, SL_Vector3* position, SL_Vector3* rotation, struct SL_InitCameraParameters init_camera_param)
{
	sl::CameraIdentifier uuid_;
	uuid_.sn = 0;
	sl::InputType input;
	input.setFromCameraID(id);
	sl::Transform pose;
	pose.setIdentity();

	sl::Translation trans;
	trans.tx = position->x;
	trans.ty = position->y;
	trans.tz = position->z;
	pose.setTranslation(trans);

	sl::float3 orientation;
	orientation.x = rotation->x;
	orientation.y = rotation->y;
	orientation.z = rotation->z;
	pose.setRotationVector(orientation);

	sl::InitCameraParameters init_cam_p;
	init_cam_p.depth_maximum_distance = init_camera_param.depth_maximum_distance;
	init_cam_p.detection_confidence_threshold = init_camera_param.object_detection_confidence;

	sl::ERROR_CODE err = multi_camera.addCamera(input, uuid_, pose, init_cam_p);
	uuid->sn = uuid_.sn;

	return err;
}

sl::ERROR_CODE ZEDMultiController::addCameraFromSN(unsigned int serial_number, SL_CameraIdentifier* uuid, SL_Vector3* position, SL_Vector3* rotation, struct SL_InitCameraParameters init_camera_param)
{
	sl::CameraIdentifier uuid_;
	uuid_.sn = 0;
	sl::InputType input;
	input.setFromSerialNumber(serial_number);
	sl::Transform pose;
	pose.setIdentity();

	sl::Translation trans;
	trans.tx = position->x;
	trans.ty = position->y;
	trans.tz = position->z;
	pose.setTranslation(trans);

	sl::float3 orientation;
	orientation.x = rotation->x;
	orientation.y = rotation->y;
	orientation.z = rotation->z;
	pose.setRotationVector(orientation);

	sl::InitCameraParameters init_cam_p;
	init_cam_p.depth_maximum_distance = init_camera_param.depth_maximum_distance;
	init_cam_p.detection_confidence_threshold = init_camera_param.object_detection_confidence;

	sl::ERROR_CODE err = multi_camera.addCamera(input, uuid_, pose, init_cam_p);
	uuid->sn = uuid_.sn;

	return err;
}


sl::ERROR_CODE ZEDMultiController::addCameraFromSVO(const char* path_svo, SL_CameraIdentifier* uuid, SL_Vector3* position, SL_Vector3* rotation, struct SL_InitCameraParameters init_camera_param)
{
	sl::CameraIdentifier uuid_;
	uuid_.sn = 0;
	sl::InputType input;
	input.setFromSVOFile(sl::String(path_svo));
	sl::Transform pose;
	pose.setIdentity();

	sl::Translation trans;
	trans.tx = position->x;
	trans.ty = position->y;
	trans.tz = position->z;
	pose.setTranslation(trans);

	sl::float3 orientation;
	orientation.x = rotation->x;
	orientation.y = rotation->y;
	orientation.z = rotation->z;
	pose.setRotationVector(orientation);


	sl::InitCameraParameters init_cam_p;
	init_cam_p.depth_maximum_distance = init_camera_param.depth_maximum_distance;
	init_cam_p.detection_confidence_threshold = init_camera_param.object_detection_confidence;

	sl::ERROR_CODE err = multi_camera.addCamera(input, uuid_, pose, init_cam_p);
	uuid->sn = uuid_.sn;

	return err;
}


sl::ERROR_CODE ZEDMultiController::addCameraFromStreaming(const char* ip, unsigned short port, SL_CameraIdentifier* uuid, SL_Vector3* position, SL_Vector3* rotation, struct SL_InitCameraParameters init_camera_param)
{
	sl::CameraIdentifier uuid_;
	uuid_.sn = 0;
	sl::InputType input;
	input.setFromStream(sl::String(ip), port);
	sl::Transform pose;
	pose.setIdentity();

	sl::Translation trans;
	trans.tx = position->x;
	trans.ty = position->y;
	trans.tz = position->z;
	pose.setTranslation(trans);

	sl::float3 orientation;
	orientation.x = rotation->x;
	orientation.y = rotation->y;
	orientation.z = rotation->z;
	pose.setRotationVector(orientation);

	sl::InitCameraParameters init_cam_p;
	init_cam_p.depth_maximum_distance = init_camera_param.depth_maximum_distance;
	init_cam_p.detection_confidence_threshold = init_camera_param.object_detection_confidence;

	sl::ERROR_CODE err = multi_camera.addCamera(input, uuid_, pose, init_cam_p);
	uuid->sn = uuid_.sn;

	return err;
}


int ZEDMultiController::grabAll(SL_RuntimeMultiCameraParameters* rt_params)
{
	sl::RuntimeMultiCameraParameters rt_m;

	rt_m.force_grab_call = rt_params->force_grab_call;
	return (int)multi_camera.grabAll(rt_m);
}

sl::ERROR_CODE ZEDMultiController::removeCamera(SL_CameraIdentifier* uuid)
{
	sl::CameraIdentifier uuid_;
	uuid_.sn = uuid->sn;
	return multi_camera.removeCamera(uuid_);
}
 
sl::ERROR_CODE ZEDMultiController::retrieveFusedObjectDetectionData(SL_Objects* data) {
	memset(data, 0, sizeof(SL_Objects));
 
	sl::Objects objects;
	sl::ERROR_CODE v = multi_camera.retrieveFusedObjects(objects);
	if (v == sl::ERROR_CODE::SUCCESS) {
		//LOG(verbosity, "retrieve objects :" + std::to_string(objects.object_list.size()));
		data->is_new = objects.is_new;
		data->is_tracked = objects.is_tracked;
		data->detection_model = (SL_DETECTION_MODEL)OD_fusion_init_params.detection_model;
		int size_objects = objects.object_list.size();
		data->image_ts = objects.timestamp;
		data->nb_object = size_objects;

		int count = 0;

		for (auto &p : objects.object_list) {
			if (count < MAX_NUMBER_OBJECT) {
				//data->data_object[count].valid = true;
				data->object_list[count].label = (SL_OBJECT_CLASS)p.label;
				data->object_list[count].sublabel = (SL_OBJECT_SUBCLASS)p.sublabel;
				data->object_list[count].tracking_state = (SL_OBJECT_TRACKING_STATE)p.tracking_state;
				data->object_list[count].action_state = (SL_OBJECT_ACTION_STATE)p.action_state;
				data->object_list[count].id = p.id;
				data->object_list[count].confidence = p.confidence;
				data->object_list[count].raw_label = p.raw_label;

				memcpy(data->object_list[count].unique_object_id, p.unique_object_id, 37 * sizeof(char));


				for (int k = 0; k < 6; k++)
					data->object_list[count].position_covariance[k] = p.position_covariance[k];

				data->object_list[count].mask = (int*)(new sl::Mat(p.mask));

				for (int l = 0; l < 4; l++) {
					data->object_list[count].bounding_box_2d[l].x = (float)p.bounding_box_2d.at(l).x;
					data->object_list[count].bounding_box_2d[l].y = (float)p.bounding_box_2d.at(l).y;
				}

				// World data
				data->object_list[count].position.x = p.position.x;
				data->object_list[count].position.y = p.position.y;
				data->object_list[count].position.z = p.position.z;

				data->object_list[count].velocity.x = p.velocity.x;
				data->object_list[count].velocity.y = p.velocity.y;
				data->object_list[count].velocity.z = p.velocity.z;

				// 3D Bounding box in world frame
				for (int m = 0; m < 8; m++) {
					if (m < p.bounding_box.size()) {
						data->object_list[count].bounding_box[m].x = p.bounding_box.at(m).x;
						data->object_list[count].bounding_box[m].y = p.bounding_box.at(m).y;
						data->object_list[count].bounding_box[m].z = p.bounding_box.at(m).z;
					}
				}

				// if skeleton
				if (data->detection_model == SL_DETECTION_MODEL_HUMAN_BODY_FAST || data->detection_model == SL_DETECTION_MODEL_HUMAN_BODY_ACCURATE || data->detection_model == SL_DETECTION_MODEL_HUMAN_BODY_MEDIUM) {
					for (int i = 0; i < (int)p.keypoint.size(); i++) {
						data->object_list[count].keypoint_2d[i].x = p.keypoint_2d.at(i).x;
						data->object_list[count].keypoint_2d[i].y = p.keypoint_2d.at(i).y;

						data->object_list[count].keypoint[i].x = p.keypoint.at(i).x;
						data->object_list[count].keypoint[i].y = p.keypoint.at(i).y;
						data->object_list[count].keypoint[i].z = p.keypoint.at(i).z;
						data->object_list[count].keypoint_confidence[i] = p.keypoint_confidence.at(i);
					}

					data->object_list[count].head_position.x = p.head_position.x;
					data->object_list[count].head_position.y = p.head_position.y;
					data->object_list[count].head_position.z = p.head_position.z;

					for (int m = 0; m < 8; m++) {
						if (m < p.head_bounding_box.size()) {
							data->object_list[count].head_bounding_box[m].x = p.head_bounding_box.at(m).x;
							data->object_list[count].head_bounding_box[m].y = p.head_bounding_box.at(m).y;
							data->object_list[count].head_bounding_box[m].z = p.head_bounding_box.at(m).z;
						}
					}

					data->object_list[count].global_root_orientation.x = p.global_root_orientation.x;
					data->object_list[count].global_root_orientation.y = p.global_root_orientation.y;
					data->object_list[count].global_root_orientation.z = p.global_root_orientation.z;
					data->object_list[count].global_root_orientation.w = p.global_root_orientation.w;

					for (int i = 0; i < p.local_orientation_per_joint.size(); i++) { // 18 or 34

						data->object_list[count].local_orientation_per_joint[i].x = p.local_orientation_per_joint[i].x;
						data->object_list[count].local_orientation_per_joint[i].y = p.local_orientation_per_joint[i].y;
						data->object_list[count].local_orientation_per_joint[i].z = p.local_orientation_per_joint[i].z;
						data->object_list[count].local_orientation_per_joint[i].w = p.local_orientation_per_joint[i].w;

						data->object_list[count].local_position_per_joint[i].x = p.local_position_per_joint[i].x;
						data->object_list[count].local_position_per_joint[i].y = p.local_position_per_joint[i].y;
						data->object_list[count].local_position_per_joint[i].z = p.local_position_per_joint[i].z;
					}
				}
				count++;
			}
		}
	}
	return v;
 
}

