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
	fusion.close();
}

void ZEDMultiController::close() {
	fusion.close();
}

SL_ERROR_CODE ZEDMultiController::process() {

	return (SL_ERROR_CODE)fusion.process();
}

SL_ERROR_CODE ZEDMultiController::subscribe(struct SL_CameraIdentifier* uuid) {

	sl::CameraIdentifier sl_uuid;
	sl_uuid.sn = uuid->sn;
	return (SL_ERROR_CODE)fusion.subscribe(sl_uuid);
}

SL_ERROR_CODE ZEDMultiController::enableObjectDetectionFusion(SL_ObjectDetectionFusionParameters* params)
{
	OD_fusion_init_params.detection_model = (sl::DETECTION_MODEL) params->detection_model;
	OD_fusion_init_params.body_format = (sl::BODY_FORMAT)params->body_format;
	return (SL_ERROR_CODE)fusion.enableObjectDetectionFusion(OD_fusion_init_params);
}

void ZEDMultiController::disableObjectDetectionFusion() {
	fusion.disableObjectDetectionFusion();
}


SL_ERROR_CODE ZEDMultiController::retrieveFusedObjects(SL_Objects* data, struct SL_ObjectDetectionFusionRuntimeParameters* rt) {
	memset(data, 0, sizeof(SL_Objects));

	sl::ObjectDetectionFusionRuntimeParameters od_rt;
	od_rt.skeleton_minimum_allowed_keypoints = rt->skeleton_minimum_allowed_keypoints;

	sl::Objects objects;
	sl::ERROR_CODE v = fusion.retrieveFusedObjects(objects, od_rt);
	if (v == sl::ERROR_CODE::SUCCESS) {
		//LOG(verbosity, "retrieve objects :" + std::to_string(objects.object_list.size()));
		data->is_new = objects.is_new;
		data->is_tracked = objects.is_tracked;
		data->detection_model = (SL_DETECTION_MODEL)OD_fusion_init_params.detection_model;
		int size_objects = objects.object_list.size();
		data->image_ts = objects.timestamp;
		data->nb_object = size_objects;

		int count = 0;

		for (auto& p : objects.object_list) {
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
	return (SL_ERROR_CODE)v;

}
