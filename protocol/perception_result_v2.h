/* SPDX-License-Identifier: Apache-2.0 */

#ifndef AKA_RK3588_PERCEPTION_RESULT_V2_H_
#define AKA_RK3588_PERCEPTION_RESULT_V2_H_

#include <stdint.h>

#define PERCEPTION_MAGIC 0x31524350U
#define PERCEPTION_VERSION 2U
#define PERCEPTION_TARGET_VISIBLE (1U << 0)
#define PERCEPTION_BUCKET_VISIBLE (1U << 1)
#define PERCEPTION_ROBOT_CI (1U << 2)

#ifdef __cplusplus
#define PERCEPTION_DEFAULT(value) = value
#else
#define PERCEPTION_DEFAULT(value)
#endif

/*
 * Versioned wire contract shared by the Starry/Linux perception process and
 * the Zephyr robot controller.  The structure occupies exactly one AxVisor
 * IVC v2 slot.  Add a new version instead of changing this layout in place.
 */
struct perception_result_v2 {
	uint32_t magic PERCEPTION_DEFAULT(PERCEPTION_MAGIC);
	uint16_t version PERCEPTION_DEFAULT(PERCEPTION_VERSION);
	uint16_t flags PERCEPTION_DEFAULT(0U);
	uint64_t sequence PERCEPTION_DEFAULT(0U);
	uint64_t monotonic_ms PERCEPTION_DEFAULT(0U);
	uint16_t frame_width PERCEPTION_DEFAULT(0U);
	uint16_t frame_height PERCEPTION_DEFAULT(0U);
	uint16_t confidence_milli PERCEPTION_DEFAULT(0U);
	uint16_t center_x PERCEPTION_DEFAULT(0U);
	uint16_t center_y PERCEPTION_DEFAULT(0U);
	uint16_t box_width PERCEPTION_DEFAULT(0U);
	uint16_t box_height PERCEPTION_DEFAULT(0U);
	uint16_t bucket_center_x PERCEPTION_DEFAULT(0U);
	uint16_t bucket_center_y PERCEPTION_DEFAULT(0U);
	uint16_t bucket_box_width PERCEPTION_DEFAULT(0U);
	uint16_t bucket_box_height PERCEPTION_DEFAULT(0U);
};

#undef PERCEPTION_DEFAULT

#endif /* AKA_RK3588_PERCEPTION_RESULT_V2_H_ */
