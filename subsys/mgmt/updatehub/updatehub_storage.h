/*
 * Copyright (c) 2023 O.S.Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __UPDATEHUB_STORAGE_H__
#define __UPDATEHUB_STORAGE_H__

#ifdef __cplusplus
extern "C" {
#endif

#if defined(CONFIG_BUILD_WITH_TFM)
#include <psa/update.h>
#define UPDATEHUB_SLOT_PARTITION_0 FWU_CALCULATE_IMAGE_ID(FWU_IMAGE_ID_SLOT_ACTIVE, \
							  FWU_IMAGE_TYPE_NONSECURE, 0)
#define UPDATEHUB_SLOT_PARTITION_1 FWU_CALCULATE_IMAGE_ID(FWU_IMAGE_ID_SLOT_ACTIVE, \
							  FWU_IMAGE_TYPE_NONSECURE, 0)
#else
#include <zephyr/dfu/flash_img.h>
#include <zephyr/storage/flash_map.h>
#define UPDATEHUB_SLOT_PARTITION_0 FIXED_PARTITION_ID(slot0_partition)
#define UPDATEHUB_SLOT_PARTITION_1 FIXED_PARTITION_ID(slot1_partition)
#endif

struct updatehub_storage_context {
#if defined(CONFIG_BUILD_WITH_TFM)
	uint32_t offset;
#else
	struct flash_img_context flash_ctx;
#endif
};

int updatehub_storage_is_partition_good(struct updatehub_storage_context *ctx);
int updatehub_storage_init(struct updatehub_storage_context *ctx,
			   const uint32_t partition_id);
int updatehub_storage_write(struct updatehub_storage_context *ctx,
			    const uint8_t *data, const size_t size,
			    const bool flush);
int updatehub_storage_check(struct updatehub_storage_context *ctx,
			    const uint32_t partition_id,
			    const uint8_t *hash, const size_t size);
int updatehub_storage_mark_partition_to_upgrade(struct updatehub_storage_context *ctx,
						const uint32_t partition_id);
int updatehub_storage_mark_partition_as_confirmed(const uint32_t partition_id);

#ifdef __cplusplus
}
#endif

#endif /* __UPDATEHUB_STORAGE_H__ */
