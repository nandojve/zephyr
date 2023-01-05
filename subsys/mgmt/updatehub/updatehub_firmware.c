/*
 * Copyright (c) 2018-2023 O.S.Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(updatehub, CONFIG_UPDATEHUB_LOG_LEVEL);

#if defined(CONFIG_BUILD_WITH_TFM)
#include <psa/update.h>
#else
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/storage/flash_map.h>
#endif

#include <zephyr/sys/printk.h>

#include "updatehub_firmware.h"

bool updatehub_get_firmware_version(const uint32_t partition_id,
				    char *version, int version_len)
{
#if defined(CONFIG_BUILD_WITH_TFM)
	psa_image_info_t info = { 0 };
	psa_status_t status = psa_fwu_query(partition_id, &info);

	if (!(status == PSA_SUCCESS && info.state == PSA_IMAGE_INSTALLED)) {
		return false;
	}

	snprintk(version, version_len, "%d.%d.%d",
		 info.version.iv_major,
		 info.version.iv_minor,
		 info.version.iv_revision);
#else
	struct mcuboot_img_header header;

	if (boot_read_bank_header(partition_id, &header,
				  sizeof(struct mcuboot_img_header)) != 0) {
		LOG_DBG("Error when executing boot_read_bank_header function");
		return false;
	}

	if (header.mcuboot_version != 1) {
		LOG_DBG("MCUboot header version not supported!");
		return false;
	}

	snprintk(version, version_len, "%d.%d.%d",
		 header.h.v1.sem_ver.major,
		 header.h.v1.sem_ver.minor,
		 header.h.v1.sem_ver.revision);
#endif

	return true;
}
