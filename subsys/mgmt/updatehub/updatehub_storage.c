/*
 * Copyright (c) 2023 O.S.Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(updatehub, CONFIG_UPDATEHUB_LOG_LEVEL);

#if defined(CONFIG_BOOTLOADER_MCUBOOT)
#include <zephyr/dfu/mcuboot.h>
#endif

#include "updatehub_storage.h"

int updatehub_storage_is_partition_good(struct updatehub_storage_context *ctx)
{
	bool ret;

	if (ctx == NULL) {
		return -EINVAL;
	}

#if defined(CONFIG_BUILD_WITH_TFM)
	psa_image_info_t info = { 0 };
	psa_status_t status = psa_fwu_query(UPDATEHUB_SLOT_PARTITION_0, &info);

	LOG_DBG("STATUS: %d, STATE: %d", status, info.state);

	ret = (status == PSA_SUCCESS && info.state == PSA_IMAGE_INSTALLED);
#else
	ret = boot_is_img_confirmed() == 0;
#endif

	return ret ? 0 : -EIO;
}

int updatehub_storage_init(struct updatehub_storage_context *ctx,
			   const uint32_t partition_id)
{
	if (ctx == NULL) {
		return -EINVAL;
	}

#if defined(CONFIG_BUILD_WITH_TFM)
	ctx->offset = 0;
	return 0;
#else
	if (boot_erase_img_bank(partition_id)) {
		return -EIO;
	}

	return flash_img_init(&ctx->flash_ctx);
#endif
}

int updatehub_storage_write(struct updatehub_storage_context *ctx,
			    const uint8_t *data, const size_t size,
			    const bool flush)
{
	if (ctx == NULL || (size > 0 && data == NULL)) {
		return -EINVAL;
	}

#if defined(CONFIG_BUILD_WITH_TFM)
	const uint32_t partition_id = 0;
	psa_status_t status = psa_fwu_write(partition_id, ctx->offset, data, size);

	switch (status) {
	case PSA_ERROR_INVALID_ARGUMENT:
		LOG_DBG("Invalid PSA argument");
		return -EINVAL;
	case PSA_ERROR_INSUFFICIENT_MEMORY:
		LOG_DBG("Insufficient memory");
		return -ENOMEM;
	case PSA_ERROR_INSUFFICIENT_STORAGE:
		LOG_DBG("Insufficient storage");
		return -EFBIG;
	case PSA_ERROR_GENERIC_ERROR:
		LOG_DBG("Generic error");
		return -EIO;
	case PSA_ERROR_CURRENTLY_INSTALLING:
		LOG_DBG("Currently installing");
		return -EBUSY;
	default:
		break;
	}

	ctx->offset += size;

	return 0;
#else
	LOG_DBG("Flash: Address: 0x%08x, Size: %d, Flush: %s",
		ctx->flash_ctx.stream.bytes_written, size,
		flush ? "True" : "False");

	return flash_img_buffered_write(&ctx->flash_ctx, data, size, flush);
#endif
}

int updatehub_storage_check(struct updatehub_storage_context *ctx,
			    const uint32_t partition_id,
			    const uint8_t *hash, const size_t size)
{
	if (ctx == NULL || hash == NULL || size == 0) {
		return -EINVAL;
	}

#if defined(CONFIG_BUILD_WITH_TFM)
	/* PSA perform image check */
	return 0;
#else
	const struct flash_img_check fic = { .match = hash, .clen = size };

	return flash_img_check(&ctx->flash_ctx, &fic, partition_id);
#endif
}

int updatehub_storage_mark_partition_to_upgrade(struct updatehub_storage_context *ctx,
						const uint32_t partition_id)
{
	if (ctx == NULL) {
		return -EINVAL;
	}

#if defined(CONFIG_BUILD_WITH_TFM)
	psa_image_id_t uuid;
	psa_image_version_t version;
	psa_status_t status = psa_fwu_install(partition_id, &uuid, &version);

	switch (status) {
	case PSA_ERROR_INVALID_ARGUMENT:
		LOG_DBG("Invalid argument");
		return -EINVAL;
	case PSA_ERROR_INVALID_SIGNATURE:
		LOG_DBG("Invalid signature");
		return -ENOEXEC;
	case PSA_ERROR_GENERIC_ERROR:
		LOG_DBG("Generic error");
		return -EFAULT;
	case PSA_ERROR_STORAGE_FAILURE:
		LOG_DBG("Storage failure");
		return -EIO;
	case PSA_ERROR_DEPENDENCY_NEEDED:
		LOG_DBG("Dependency needed: uuid: %x version: %d.%d.%d-%d",
			uuid, version.iv_major, version.iv_minor,
			version.iv_revision, version.iv_build_num);
		return -EXDEV;
	}

	psa_image_info_t info = { 0 };

	psa_fwu_query(partition_id, &info);
	switch (info.state) {
	case PSA_IMAGE_UNDEFINED:
		LOG_DBG("New Firmware state is UNDEFINED");
		break;
	case PSA_IMAGE_CANDIDATE:
		LOG_DBG("New firmware set candidate...");
		break;
	case PSA_IMAGE_INSTALLED:
		LOG_DBG("Rebooting...");
		break;
	case PSA_IMAGE_REJECTED:
		LOG_DBG("New Firmware rejected");
		break;
	case PSA_IMAGE_PENDING_INSTALL:
		LOG_DBG("New Firmware pending install...");
		break;
	case PSA_IMAGE_REBOOT_NEEDED:
		LOG_DBG("Reboot Needed...");
		break;
	}

	return 0;
#else
	return boot_request_upgrade_multi(partition_id, BOOT_UPGRADE_TEST);
#endif
}

int updatehub_storage_mark_partition_as_confirmed(const uint32_t partition_id)
{
#if defined(CONFIG_BUILD_WITH_TFM)
	psa_status_t status = psa_fwu_accept(partition_id);

	switch (status) {
	case PSA_ERROR_NOT_SUPPORTED:
		LOG_DBG("Image do not support PSA_IMAGE_PENDING_INSTALL");
		return -EINVAL;
	case PSA_ERROR_NOT_PERMITTED:
		LOG_DBG("Caller do not have permission to execute this operation");
		return -EPERM;
	}

	return 0;
#else
	return boot_write_img_confirmed();
#endif
}
