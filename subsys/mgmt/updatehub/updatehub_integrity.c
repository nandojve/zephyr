/*
 * Copyright (c) 2023 O.S.Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(updatehub, CONFIG_UPDATEHUB_LOG_LEVEL);

#include "updatehub_integrity.h"

int updatehub_integrity_init(struct updatehub_crypto_context *ctx)
{
	if (ctx == NULL) {
		LOG_DBG("Invalid integrity context");
		return -EINVAL;
	}

	memset(ctx, 0, sizeof(struct updatehub_crypto_context));

#if defined(CONFIG_BUILD_WITH_TFM)
	const psa_algorithm_t alg = PSA_ALG_SHA_256;
	psa_status_t status;

	ctx->hash_handle = psa_hash_operation_init();

	status = psa_hash_setup(&ctx->hash_handle, alg);
	if (status != PSA_SUCCESS) {
		LOG_DBG("Failed to setup hash op.");
		psa_hash_abort(&ctx->hash_handle);
		return -EACCES;
	}
#elif defined(CONFIG_FLASH_AREA_CHECK_INTEGRITY_MBEDTLS)
	int ret;

	ctx->md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
	if (ctx->md_info == NULL) {
		LOG_DBG("Message Digest not found or not enabled");
		return -ENOENT;
	}

	mbedtls_md_init(&ctx->md_ctx);
	ret = mbedtls_md_setup(&ctx->md_ctx, ctx->md_info, 0);
	if (ret == MBEDTLS_ERR_MD_BAD_INPUT_DATA) {
		LOG_DBG("Bad Message Digest selected");
		return -EFAULT;
	}
	if (ret == MBEDTLS_ERR_MD_ALLOC_FAILED) {
		LOG_DBG("Failed to allocate memory");
		return -ENOMEM;
	}

	ret = mbedtls_md_starts(&ctx->md_ctx);
	if (ret == MBEDTLS_ERR_MD_BAD_INPUT_DATA) {
		LOG_DBG("Bad Message Digest selected");
		return -EFAULT;
	}
#elif defined(CONFIG_FLASH_AREA_CHECK_INTEGRITY_TC)
	if (tc_sha256_init(&ctx->sha256sum) != TC_CRYPTO_SUCCESS) {
		LOG_DBG("Invalid integrity context");
		return -EFAULT;
	}
#endif

	return 0;
}

int updatehub_integrity_update(struct updatehub_crypto_context *ctx,
			       const uint8_t *buffer, const uint32_t len)
{
	if (ctx == NULL || buffer == NULL) {
		return -EINVAL;
	}

	/* bypass */
	if (len == 0) {
		return 0;
	}

#if defined(CONFIG_BUILD_WITH_TFM)
	psa_status_t status = psa_hash_update(&ctx->hash_handle, buffer, len);

	if (status != PSA_SUCCESS) {
		LOG_DBG("Failed to update hash.");
		psa_hash_abort(&ctx->hash_handle);
		return -EFAULT;
	}
#elif defined(CONFIG_FLASH_AREA_CHECK_INTEGRITY_MBEDTLS)
	int ret = mbedtls_md_update(&ctx->md_ctx, buffer, len);
	if (ret == MBEDTLS_ERR_MD_BAD_INPUT_DATA) {
		LOG_DBG("Bad Message Digest selected");
		return -EFAULT;
	}
#elif defined(CONFIG_FLASH_AREA_CHECK_INTEGRITY_TC)
	if (tc_sha256_update(&ctx->sha256sum, buffer, len) != TC_CRYPTO_SUCCESS) {
		LOG_DBG("Invalid integrity context or invalid buffer");
		return -EFAULT;
	}
#endif

	return 0;
}

int updatehub_integrity_finish(struct updatehub_crypto_context *ctx,
			       uint8_t *hash, const uint32_t size)
{
	if (ctx == NULL || hash == NULL) {
		return -EINVAL;
	}

#if defined(CONFIG_BUILD_WITH_TFM)
	size_t hash_len;
	psa_status_t status = psa_hash_finish(&ctx->hash_handle, hash,
					      size, &hash_len);

	if (status != PSA_SUCCESS) {
		LOG_DBG("Failed to finalize hash");
		psa_hash_abort(&ctx->hash_handle);
		return -EFAULT;
	}
#elif defined(CONFIG_FLASH_AREA_CHECK_INTEGRITY_MBEDTLS)
	if (size < mbedtls_md_get_size(ctx->md_info)) {
		LOG_DBG("HASH input buffer is to small to store the message digest");
		return -EINVAL;
	}

	int ret = mbedtls_md_finish(&ctx->md_ctx, hash);
	if (ret == MBEDTLS_ERR_MD_BAD_INPUT_DATA) {
		LOG_DBG("Bad Message Digest selected");
		return -EFAULT;
	}

	mbedtls_md_free(&ctx->md_ctx);
#elif defined(CONFIG_FLASH_AREA_CHECK_INTEGRITY_TC)
	if (tc_sha256_final(hash, &ctx->sha256sum) != TC_CRYPTO_SUCCESS) {
		LOG_DBG("Invalid integrity context or invalid hash pointer");
		return -EFAULT;
	}
#endif

	return 0;
}
