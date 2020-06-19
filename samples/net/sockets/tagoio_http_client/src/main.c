/*
 * Copyright (c) 2020 Gerson Fernando Budke <nandojve@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(tagoio, CONFIG_TAGOIO_HTTP_LOG_LEVEL);

#include <zephyr.h>
#include <net/socket.h>
#include <net/http_client.h>
#include <random/rand32.h>
#include <stdio.h>

#include "ca_certificate.h"
#include "wifi.h"
#include "sockets.h"

static struct tagoio_context_t ctx;

static void response_cb(struct http_response *rsp,
			enum http_final_call final_data,
			void *user_data)
{
	if (final_data == HTTP_DATA_MORE) {
		LOG_DBG("Partial data received (%zd bytes)", rsp->data_len);
	} else if (final_data == HTTP_DATA_FINAL) {
		LOG_DBG("All the data received (%zd bytes)", rsp->data_len);
	}

	LOG_DBG("Response status %s", rsp->http_status);
}

static int collect_data(void)
{
	#define lower  20000
	#define upper 100000
	#define base    1000.00f

	float temp;
	int size;

	/* Generate a temperature between 20 and 100 celsius degree */
	temp = ((sys_rand32_get() % (upper - lower + 1)) + lower);
	ctx.raw_temperature = temp / base;

	size = snprintf(ctx.payload, sizeof(ctx.payload),
			"{\"variable\": \"temperature\","
			"\"unit\": \"c\",\"value\": %f}",
			ctx.raw_temperature);

	LOG_INF("%s", ctx.payload);

	return 0;
}

static void next_turn(void)
{
	if (collect_data() < 0) {
		LOG_INF("Error collecting data.");
		return;
	}

	if (tagoio_connect(&ctx) < 0) {
		LOG_INF("No connection available.");
		return;
	}

	if (tagoio_http_push(&ctx, response_cb) < 0) {
		LOG_INF("Error pushing data.");
		return;
	}
}

void main(void)
{
	printf("\n");
	LOG_INF("TagoIO IoT - HTTP Client - Temperature demo");

	wifi_connect();

	while (true) {
		next_turn();

		k_sleep(K_SECONDS(CONFIG_TAGOIO_HTTP_POOL_INTERVAL));
	}
}
