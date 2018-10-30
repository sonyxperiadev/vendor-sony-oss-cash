/*
 * CASH! Camera Augmented Sensing Helper
 * a multi-sensor camera helper server
 *
 * Copyright (C) 2018 AngeloGioacchino Del Regno <kholk11@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "CASHCTL"

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <cutils/android_filesystem_config.h>
#include <hardware/hardware.h>
#include <hardware/power.h>
#include <log/log.h>

#include "cash_ext.h"
#include "cash_private.h"

static int32_t send_cashsvr_data(struct cash_params params, struct cash_response *cash_resp)
{
	register int sock;
	int ret, len = sizeof(struct sockaddr_un);
	fd_set receivefd;
	struct sockaddr_un server_address;
	struct timeval timeout;

	/* Get socket in the UNIX domain */
	sock = socket(PF_UNIX, SOCK_SEQPACKET, 0);
	if (sock < 0) {
		ALOGE("Could not get the CASH Server from client");
		return -EPROTO;
	}

	memset(&server_address, 0, sizeof(struct sockaddr_un));
	server_address.sun_family = AF_UNIX;
	strcpy(server_address.sun_path, CASHSERVER_SOCKET);

	/* Set nonblocking I/O for socket to avoid stall */
	fcntl(sock, F_SETFL, O_NONBLOCK);

	ret = connect(sock, (struct sockaddr*)&server_address, len);
	if (ret < 0) {
		ALOGE("Cannot connect to CASH Server socket");
		goto end;
	}

	/* Send the filled struct */
	ret = send(sock, &params, sizeof(struct cash_params), 0);
	if (ret < 0) {
		ALOGE("Cannot send data to CASH Server");
		goto end;
	}

	/* Setup for receiving server reply (handle) */
	/* Initialize and set a new FD for receive operation */
	FD_ZERO(&receivefd);
	FD_SET(sock, &receivefd);

	/*
	 * Set a six seconds timeout for select operation
	 * because serial communication may be slow sometimes
	 */
	timeout.tv_sec = 6;
	timeout.tv_usec = 0;

	/* Wait until the socket is ready for receive operation */
	ret = select(sock+1, &receivefd, NULL, NULL, &timeout);
	if (ret < 0) {
		ALOGE("Socket error. Cannot continue.");
		if (ret == -1) {
			ALOGE("Socket not ready: timed out");
			ret = -ETIMEDOUT;
		}
		goto end;
	}

	/* New FD is set and the socket is ready to receive data */
	ret = recv(sock, cash_resp, sizeof(struct cash_response), 0);
	if (ret == -1) {
		ALOGE("Cannot receive reply from CASH Server");
		ret = -EINVAL;
	}
end:
	if (sock)
		close(sock);
	return ret;
}

static int32_t cashsvr_send_set(int operation, int value, struct cash_response *cash_resp)
{
	struct cash_params params;

	params.operation = operation;
	params.value = (int32_t)value;

	return send_cashsvr_data(params, cash_resp);
}

int cash_tof_start(int value)
{
	int rc;
	struct cash_response cash_resp;
	rc = cashsvr_send_set(OP_TOF_START, value, &cash_resp);
	if(rc > 0) {
		return cash_resp.retval;
	}
	return rc;
}

int cash_is_tof_in_range(void)
{
	int rc;
	struct cash_response cash_resp;
	rc = cashsvr_send_set(OP_CHECK_TOF_RANGE, 0, &cash_resp);
	if(rc > 0) {
		return cash_resp.retval;
	}
	return rc;
}

int32_t cash_get_focus(void)
{
	int rc;
	struct cash_response cash_resp;
	rc = cashsvr_send_set(OP_FOCUS_GET, 0, &cash_resp);
	if(rc > 0) {
		return cash_resp.focus_step;
	}
	return rc;
}

int cash_rgbc_start(int value)
{
	int rc;
	struct cash_response cash_resp;
	rc = cashsvr_send_set(OP_RGBC_START, value, &cash_resp);
	if(rc > 0)
		return cash_resp.retval;
	return rc;
}

int cash_is_rgbc_in_range(void)
{
	int rc;
	struct cash_response cash_resp;
	rc = cashsvr_send_set(OP_CHECK_RGBC_RANGE, 0, &cash_resp);
	if(rc > 0)
		return cash_resp.retval;
	return rc;
}

struct exptime_iso_tpl cash_get_exptime_iso(void)
{
	int rc;
	struct cash_response cash_resp;
	struct exptime_iso_tpl exptime_iso = { -1, -1};
	rc = cashsvr_send_set(OP_EXPTIME_ISO_GET, 0, &cash_resp);
	if(rc > 0) {
		exptime_iso.exptime = cash_resp.exptime;
		exptime_iso.iso = cash_resp.iso;
	}
	return exptime_iso;
}


