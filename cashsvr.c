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

#define LOG_TAG			"CASH"

#include <cutils/properties.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <assert.h>

#include <cutils/android_filesystem_config.h>
#include <log/log.h>

#include <libpolyreg/polyreg.h>
#include "cash_private.h"
#include "cash_input.h"
#include "cash_ext.h"

#define UNUSED __attribute__((unused))

/* Serial port fd */
static int serport = -1;
static struct cash_focus_params focus_conf;
static struct cash_configuration cash_conf;

/* CASH Server */
static int sock;
static int clientsock;
static struct sockaddr_un server_addr;
static pthread_t cashsvr_thread;
static bool ucthread_run = true;

/* Debugging defines */
// #define DEBUG_CMDS
// #define DEBUG_FOCUS

static int cashsvr_tof_start(int ena)
{
	return cash_input_tof_start(ena);
}

/*
 * cashsvr_is_tof_in_range - Checks if the ToF reading is between the
 *                           allowed range.
 *
 * \return Returns 0 (FALSE) for "out of range" or "error" or 1 (TRUE)
 */
int cashsvr_is_tof_in_range(void)
{
	int tof_score, rc;
	struct cash_vl53l0 tof_data;

	if (cash_conf.disable_tof)
		return 0;

	if (!cash_input_is_tof_alive())
		return 0;

	if (cash_conf.use_tof_stabilized) {
		tof_score = cash_tof_thr_read_stabilized(&tof_data,
				TOF_STABILIZATION_DEF_RUNS,
				TOF_STABILIZATION_MATCH_NO,
				TOF_STABILIZATION_WAIT_MS,
				TOF_STABILIZATION_HYST_MM);

		ALOGI("Got tof score %d", tof_score);
	} else {
		rc = cash_tof_read_inst(&tof_data);
		if (rc < 0)
			return 0;
	}

	if (tof_data.range_mm < cash_conf.tof_min ||
	    tof_data.range_mm > cash_conf.tof_max)
		return 0;

	return 1;
}

int32_t cashsvr_get_focus(void) {
	int rc, tof_score;
	int32_t focus_step;
	struct cash_vl53l0 tof_data;

	if (cash_conf.use_tof_stabilized) {
		tof_score = cash_tof_thr_read_stabilized(&tof_data,
				cash_conf.tof_max_runs,
				TOF_STABILIZATION_MATCH_NO,
				TOF_STABILIZATION_WAIT_MS,
				cash_conf.tof_hyst);
	} else {
		rc = cash_tof_read_inst(&tof_data);
		if (rc < 0)
			return 0;
	}

	focus_step = (int32_t)polyreg_f(tof_data.range_mm, focus_conf.terms,
					cash_conf.polyreg_degree);

	ALOGD("Setting focus %d for %dmm", focus_step, tof_data.range_mm);

	return focus_step;
}

/*
 * cash_dispatch - Recognizes the requested operation and calls
 *		    the appropriate functions.
 *
 * \return Returns success(0) or negative errno.
 */
static int32_t cash_dispatch(struct cash_params *params)
{
	int32_t rc;
	int val = params->value;

	switch (params->operation) {
	case OP_TOF_START:
		rc = cashsvr_tof_start(val);
	case OP_CHECK_TOF_RANGE:
		rc = cashsvr_is_tof_in_range();
		break;
	case OP_FOCUS_GET:
		rc = cashsvr_get_focus();
		break;
	default:
		ALOGE("Invalid operation requested.");
		rc = -2;
	}

	return rc;
}

static void *cashsvr_looper(void *unusedvar UNUSED)
{
	int ret;
	int32_t cash_reply = -EINVAL;
	uint8_t retry;
	socklen_t clientlen = sizeof(struct sockaddr_un);
	struct sockaddr_un client_addr;
	struct cash_params extparams;

reloop:
	ALOGI("CASH Server is waiting for connection...");
	if (clientsock)
		close(clientsock);
	retry = 0;
	while (((clientsock = accept(sock, (struct sockaddr*)&client_addr,
		&clientlen)) > 0) && (ucthread_run == true))
	{
		ret = recv(clientsock, &extparams,
			sizeof(struct cash_params), 0);
		if (!ret) {
			ALOGE("Cannot receive data from client");
			goto reloop;
		}

		if (ret != sizeof(struct cash_params)) {
			ALOGE("Received data size mismatch!!");
			goto reloop;
		} else ret = 0;

		cash_reply = cash_dispatch(&extparams);

retry_send:
		retry++;
		ret = send(clientsock, &cash_reply,
			sizeof(cash_reply), 0);
		if (ret == -1) {
			cash_reply = -EINVAL;
			if (retry < 50)
				goto retry_send;
			ALOGE("ERROR: Cannot send reply!!!");
			goto reloop;
		} else retry = 0;

		if (clientsock)
			close(clientsock);
	}

	ALOGI("Camera Augmented Sensing Helper Server terminated.");
	pthread_exit((void*)((int)0));
}

static int manage_cashsvr(bool start)
{
	int ret;
	struct stat st = {0};

	if (start == false) {
		ucthread_run = false;
		if (clientsock) {
			shutdown(clientsock, SHUT_RDWR);
			close(clientsock);
		}
		if (sock) {
			shutdown(sock, SHUT_RDWR);
			close(sock);
		}

		return 0;
	}

	ucthread_run = true;

	/* Create folder, if doesn't exist */
	if (stat(CASHSERVER_DIR, &st) == -1) {
		mkdir(CASHSERVER_DIR, 0773);
	}

	/* Get socket in the UNIX domain */
	sock = socket(PF_UNIX, SOCK_SEQPACKET, 0);
	if (sock < 0) {
		ALOGE("Could not create the socket");
		return -EPROTO;
	}

	/* Create address */
	memset(&server_addr, 0, sizeof(struct sockaddr_un));
	server_addr.sun_family = AF_UNIX;
	strcpy(server_addr.sun_path, CASHSERVER_SOCKET);

	/* Free the existing socket file, if any */
	unlink(CASHSERVER_SOCKET);

	/* Bind the address to the socket */
	ret = bind(sock, (struct sockaddr*)&server_addr,
			sizeof(struct sockaddr_un));
	if (ret != 0) {
		ALOGE("Cannot bind socket");
		return -EINVAL;
	}

	/* Set socket permissions */
	chown(server_addr.sun_path, AID_ROOT, AID_SYSTEM);
	chmod(server_addr.sun_path, 0666);

	/* Listen on this socket */
	ret = listen(sock, CASHSERVER_MAXCONN);
	if (ret != 0) {
		ALOGE("Cannot listen on socket");
		return ret;
	}

	ret = pthread_create(&cashsvr_thread, NULL, cashsvr_looper, NULL);
	if (ret != 0) {
		ALOGE("Cannot create CASH thread");
		return -ENXIO;
	}

	return 0;
}

/*
 * cash_autofocus_get_coeff - Prepares the focus algorithm in advance
 *			       by getting the polynomial regression's
 *			       correlation coefficient for the provided
 *			       input-focus table.
 *
 * \return Returns zero or negative errno.
 */
static int cash_autofocus_get_coeff(void)
{
	uint32_t i;
	struct pair_data *pairs;
	double coeff;
	int rs = 3 * FOCTBL_POLYREG_DEGREE;

	if (focus_conf.table == NULL)
		return -3;

	pairs = (struct pair_data*)calloc(focus_conf.num_steps+1,
				 sizeof(struct pair_data));
	if (pairs == NULL) {
		ALOGE("Memory exhausted. Cannot write focus table");
		return -4;
	}

	for (i = 0; i <= focus_conf.num_steps; i++) {
		pairs[i].x = focus_conf.table[i].input_val;
		pairs[i].y = focus_conf.table[i].focus_step;
		ALOGD("Table x:%.2f  y:%.2f",
			pairs[i].x, pairs[i].y);
	}

	ALOGE("Got %d pairs", focus_conf.num_steps);

	focus_conf.terms = (double*)calloc(rs, sizeof(double));

	compute_coefficients(pairs, focus_conf.num_steps,
				cash_conf.polyreg_degree, focus_conf.terms);
	if (focus_conf.terms == NULL) {
		ALOGE("FATAL: Cannot compute coefficients.");
		return -5;
	}

	for (i = 0; i < focus_conf.num_steps; i++)
		ALOGE("Term%d: %.10f",i, focus_conf.terms[i]);

	coeff = corr_coeff(pairs, focus_conf.num_steps, focus_conf.terms);
	if (coeff > 1.0f)
		ALOGW("WARNING! The correlation coefficient is >1!!");
	else if (coeff == 0.0f)
		ALOGW("WARNING! The correlation coefficient is ZERO!!");

	ALOGD("Correlation coefficient: %.10f", coeff);

	ALOGI("Auto-Focus Polynomial Regression coordinates loaded.");

	return 0;
}

int cashsvr_configure(void)
{
        char propbuf[PROPERTY_VALUE_MAX];
	int rc = 0;

	cash_conf.tof_min = 0;
	cash_conf.tof_max = 1030;
	cash_conf.tof_hyst = TOF_STABILIZATION_HYST_MM;
	cash_conf.tof_max_runs = TOF_STABILIZATION_DEF_RUNS;
	cash_conf.polyreg_degree = FOCTBL_POLYREG_DEGREE;
	cash_conf.polyreg_extra = 0;
	cash_conf.use_tof_stabilized = 0;
	cash_conf.disable_tof = 0;

	rc = parse_cash_xml_data(CASHSERVER_CONF_FILE, "tof_focus",
				&focus_conf, &cash_conf);
	if (rc < 0) {
		ALOGE("Cannot parse configuration for ToF assisted AF");
	} else {
		rc = cash_input_tof_init();
		if (rc < 0)
			ALOGW("Cannot open ToF. Ranging will be unavailable");
		cash_autofocus_get_coeff();
	}

	/*
	 * Use stabilized read with score system or otherwise do
	 * a single reading of the ToF distance measurement and
	 * instantly trust it if this configuration is zero.
	 */
        property_get("persist.cash.tof.stabilized", propbuf, "0");
	if (atoi(propbuf) > 0)
		cash_conf.use_tof_stabilized = 1;

	/*
	 * Disable ToF functionality if this configuration
	 * option is 1.
	 */
        property_get("persist.cash.tof.disable", propbuf, "0");
	if (atoi(propbuf) > 0)
		cash_conf.disable_tof = 1;

	return rc;
}

int main(void)
{
	int rc;

	ALOGI("Initializing Camera Augmented Sensing Helper Server...");

	rc = cashsvr_configure();
	if (rc != 0)
		ALOGW("Configuration went wrong. You will experience issues.");
start:
	/* All devices opened and configured. Start! */
	rc = manage_cashsvr(true);
	if (rc == 0) {
		ALOGI("Camera Augmented Sensing Helper Server started");
	} else {
		ALOGE("Could not start Camera Augmented Sensing Helper Server");
		goto err;
	}

	pthread_join(cashsvr_thread, (void**)&rc);
	if (rc == 0)
		goto start;

	return rc;
err:
	close(serport);
	ALOGE("Camera Augmented Sensing Helper Server initialization FAILED.");

	return rc;
}
