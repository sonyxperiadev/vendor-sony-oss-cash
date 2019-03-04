/*
 * CASH! Camera Augmented Sensing Helper
 * a multi-sensor camera helper server
 *
 * Copyright (C) 2018-2019 AngeloGioacchino Del Regno <kholk11@gmail.com>
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


#define LOG_TAG                 "CASH_MISCTA_READER"

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <assert.h>
#include <string.h>
#include <unistd.h>
#include <linux/input.h>

#include <cutils/android_filesystem_config.h>
#include <log/log.h>

#include "cash_private.h"
#include "cash_input_common.h"
#include "cash_ext.h"

struct miscta_link {
	void *ta_handle;
	int (*ta_open)(uint8_t p, uint8_t m, uint8_t c);
	int (*ta_close)(void);
	int (*ta_getsize)(uint32_t id, uint32_t *size);
	int (*ta_read)(uint32_t id, void *buf, uint32_t size);
};
static struct miscta_link miscta_lnk;

static int cash_miscta_wire_up(void)
{
	miscta_lnk.ta_handle = dlopen(CASHSERVER_LIB_TA, RTLD_NOW);
	if (!miscta_lnk.ta_handle) {
		ALOGE("%s: DLOPEN failed for %s", __func__, CASHSERVER_LIB_TA);
		return -1;
	}

	miscta_lnk.ta_open = dlsym(miscta_lnk.ta_handle, "ta_open");
	if (!miscta_lnk.ta_open) {
		ALOGE("%s: DLSYM failed for ta_open", __func__);
		return -1;
	}

	miscta_lnk.ta_close = dlsym(miscta_lnk.ta_handle, "ta_close");
	if (!miscta_lnk.ta_close) {
		ALOGE("%s: DLSYM failed for ta_close", __func__);
		return -1;
	}

	miscta_lnk.ta_getsize = dlsym(miscta_lnk.ta_handle, "ta_getsize");
	if (!miscta_lnk.ta_open) {
		ALOGE("%s: DLSYM failed for ta_getsize", __func__);
		return -1;
	}

	miscta_lnk.ta_read = dlsym(miscta_lnk.ta_handle, "ta_read");
	if (!miscta_lnk.ta_read) {
		ALOGE("%s: DLSYM failed for ta_read", __func__);
		return -1;
	}

	return 0;
}

static int cash_miscta_read_unit(uint32_t id, void *data_out)
{
	uint32_t ta_unit_sz = 0;
	int rc;

	rc = miscta_lnk.ta_getsize(id, &ta_unit_sz);
	if (ta_unit_sz == 0) {
		ALOGE("FATAL: Cannot get MiscTA unit %u size", id);
		return -1;
	};

	rc = miscta_lnk.ta_read(id, data_out, ta_unit_sz);
	if (rc) {
		ALOGE("WARNING: MiscTA unit %u read returns %d", id, rc);
	};

	return 0;
}

/*
 * cash_miscta_read_store_params - Read configuration from MiscTA and
 *				   store it to the CASH data file
 *
 * Call this function with force == true to force params read and
 * store even if the calibration file already exists.
 */
static int cash_miscta_read_store_params(bool force)
{
	struct cash_tamisc_calib_params tacfg;
	struct stat st = {0};
	bool read_error = false;
	int rc, fd, i;

	if ((stat(CASHSERVER_CALDATA_FILE, &st) == 0) && (!force)) {
		/* If file exists and not forcing re-read, just exit. */
		return 0;
	}

	fd = open(CASHSERVER_CALDATA_FILE, O_WRONLY | O_CREAT, 0664);
	if (fd < 0) {
		ALOGE("FATAL: Cannot open/create %s", CASHSERVER_CALDATA_FILE);
		return -1;
	}

	/* Wire up the function pointers */
	rc = cash_miscta_wire_up();
	if (rc)
		return rc;

	/* This can be painfully slow */
	for (i = 0; i < 12; i++) {
		rc = miscta_lnk.ta_open(2, 0x1, 1);
		if (!rc)
			break;

		sleep(5);
	}
	if (rc) {
		ALOGE("FATAL: Cannot open MiscTA");
		return -1;
	}

	rc = 0;

	rc = cash_miscta_read_unit(TA_UNIT_RGBCIR_CAPS1, &tacfg.rgbcir_caps1);
	rc += cash_miscta_read_unit(TA_UNIT_RGBCIR_CAPS2, &tacfg.rgbcir_caps2);
	if (rc) {
		ALOGE("FATAL: Cannot read RGBCIR config from MiscTA\n");
		read_error = true;
	}

	/* Number of SPADs */
	rc = cash_miscta_read_unit(TA_UNIT_TOF_SPAD_NUM, &tacfg.tof_spad_num);

	/* Type of SPADs: Aperture Type == 1 - Non-Aperture Type == 0 */
	rc += cash_miscta_read_unit(TA_UNIT_TOF_SPAD_TYPE,
					&tacfg.tof_spad_type);

	/* Calibration Data offset, expressed in micrometers */
	rc += cash_miscta_read_unit(TA_UNIT_TOF_UM_OFFSET,
					&tacfg.tof_um_offset);
	if (rc) {
		ALOGE("FATAL: Cannot read ToF config from MiscTA\n");
		ALOGE("Your ToF sensor may work suboptimally.\n");
		read_error = true;
	}

	miscta_lnk.ta_close();

	/* Write the struct into the calibration file */
	write(fd, &tacfg, sizeof(struct cash_tamisc_calib_params));
	close(fd);

	if (read_error)
		return -1;

	return rc;
}

int cash_miscta_init_params(struct cash_tamisc_calib_params *conf)
{
	struct stat st = {0};
	int ret, fd;

	/* The folder has to be created in OS init scripts. */
	ret = stat(CASHSERVER_DATASTORE_DIR, &st);
	if (ret == -1) {
		ALOGW("%s does not exist. Bailing out.",
				CASHSERVER_DATASTORE_DIR);
		return -1;
	}

	ret = cash_miscta_read_store_params(false);
	if (ret)
		return ret;

	fd = open(CASHSERVER_CALDATA_FILE, O_RDONLY);
	if (fd < 0)
		return -1;

	read(fd, conf, sizeof(struct cash_tamisc_calib_params));
	close(fd);

	return 0;
}
