/*
 * CASH! Camera Augmented Sensing Helper
 * a multi-sensor camera helper server
 *
 * RGBC-IR module
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

#define LOG_TAG			"CASH_RGBC_INPUT"

#include <sys/poll.h>
#include <sys/epoll.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
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
#include "cash_input_rgbc.h"
#include "cash_ext.h"

#define TCS3490_STR		"AMS TCS3490 Sensor"
#define TCS3490_ALS_ITIME	"127"
#define TCS3490_ALS_GAIN_LOW	"1"
#define TCS3490_ALS_GAIN_MID	"4"
#define TCS3490_ALS_GAIN_HIGH	"8"

static int tcsvl_fd;

static char *rgbc_chip_power_path;
static char *rgbc_power_state_path;
static char *rgbc_gain_path;
static char *rgbc_Itime_path;
static bool rgbc_enabled = false;

struct cash_tcs3490 tcsvl_status;

#define UNUSED __attribute__((unused))

#define LEN_NAME	4
#define LEN_CHIP_POW	8
#define LEN_POWER_STATE	15
#define LEN_ITIME	9
#define LEN_GAIN	9

int cash_rgbc_enable(bool enable)
{
	int rc;

	if (rgbc_chip_power_path == NULL || rgbc_power_state_path == NULL)
		return -1;

	/* Reset the readings to start fresh */
	tcsvl_status.red = -1;
	tcsvl_status.green = -1;
	tcsvl_status.blue = -1;
	tcsvl_status.clear = -1;

	/* enabling/disabling requires writing to sysfs twice
	 * chip_power to power up/down the chip
	 * als_power_state to start/stop the work
	 */
	if (enable) {
		rc = cash_set_parameter(rgbc_chip_power_path, "1", 1);
		if (!rc)
			rc = cash_set_parameter(rgbc_power_state_path, "1", 1);
		// set default gain and Itime if sensor was just enabled
		if (!rc)
			rc = cash_set_parameter(rgbc_gain_path, TCS3490_ALS_GAIN_LOW, 1);
		if (!rc)
			rc = cash_set_parameter(rgbc_Itime_path, TCS3490_ALS_ITIME, 3);
	} else {
		rc = cash_set_parameter(rgbc_power_state_path, "0", 1);
		if (!rc)
			rc = cash_set_parameter(rgbc_chip_power_path, "0", 1);
	}

	if (rc) {
		ALOGW("ERROR! Cannot %sable RGBC!", enable ? "en" : "dis");
		goto end;
	}

	rc = 0;

end:
	usleep(100000);
	rgbc_enabled = enable;

	return rc;
}

static int cash_rgbc_sys_init(int devno, int plen)
{
	int rc = 0;

	// allocate memory for all paths needed
	rgbc_chip_power_path = (char*)calloc(plen + LEN_CHIP_POW, sizeof(char));
	if (rgbc_chip_power_path == NULL) {
		ALOGE("Memory exhausted. Cannot allocate.");
		return -3;
	}

	rgbc_power_state_path = (char*)calloc(plen + LEN_POWER_STATE, sizeof(char));
	if (rgbc_power_state_path == NULL) {
		ALOGE("Memory exhausted. Cannot allocate.");
		return -3;
	}

	rgbc_Itime_path = (char*)calloc(plen + LEN_ITIME, sizeof(char));
	if (rgbc_Itime_path == NULL) {
		ALOGE("Memory exhausted. Cannot allocate.");
		free(rgbc_Itime_path);
		return -3;
	}

	rgbc_gain_path = (char*)calloc(plen + LEN_GAIN, sizeof(char));
	if (rgbc_gain_path == NULL) {
		ALOGE("Memory exhausted. Cannot allocate.");
		free(rgbc_gain_path);
		return -3;
	}

	// populate the paths
	snprintf(rgbc_chip_power_path, plen + LEN_CHIP_POW,
		"%s%d/chip_pow", sysfs_input_str, devno);
	snprintf(rgbc_power_state_path, plen + LEN_POWER_STATE,
		"%s%d/als_power_state", sysfs_input_str, devno);
	snprintf(rgbc_Itime_path, plen + LEN_ITIME,
		"%s%d/als_Itime", sysfs_input_str, devno);
	snprintf(rgbc_gain_path, plen + LEN_GAIN,
		"%s%d/als_gain", sysfs_input_str, devno);

	
	// call chown on the paths to allow access after context switch
	rc = cash_set_permissions(rgbc_chip_power_path, "system", "input");
	rc += cash_set_permissions(rgbc_power_state_path, "system", "input");
	rc += cash_set_permissions(rgbc_Itime_path, "system", "input");
	rc += cash_set_permissions(rgbc_gain_path, "system", "input");
	if (rc < 0)
		return rc;

	return 0;
}

/* TODO: Use IOCTL EVIOCGNAME as a waaaay better way */
static int cash_input_rgbc_find_inputdev(int maxdevs, int idev_len, char* idev_name)
{
	int fd, plen, rlen, rc, i;
	int plen_xtra = 3;
	char buf[254];
	char* path;

	/*
	 * Avoid checking more than 99 input devices,
	 * because... really? :)
	 */
	if (maxdevs > 99)
		return -1;

	if (maxdevs > 9)
		plen_xtra++;

	plen = strlen(sysfs_input_str) + plen_xtra;
	path = (char*) calloc(plen + LEN_NAME, sizeof(char));
	if (path == NULL) {
		ALOGE("Out of memory. Cannot allocate.");
		rc = -1;
		goto end;
	}

	for (i = 0; i <= maxdevs; i++) {
		rc = snprintf(path, plen + LEN_NAME, "%s%d/name",
							sysfs_input_str, i);

		if (fd > 0)
			close(fd);

		fd = open(path, (O_RDONLY | O_NONBLOCK));
		if (fd < 0) {
			ALOGD("Cannot open %s", path);
			continue;
		}

		rlen = read(fd, buf, idev_len);
		if (!rlen)
			continue;

		/*
		 * If the string length doesn't match, avoid doing
		 * expensive stuff, as the string cannot be the same.
		 * Also consider some mistakes and compare it with
		 * the name length - 1.
		 * That will cover corner cases and will not evaluate
		 * erroneously positive too many times...
		 */
		if (rlen < idev_len - 1)
			continue;

		rc = strncmp(buf, idev_name, idev_len - 1);
		if (rc == 0) {
			close(fd);

			rc = cash_rgbc_sys_init(i, plen);
			if (rc < 0)
				goto end;

			return i;
		}
	}
end:
	return rc;
}

int cash_input_rgbc_thr_read(struct cash_tcs3490 *tcsvl_cur,
			int rgbc_fd)
{
	struct input_event evt[16];
	int i, len, rc;
	uint16_t type, code;
	int32_t value;

	rc = read(rgbc_fd, &evt, sizeof(evt));
	if (!rc) {
		return rc;
	}

	len = rc / sizeof(struct input_event);

	for (i = 0; i < len; i++) {
		type = evt[i].type;
		code = evt[i].code;
		value = evt[i].value;

		switch (code) {
			case ABS_MISC:
				if (value >= 0) {
					tcsvl_cur->clear = value;
				}
				break;
			case ABS_HAT0X:
				if (value >= 0) {
					tcsvl_cur->red = value;
				}
				break;
			case ABS_HAT0Y:
				if (value >= 0) {
					tcsvl_cur->green = value;
				}
				break;
			case ABS_HAT1X:
				if (value >= 0) {
					tcsvl_cur->blue = value;
				}
				break;
			case ABS_HAT1Y:
				if (value >= 0) {
					tcsvl_cur->ir = value;
				}
				break;
			default:
				break;
		}
	}

	ALOGV("RGBC VALUES R:%d G:%d B:%d C:%d IR:%d", tcsvl_cur->red, tcsvl_cur->green, tcsvl_cur->blue, tcsvl_cur->clear, tcsvl_cur->ir);
	return 0;
}

int cash_rgbc_read_inst(struct cash_tcs3490 *tcsvl_final)
{
	/* Thread not running, we'd read nothing good here! */
	if (!cash_thread_run[THREAD_RGBC])
		return -1;

	/* Sensor is disabled, what are we trying to read?! */
	if (!rgbc_enabled)
		return -1;

	/* No reading available */
	if (tcsvl_status.clear < 0) {
		ALOGE("ToF: No reading! clear %d", tcsvl_status.clear);
		return -1;
	}

	tcsvl_final->clear = tcsvl_status.clear;

	/* Return a fake score of 1 */
	return 1;
}

static void cash_input_rgbc_thread(void)
{
	int ret;
	int i;
	struct epoll_event pevt[10];

	cash_rgbc_enable(true);

	ALOGD("RGBC Thread started");

	while (cash_thread_run[THREAD_RGBC]) {
		ret = epoll_wait(cash_pollfd[FD_RGBC], pevt,
					10, cash_pfdelay_ms[FD_RGBC]);
		for (i = 0; i < ret; i++) {
			if (pevt[i].events & EPOLLERR ||
			    pevt[i].events & EPOLLHUP ||
			    !(pevt[i].events & EPOLLIN))
				continue;

			if (cash_pollevt[FD_RGBC].data.fd)
				cash_input_rgbc_thr_read(&tcsvl_status,
					cash_pollevt[FD_RGBC].data.fd);
		}
	}

	cash_rgbc_enable(false);

	pthread_exit((void*)((int)0));
}

struct thread_data cash_rgbc_thread_data = {
	.thread_no = THREAD_RGBC,
	.thread_func = cash_input_rgbc_thread
};

int cash_input_rgbc_start(bool start)
{
	return cash_input_threadman(start, &cash_rgbc_thread_data);
}

bool cash_input_is_rgbc_alive(void)
{
	return cash_thread_run[THREAD_RGBC];
}

int cash_input_rgbc_init(__attribute__((unused))struct cash_tamisc_calib_params *calib_params)
{
	int dlen, evtno, rc;
	char *devname, *devpath;

	dlen = strlen(TCS3490_STR);
	devname = (char*) calloc(dlen, sizeof(char));
	snprintf(devname, dlen, "%s", TCS3490_STR);

	evtno = cash_input_rgbc_find_inputdev(ITERATE_MAX_DEVS, dlen, devname);
	if (evtno < 0)
		return -1;

	dlen = strlen(devfs_input_str) + 3;
	if (evtno > 9)
		dlen++;

	devpath = (char*) calloc(dlen, sizeof(char));
	if (devpath == NULL)
		return -1;

	snprintf(devpath, dlen, "%s%d", devfs_input_str, evtno);


	tcsvl_fd = open(devpath, (O_RDONLY | O_NONBLOCK));
	if (tcsvl_fd < 0) {
		ALOGE("Error: cannot open the %s input device at %s.",
			devname, devpath);
		return -1;
	}

	cash_pollfd[FD_RGBC] = epoll_create1(0);
	if (cash_pollfd[FD_RGBC] == -1) {
		ALOGE("Error: Cannot create epoll descriptor");
		return -1;
	}

	cash_pfds[FD_RGBC].fd = tcsvl_fd;
	cash_pfds[FD_RGBC].events = POLLIN;
	cash_pfdelay_ms[FD_RGBC] = 1000;

	cash_pollevt[FD_RGBC].events = POLLIN;
	cash_pollevt[FD_RGBC].data.fd = tcsvl_fd;

	rc = epoll_ctl(cash_pollfd[FD_RGBC], EPOLL_CTL_ADD,
					tcsvl_fd, &cash_pollevt[FD_RGBC]);
	if (rc) {
		ALOGE("Cannot add epoll control");
		return -1;
	}

	cash_thread_run[THREAD_RGBC] = false;

	return 0;
}

