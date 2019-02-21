/*
 * CASH! Camera Augmented Sensing Helper
 * a multi-sensor camera helper server
 *
 * Input devices module
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

#include <errno.h>
#include <stdbool.h>
#include <pthread.h>
#include <unistd.h>
#include <pwd.h>
#include <log/log.h>

#include "cash_input_common.h"


/* Start/stop threads */
int cash_input_threadman(bool start, struct thread_data *thread_data)
{
	int ret = -1;
	int thread_no = thread_data->thread_no;

	if (start == false) {
		static void *join_retval;  // Unused
		/* Instruct thread to stop: */
		cash_thread_run[thread_no] = false;
		/* Wait until thread really exits: */
		pthread_join(cash_pthreads[thread_no], join_retval);
		return 0;
	};

	cash_thread_run[thread_no] = true;

	if (thread_no < THREAD_MAX) {
		ret = pthread_create(&cash_pthreads[thread_no], NULL,
				thread_data->thread_func, NULL);
		if (ret != 0) {
			ALOGE("Cannot create thread with number %d", thread_no);
			return -ENXIO;
		}
	}

	return ret;
}

int cash_set_parameter(char* path, char* value, int value_len) {
	int fd, rc;

	fd = open(path, O_WRONLY);
	if (fd < 0) {
		ALOGD("Cannot open %s", path);
		return -1;
	}

	rc = write(fd, value, value_len);
	if (rc < value_len) {
		ALOGW("ERROR! Cannot write value %s to %s", value, path);
		return -1;
	}
	close(fd);
	return 0;
}

/*
 * cash_set_permissions - Common utility function to set file permissions
 *
 * \param fpath - Path of the file or directory
 * \param str_uid - Username string
 * \param str_gid - Group name string
 *
 * \return Returns zero for success or -1 for error.
 */
int cash_set_permissions(char* fpath, char* str_uid, char* str_gid)
{
	struct passwd *pwd;
	struct passwd *grp;
	uid_t uid;
	gid_t gid;
	int rc = 0;

	/* get user and group to call chown */
	pwd = getpwnam(str_uid);
	if (pwd == NULL) {
		ALOGD("failed to get uid for %s", str_uid);
		return -1;
	}

	grp = getpwnam(str_gid);
	if (grp == NULL) {
		ALOGD("failed to get gid for %s", str_gid);
		return -1;
	}

	uid = pwd->pw_uid;
	gid = grp->pw_gid;

	rc = chown(fpath, uid, gid);
	if (rc == -1)
		ALOGE("Cannot set permissions for %s", fpath);

	return rc;
}
