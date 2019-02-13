/*
 * CASH! Camera Augmented Sensing Helper
 * a multi-sensor camera helper server
 *
 * Input devices module
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

#include <sys/poll.h>
#include <sys/epoll.h>

#define ITERATE_MAX_DEVS	9

enum thread_number {
	THREAD_TOF,
	THREAD_RGBC,
	THREAD_MAX
};


enum fd_number {
	FD_TOF,
	FD_RGBC,
	FD_MAX
};

struct thread_data {
	enum thread_number thread_no;
	void *thread_func;
};

bool cash_thread_run[THREAD_MAX];
pthread_t cash_pthreads[THREAD_MAX];
struct pollfd cash_pfds[FD_MAX];
struct epoll_event cash_pollevt[FD_MAX];
int cash_pollfd[FD_MAX];
int cash_pfdelay_ms[FD_MAX];

static const char sysfs_input_str[] = "/sys/class/input/input";
static const char devfs_input_str[] = "/dev/input/event";

int cash_input_threadman(bool start, struct thread_data *thread_data);
int cash_set_parameter(char* path, char* value, int value_len);
int cash_set_permissions(char* fpath, char* str_uid, char* str_gid);
