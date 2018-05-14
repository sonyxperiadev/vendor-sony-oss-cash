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

#include <stdbool.h>

/* CASH Server definitions */
#define CASHSERVER_DIR			"/dev/socket/cashsvr/"
#define CASHSERVER_SOCKET		CASHSERVER_DIR "cashsvr"
#define CASHSERVER_MAXCONN		10

#define CASHSERVER_CONF_FILE		"/vendor/etc/tof_focus_calibration.xml"

typedef enum {
	OP_INITIALIZE = 0,
	OP_TOF_START,
	OP_CHECK_TOF_RANGE,
	OP_FOCUS_GET,
	OP_MAX,
} cash_svr_ops_t;

struct cash_cached_data {
	bool light_suspended;
	uint8_t light;
	uint8_t focus;
	uint8_t keystone;
};

struct cash_foctbl_entry {
	int32_t input_val;
	int32_t focus_step;
};

struct cash_focus_params {
	struct cash_foctbl_entry *table;
	unsigned int num_steps;
	double *terms;
};

struct cash_configuration {
	int32_t tof_min;
	int32_t tof_max;
	int32_t tof_hyst;
	int32_t tof_max_runs;
	int32_t polyreg_degree;
	int32_t polyreg_extra;
	int8_t  use_tof_stabilized;
	int8_t  disable_tof;
};

struct cash_focus_state {
	int16_t far_max;
	int16_t near_max;
	int16_t cur_focus;
};

struct cash_params {
	int32_t operation;
	int32_t value;
};

int parse_cash_xml_data(char* filepath, char* node, 
			struct cash_focus_params *cash_focus,
			struct cash_configuration *cash_config);

#define REPLY_FOCUS_CUSTOM_LEN		7
#define REPLY_SHORT_FOCUS_LEN		2
#define FOCUS_PROCESSING_MAX_PASS	6
#define FOCTBL_POLYREG_DEGREE		5



