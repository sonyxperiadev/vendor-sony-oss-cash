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

#define CASHSERVER_TOF_CONF_FILE	"/vendor/etc/tof_focus_calibration.xml"
#define CASHSERVER_RGBC_CONF_FILE	"/vendor/etc/cash_expcol_calibration.xml"

#define CASHSERVER_DATASTORE_DIR	"/data/vendor/cashsvr/"
#define CASHSERVER_CALDATA_FILE		CASHSERVER_DATASTORE_DIR "miscta_caldata.bin"

#define CASHSERVER_LIB_TA		"libta.so"
#define TA_UNIT_RGBCIR_CAPS1		4880
#define TA_UNIT_RGBCIR_CAPS2		4881
#define TA_UNIT_TOF_SPAD_NUM		4882
#define TA_UNIT_TOF_SPAD_TYPE		4883
#define TA_UNIT_TOF_UM_OFFSET		4884

typedef enum {
	OP_INITIALIZE = 0,
	OP_TOF_START,
	OP_CHECK_TOF_RANGE,
	OP_FOCUS_GET,
	OP_RGBC_START,
	OP_CHECK_RGBC_RANGE,
	OP_EXPTIME_ISO_GET,
	OP_MAX,
} cash_svr_ops_t;

struct cash_polyreg_tbl_entry {
	int32_t input_val;
	int32_t output_val;
};

struct cash_polyreg_params {
	struct cash_polyreg_tbl_entry *table;
	unsigned int num_steps;
	double *terms;
};

struct cash_configuration {
	int32_t tof_min;
	int32_t tof_max;
	int32_t tof_hyst;
	int32_t tof_max_runs;
	int32_t tof_polyreg_degree;
	int32_t tof_polyreg_extra;
	int8_t  use_tof_stabilized;
	int8_t  disable_tof;
	int32_t rgbc_clear_min;
	int32_t rgbc_clear_max;
	int32_t rgbc_polyreg_degree;
	int32_t rgbc_polyreg_extra;
	int8_t  disable_rgbc;
	int64_t *exposure_times;
	int32_t nexposure_times;
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

struct cash_tamisc_calib_params {
	uint16_t rgbcir_caps1[5];
	uint16_t rgbcir_caps2[5];
	uint32_t tof_um_offset;
	uint32_t tof_spad_num;
	uint8_t tof_spad_type;
};

struct cash_response {
	bool retval;
	int32_t focus_step;
	int64_t exptime;
	int32_t iso;
};

int parse_cash_tof_xml_data(char* filepath, char* node, 
			struct cash_polyreg_params *cash_focus,
			struct cash_configuration *cash_config);

int parse_cash_rgbc_xml_data(char* filepath, char* node, 
			struct cash_polyreg_params *cash_rgbc_clear_iso,
			struct cash_configuration *cash_config);

int cash_miscta_init_params(struct cash_tamisc_calib_params *conf);

#define REPLY_FOCUS_CUSTOM_LEN		7
#define REPLY_SHORT_FOCUS_LEN		2
#define FOCUS_PROCESSING_MAX_PASS	6
#define FOCTBL_POLYREG_DEGREE		5



