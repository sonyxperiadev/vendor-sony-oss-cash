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

#define TOF_DEFAULT_MIN_MM			0
#define TOF_DEFAULT_MAX_MM			1030
#define TOF_STABILIZATION_DEF_RUNS		4
#define TOF_STABILIZATION_WAIT_MS		10
#define TOF_STABILIZATION_HYST_MM		7
#define TOF_STABILIZATION_MATCH_NO		3

struct cash_vl53l0 {
	int range_mm;
	int distance;
	int range_status;
	int measure_mode;
};

int cash_input_tof_read(struct cash_vl53l0 *stmvl_cur,
	uint16_t want_code);
int cash_tof_read_stabilized(
	struct cash_vl53l0 *stmvl_final,
	int runs, int nmatch, int sleep_ms, int hyst);
int cash_tof_read_inst(struct cash_vl53l0 *stmvl_final);
int cash_tof_thr_read_stabilized(
	struct cash_vl53l0 *stmvl_final,
	int runs, int nmatch, int sleep_ms, int hyst);
int cash_input_tof_start(bool start);
bool cash_input_is_tof_alive(void);
int cash_input_tof_init(struct cash_tamisc_calib_params *calib_params);

