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

#ifndef CASHSVR_EXT_H
#define CASHSVR_EXT_H

struct exptime_iso_tpl {
	int64_t exptime;
	int32_t iso;
};

int cash_tof_start(int value);
int cash_is_tof_in_range(void);
int32_t cash_get_focus(void);

int cash_rgbc_start(int value);
int cash_is_rgbc_in_range(void);
struct exptime_iso_tpl cash_get_exptime_iso(void);

#endif
