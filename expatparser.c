/*
 * CASH! Camera Augmented Sensing Helper
 * a multi-sensor camera helper server
 *
 * XML Configuration Parser
 *
 * Copyright (C) 2018 AngeloGioacchino Del Regno <kholk11@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *	  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#define LOG_TAG "CASH-XMLParser"

#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <expat.h>

#include "cash_private.h"

#include <cutils/properties.h>
#include <log/log.h>

#define UNUSED __attribute__((unused))

#define CASH_MAX_POLYREG_TBL_ENTRIES	500

static short xml_depth = 0;
static short parse = -1;
static char* main_node;

static char millimeters[255];
static char focus_steps[255];
static char tof_min[50], tof_max[50], tof_hyst[4], tof_max_runs[4];
static char tof_polyreg_degree[3], tof_polyreg_extra[3];
static char clear_values[255];
static char iso_values[255];
static char exposure_times[255];
static char rgbc_clear_min[50], rgbc_clear_max[50];
static char rgbc_polyreg_degree[3], rgbc_polyreg_extra[3];

struct cash_polyreg_params focus_params;
struct cash_polyreg_params clear_iso_params;

void parseElm(const char *elm, const char **attr)
{
	int i;

	if (strcmp("focus", elm) == 0) {
		for (i = 0; attr[i]; i += 2) {
			if (strcmp("millimeters", attr[i]) == 0)
				strcpy(millimeters, attr[i+1]);
			else if (strcmp("focus_step", attr[i]) == 0)
				strcpy(focus_steps, attr[i+1]);
		}
	}

	if (strcmp("polyreg_tuning", elm) == 0) {
		for (i = 0; attr[i]; i += 2) {
			if (strcmp("degree", attr[i]) == 0)
				strcpy(tof_polyreg_degree, attr[i+1]);
			else if (strcmp("extra", attr[i]) == 0)
				strcpy(tof_polyreg_extra, attr[i+1]);
		}
	}

	if (strcmp("ranging_limits", elm) == 0) {
		for (i = 0; attr[i]; i += 2) {
			if (strcmp("min_range", attr[i]) == 0)
				strcpy(tof_min, attr[i+1]);
			else if (strcmp("max_range", attr[i]) == 0)
				strcpy(tof_max, attr[i+1]);
		}
	}

	if (strcmp("ranging_params", elm) == 0) {
		for (i = 0; attr[i]; i += 2) {
			if (strcmp("hysteresis", attr[i]) == 0)
				strcpy(tof_hyst, attr[i+1]);
			else if (strcmp("max_runs", attr[i]) == 0)
				strcpy(tof_max_runs, attr[i+1]);
		}
	}

	if (strcmp("rgbc_clear_iso_exptime", elm) == 0) {
		for (i = 0; attr[i]; i += 2) {
			if (strcmp("clear_values", attr[i]) == 0)
				strcpy(clear_values, attr[i+1]);
			else if (strcmp("iso_values", attr[i]) == 0)
				strcpy(iso_values, attr[i+1]);
			else if (strcmp("exposure_times", attr[i]) == 0)
				strcpy(exposure_times, attr[i+1]);
		}
	}

	if (strcmp("rgbc_clear_limits", elm) == 0) {
		for (i = 0; attr[i]; i += 2) {
			if (strcmp("min_range", attr[i]) == 0)
				strcpy(rgbc_clear_min, attr[i+1]);
			else if (strcmp("max_range", attr[i]) == 0)
				strcpy(rgbc_clear_max, attr[i+1]);
		}
	}

	if (strcmp("rgbc_polyreg_tuning", elm) == 0) {
		for (i = 0; attr[i]; i += 2) {
			if (strcmp("degree", attr[i]) == 0)
				strcpy(rgbc_polyreg_degree, attr[i+1]);
			else if (strcmp("extra", attr[i]) == 0)
				strcpy(rgbc_polyreg_extra, attr[i+1]);
		}
	}
}

void startElm(void *data UNUSED, const char *elm, const char **attr)
{
	xml_depth++;

	if (strncmp(main_node, elm, strlen(main_node)) == 0)
		parse = xml_depth;

	if (parse > 0)
		parseElm(elm, attr);
}

void endElm(void *data UNUSED, const char *elm UNUSED)
{
	if ((parse > 0) && (parse == xml_depth))
		parse = -1;

	xml_depth--;
}

void str_handler(void *data, const char *str, int len)
{
	char *buf = malloc(len+1);

	strncpy(buf, str, len);
	buf[len+1] = '\0';

	data = (void*)buf;
}

int parse_cash_tof_xml_data(char* filepath, char* node,
			struct cash_polyreg_params *cash_focus,
			struct cash_configuration *cash_config)
{
	int ret, fd, count, sz, tmp;
	char *buf, *mend, *fend;
	struct stat st;
	XML_Parser pa;
	struct cash_polyreg_tbl_entry *tbl_entry = NULL;

	fd = open(filepath, O_RDONLY);
	if (fd < 0) {
		ALOGE("Cannot open configuration file!!!");
		return -ENOENT;
	}

	pa = XML_ParserCreate(NULL);

	stat(filepath, &st);
	sz = st.st_size;

	/* Security check: do NOT parse too big files */
	if (sz > 131072) {
		ALOGE("File is huge. Preventing parse as a security measure.");
		ret = -E2BIG;
		goto secfail;
	}

	buf = malloc(sizeof(char)*sz);

	memset(buf, 0, sz);
	count = read(fd, buf, (sz - 1));
	if (count < 0) {
		ALOGE("Cannot read configuration file!!!");
		ret = -EIO;
		goto end;
	}

	focus_params.table = malloc(CASH_MAX_POLYREG_TBL_ENTRIES *
			sizeof(struct cash_polyreg_tbl_entry));
	if (focus_params.table == NULL) {
		ALOGE("Out of memory. Cannot allocate.");
		ret = -ENOMEM;
		goto end;
	}
	focus_params.num_steps = 0;

	XML_SetElementHandler(pa, startElm, endElm);
	XML_SetCharacterDataHandler(pa, str_handler);

	main_node = node;

	if (XML_Parse(pa, buf, strlen(buf), XML_TRUE) == XML_STATUS_ERROR) {
		ALOGE("XML Parse error: %s\n", XML_ErrorString(
						XML_GetErrorCode(pa)));
		ret = -EINVAL;
		goto end;
	}
	ret = 0;

	/* Check values and fill the temporary struct */
	tbl_entry = &focus_params.table[focus_params.num_steps];

	tbl_entry->input_val = (int32_t)strtol(millimeters, &mend, 10);
	tbl_entry->output_val = (int32_t)strtol(focus_steps, &fend, 10);

	if (tbl_entry->input_val == 0 || tbl_entry->output_val == 0) {
		ret = -EINVAL;
		free(focus_params.table);
		goto end;
	}

	do {
		focus_params.num_steps++;
		tbl_entry = &focus_params.table[focus_params.num_steps];

		tbl_entry->input_val = (int)strtol(mend, &mend, 10);
		tbl_entry->output_val = (int)strtol(fend, &fend, 10);

		/* We've reached the end, farewell! */
		if (tbl_entry->input_val == 0 || tbl_entry->output_val == 0) {
			break;
		}
	} while (focus_params.num_steps < CASH_MAX_POLYREG_TBL_ENTRIES);

	/* All ok! */
	cash_focus->num_steps = focus_params.num_steps;
	cash_focus->table = focus_params.table;

	/* These configurations are not mandatory */
	tmp = (int32_t)strtol(tof_min, NULL, 10);
	if (tmp != 0) {
		cash_config->tof_min = tmp;
	}

	tmp = (int32_t)strtol(tof_max, NULL, 10);
	if (tmp != 0) {
		cash_config->tof_max = tmp;
	}

	tmp = (int32_t)strtol(tof_hyst, NULL, 10);
	if (tmp != 0) {
		cash_config->tof_hyst = tmp;
	}

	tmp = (int32_t)strtol(tof_max_runs, NULL, 10);
	if (tmp != 0) {
		cash_config->tof_max_runs = tmp;
	}

	tmp = (int32_t)strtol(tof_polyreg_degree, NULL, 10);
	if (tmp != 0) {
		cash_config->tof_polyreg_degree = tmp;
	}

	tmp = (int32_t)strtol(tof_polyreg_extra, NULL, 10);
	if (tmp != 0) {
		cash_config->tof_polyreg_extra = tmp;
	}

end:
	free(buf);
secfail:
	close(fd);
	XML_ParserFree(pa);

	return ret;
}

int parse_cash_rgbc_xml_data(char* filepath, char* node,
			struct cash_polyreg_params *cash_rgbc_clear_iso,
			struct cash_configuration *cash_config)
{
	int ret, fd, count, sz, tmp;
	char *buf, *mend, *fend;
	struct stat st;
	XML_Parser pa;
	struct cash_polyreg_tbl_entry *tbl_entry = NULL;
	int64_t *exposure_time = NULL;

	fd = open(filepath, O_RDONLY);
	if (fd < 0) {
		ALOGE("Cannot open configuration file!!!");
		return -ENOENT;
	}

	pa = XML_ParserCreate(NULL);

	stat(filepath, &st);
	sz = st.st_size;

	/* Security check: do NOT parse too big files */
	if (sz > 131072) {
		ALOGE("File is huge. Preventing parse as a security measure.");
		ret = -E2BIG;
		goto rgbc_secfail;
	}

	buf = malloc(sizeof(char)*sz);

	memset(buf, 0, sz);
	count = read(fd, buf, (sz - 1));
	if (count < 0) {
		ALOGE("Cannot read configuration file!!!");
		ret = -EIO;
		goto rgbc_end;
	}

	clear_iso_params.table = malloc(CASH_MAX_POLYREG_TBL_ENTRIES *
			sizeof(struct cash_polyreg_tbl_entry));
	if (clear_iso_params.table == NULL) {
		ALOGE("Out of memory. Cannot allocate.");
		ret = -ENOMEM;
		goto rgbc_end;
	}
	clear_iso_params.num_steps = 0;

	cash_config->exposure_times = malloc(CASH_MAX_POLYREG_TBL_ENTRIES *
			sizeof(int64_t));
	if (cash_config->exposure_times == NULL) {
		ALOGE("Out of memory. Cannot allocate.");
		ret = -ENOMEM;
		goto rgbc_end;
	}
	cash_config->nexposure_times = 0;

	XML_SetElementHandler(pa, startElm, endElm);
	XML_SetCharacterDataHandler(pa, str_handler);

	main_node = node;

	if (XML_Parse(pa, buf, strlen(buf), XML_TRUE) == XML_STATUS_ERROR) {
		ALOGE("XML Parse error: %s\n", XML_ErrorString(
						XML_GetErrorCode(pa)));
		ret = -EINVAL;
		goto rgbc_end;
	}
	ret = 0;

	/* Check values and fill the temporary struct */
	tbl_entry = &clear_iso_params.table[clear_iso_params.num_steps];

	tbl_entry->input_val = (int32_t)strtol(clear_values, &mend, 10);
	tbl_entry->output_val = (int32_t)strtol(iso_values, &fend, 10);

	if (tbl_entry->input_val == 0 || tbl_entry->output_val == 0) {
		ret = -EINVAL;
		free(clear_iso_params.table);
		free(cash_config->exposure_times);
		goto rgbc_end;
	}

	do {
		clear_iso_params.num_steps++;
		tbl_entry = &clear_iso_params.table[clear_iso_params.num_steps];

		tbl_entry->input_val = (int)strtol(mend, &mend, 10);
		tbl_entry->output_val = (int)strtol(fend, &fend, 10);

		/* We've reached the end, farewell! */
		if (tbl_entry->input_val == 0 || tbl_entry->output_val == 0) {
			break;
		}
	} while (clear_iso_params.num_steps < CASH_MAX_POLYREG_TBL_ENTRIES);

	/*
	 * Store exposure times found in the XML
	 * The times are stored in a list similar to clear and ISO values
	 */
	exposure_time = &cash_config->exposure_times[cash_config->nexposure_times];
	*exposure_time = (int32_t)strtol(exposure_times, &mend, 10);
	if (exposure_time == 0) {
		ret = -EINVAL;
		free(clear_iso_params.table);
		free(cash_config->exposure_times);
		goto rgbc_end;
	}

	do {
		cash_config->nexposure_times++;
		exposure_time = &cash_config->exposure_times[cash_config->nexposure_times];
		*exposure_time = (int32_t)strtol(mend, &mend, 10);

		/* We've reached the end, farewell! */
		if (exposure_time == 0) {
			break;
		}
	} while (cash_config->nexposure_times < CASH_MAX_POLYREG_TBL_ENTRIES);

	/* All ok! */
	cash_rgbc_clear_iso->num_steps = clear_iso_params.num_steps;
	cash_rgbc_clear_iso->table = clear_iso_params.table;

	/* These configurations are not mandatory */
	tmp = (int32_t)strtol(rgbc_clear_min, NULL, 10);
	if (tmp != 0) {
		cash_config->rgbc_clear_min = tmp;
	}

	tmp = (int32_t)strtol(rgbc_clear_max, NULL, 10);
	if (tmp != 0) {
		cash_config->rgbc_clear_max = tmp;
	}

	tmp = (int32_t)strtol(rgbc_polyreg_degree, NULL, 10);
	if (tmp != 0) {
		cash_config->rgbc_polyreg_degree = tmp;
	}

	tmp = (int32_t)strtol(rgbc_polyreg_extra, NULL, 10);
	if (tmp != 0) {
		cash_config->rgbc_polyreg_extra = tmp;
	}

rgbc_end:
	free(buf);
rgbc_secfail:
	close(fd);
	XML_ParserFree(pa);

	return ret;
}
