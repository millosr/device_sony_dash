/*
 * Copyright (C) 2012 Sony Mobile Communications AB.
 * Copyright (C) 2016 nAOSP ROM
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

#define LOG_TAG "DASH - bma250_motion"

#include <string.h>
#include <stdlib.h>
#include "sensors_log.h"
#include <unistd.h>
#include <cutils/properties.h>
#include <fcntl.h>
#include <linux/input.h>
#include <pthread.h>
#include <errno.h>
#include "sensors_list.h"
#include "sensors_fifo.h"
#include "sensors_select.h"
#include "sensor_util.h"
#include "sensors_id.h"
#include "sensors_config.h"
#include "sensors_sysfs.h"

#define BMA250_MOTION_NAME "bma250_motion"

#define UNUSED_PARAM(param) ((void)(param))

enum {
	BMA250_MOTION_TAP,
	BMA250_MOTION_SLOPE,
	BMA250_MOTION_LOW_G,
	BMA250_MOTION_HIGH_G,
	BMA250_MOTION_COUNT,
};

struct sensor_desc {
	struct sensor_t sensor;
	struct sensor_api_t api;
	
	void *handle;
	int motion;
	int init;
	int active;
};

struct bma250_motion_sensor_composition {
	struct sensor_desc pickup;
	struct sensor_desc significant;
	
	struct sensors_select_t select_worker;
	struct sensors_sysfs_t sysfs;
	
	pthread_mutex_t lock;
};

static int bma250_motion_init(struct sensor_api_t *s);
static void bma250_motion_internal_activate(struct sensor_desc *d, int enable);
static int bma250_motion_activate(struct sensor_api_t *s, int enable);
static int bma250_motion_set_delay(struct sensor_api_t *s, int64_t ns);
static void bma250_motion_close(struct sensor_api_t *s);
static void *bma250_motion_read(void *arg);
static void bma250na_motion_register(struct bma250_motion_sensor_composition *sc);

static int bma250_motion_init(struct sensor_api_t *s)
{
	int fd;
	struct sensor_desc *d = container_of(s, struct sensor_desc, api);
	struct bma250_motion_sensor_composition *sc = d->handle;
	int init = sc->pickup.init || sc->significant.init;
	
	d->init = 1;

	/* check for availability */
	fd = open_input_dev_by_name(BMA250_MOTION_NAME, O_RDONLY | O_NONBLOCK);
	if (fd < 0) {
		ALOGE("%s: failed to open input dev %s, error: %s\n",
			__func__, BMA250_MOTION_NAME, strerror(errno));
		return -1;
	}
	close(fd);
	
	if (init)
		return 0;

	sensors_sysfs_init(&sc->sysfs, BMA250_MOTION_NAME, SYSFS_TYPE_INPUT_DEV);
	sensors_select_init(&sc->select_worker, bma250_motion_read, sc, -1);

	return 0;
}

static void bma250_motion_internal_activate(struct sensor_desc *d, int enable)
{
	struct bma250_motion_sensor_composition *sc = d->handle;
	
	d->motion = 0;
	d->active = enable;
	
	/* sysfs */
	switch (d->sensor.type) {
	case SENSOR_TYPE_PICK_UP_GESTURE:
		/*
		 * MODE: 0010 (status on Y axe)
		 * EVENT: 11 (report with ABS_MISC)
		 * FLAT: 1 (need to be flat before reporting high_g on Y)
		 * GAP: 0
		 * high_g_mode : 0111 0010 (114) (0x72)
		 */
		sc->sysfs.write_int(&sc->sysfs, "high_g_mode", enable ? 0x72 : 0x00);
		break;
	case SENSOR_TYPE_SIGNIFICANT_MOTION:
		/*
		 * MODE: 0111 (status on X/Y/Z axes)
		 * EVENT: 11 (report with ABS_MISC)
		 * FLAT: 0
		 * GAP: 0
		 * slope_mode : 0011 0111 (55) (0x37)
		 */
		sc->sysfs.write_int(&sc->sysfs, "slope_mode", enable ? 0x37 : 0x00);
		break;
	}
}

static int bma250_motion_activate(struct sensor_api_t *s, int enable)
{
	struct sensor_desc *d = container_of(s, struct sensor_desc, api);
	struct bma250_motion_sensor_composition *sc = d->handle;
	int fd = sc->select_worker.get_fd(&sc->select_worker);
	
	//sysfs control
	bma250_motion_internal_activate(d, enable);
	
	/* suspend/resume will be handled in kernel-space */
	if (enable && (fd < 0)) {
		fd = open_input_dev_by_name(BMA250_MOTION_NAME,
			O_RDONLY | O_NONBLOCK);
		if (fd < 0) {
			ALOGE("%s: failed to open input dev %s, error: %s\n",
				__func__, BMA250_MOTION_NAME, strerror(errno));
			return -1;
		}
		sc->select_worker.set_fd(&sc->select_worker, fd);
		sc->select_worker.resume(&sc->select_worker);
	} else if (!enable && (fd > 0)) {
		if (!sc->pickup.active &&
		    !sc->significant.active) {
			sc->select_worker.suspend(&sc->select_worker);
			sc->select_worker.set_fd(&sc->select_worker, -1);
		}
	}
	
	return 0;
}

static int bma250_motion_set_delay(struct sensor_api_t *s, int64_t ns)
{
	UNUSED_PARAM(s);
	UNUSED_PARAM(ns);
	return 0;
}

static void bma250_motion_close(struct sensor_api_t *s)
{
	struct sensor_desc *d = container_of(s, struct sensor_desc, api);
	struct bma250_motion_sensor_composition *sc = d->handle;
	
	d->init = 0;
	if (!(sc->pickup.init || sc->significant.init))
		sc->select_worker.destroy(&sc->select_worker);
}

static void *bma250_motion_read(void *arg)
{
	struct bma250_motion_sensor_composition *sc = arg;
	struct input_event event;
	int fd = sc->select_worker.get_fd(&sc->select_worker);
	
	sensors_event_t sd;
	memset(&sd, 0, sizeof(sd));

	pthread_mutex_lock(&sc->lock);
	while (read(fd, &event, sizeof(event)) > 0) {
		switch (event.type) {
		case EV_ABS:
			if (event.value == BMA250_MOTION_HIGH_G){
				/* pickup the device */
				sc->pickup.motion = 1;
			} else if (event.value == BMA250_MOTION_SLOPE) {
				/* significant_motion */
				sc->significant.motion = 1;
			}
			break;

		case EV_SYN:
			if (sc->pickup.active && sc->pickup.motion) {
				//one-shot so desactivates itself
				bma250_motion_internal_activate(&sc->pickup, 0);
				
				//report event
				sd.sensor = sc->pickup.sensor.handle;
				sd.type = sc->pickup.sensor.type;
				sd.version = sc->pickup.sensor.version;
				sd.timestamp = get_current_nano_time();
				sd.data[0] = 1.0;

				sensors_fifo_put(&sd);
			}
			
			if (sc->significant.active && sc->significant.motion) {
				//one-shot so desactivates itself
				bma250_motion_internal_activate(&sc->significant, 0);
				
				//report event
				sd.sensor = sc->significant.sensor.handle;
				sd.type = sc->significant.sensor.type;
				sd.version = sc->significant.sensor.version;
				sd.timestamp = get_current_nano_time();
				sd.data[0] = 1.0;

				sensors_fifo_put(&sd);
			}
			
			break;
		}
	}
	pthread_mutex_unlock(&sc->lock);

	return NULL;
}

static void bma250na_motion_register(struct bma250_motion_sensor_composition *sc) {
	char value[PROPERTY_VALUE_MAX];

	pthread_mutex_init(&sc->lock, NULL);
	sc->pickup.handle = sc->significant.handle = sc;

	property_get("persist.sensors.pickup", value, "0");
	if (atoi(value))
		(void)sensors_list_register(&sc->pickup.sensor, &sc->pickup.api);

	property_get("persist.sensors.significant", value, "0");
	if (atoi(value))
		(void)sensors_list_register(&sc->significant.sensor, &sc->significant.api);
}

static struct bma250_motion_sensor_composition bma250_motion = {
	.pickup = {
		.sensor = {
			.name = "BMA250 accelerometer Pickup",
			.vendor = "Bosch Sensortec GmbH",
			.version = sizeof(sensors_event_t),
			.handle = SENSOR_PICK_UP_GESTURE_HANDLE,
			.type = SENSOR_TYPE_PICK_UP_GESTURE,
			.power = 0.15,
			.flags = SENSOR_FLAG_WAKE_UP,
			.stringType = SENSOR_STRING_TYPE_PICK_UP_GESTURE,
			.requiredPermission = 0,
			.flags = SENSOR_FLAG_CONTINUOUS_MODE,
		},
		.api = {
			.init = bma250_motion_init,
			.activate = bma250_motion_activate,
			.set_delay = bma250_motion_set_delay,
			.close = bma250_motion_close
		},
	},
	.significant = {
		.sensor = {
			.name = "BMA250 accelerometer Significant",
			.vendor = "Bosch Sensortec GmbH",
			.version = sizeof(sensors_event_t),
			.handle = SENSOR_SIGNIFICANT_MOTION_HANDLE,
			.type = SENSOR_TYPE_SIGNIFICANT_MOTION,
			.power = 0.15,
			.flags = SENSOR_FLAG_WAKE_UP,
			.stringType = SENSOR_STRING_TYPE_SIGNIFICANT_MOTION,
			.requiredPermission = 0,
			.flags = SENSOR_FLAG_CONTINUOUS_MODE,
		},
		.api = {
			.init = bma250_motion_init,
			.activate = bma250_motion_activate,
			.set_delay = bma250_motion_set_delay,
			.close = bma250_motion_close
		},
	},
};

list_constructor(bma250na_motion_init_driver);
void bma250na_motion_init_driver()
{
	bma250na_motion_register(&bma250_motion);
}
