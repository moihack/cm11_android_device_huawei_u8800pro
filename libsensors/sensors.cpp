/*
 * Copyright (C) 2012 The Android Open-Source Project
 * Copyright (C) 2014 Rudolf Tammekivi <rtammekivi@gmail.com>
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

#include <hardware/sensors.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdlib.h>

#include <linux/input.h>

#include <cutils/log.h>

#include "sensors.h"

#include "AccelSensor.h"
#include "AkmSensor.h"
#include "GyroSensor.h"
#include "LightSensor.h"
#include "ProximitySensor.h"

/*****************************************************************************/

/*
 * The SENSORS Module
 */

static const struct sensor_t sSensorList[] = {
    {
        "LIS3DH 3-axis Accelerometer",
        "ST Microelectronics",
        1,
        ID_A,
        SENSOR_TYPE_ACCELEROMETER,
        MAX_RANGE_A,
        CONVERT_A,
        0.145f,
        10000,
        0,
        0,
        { 0 },
    },
    {
        "AK8975 3-axis Magnetic field sensor",
        "Asahi Kasei Microdevices",
        1,
        ID_M,
        SENSOR_TYPE_MAGNETIC_FIELD,
        1228.8f,
        CONVERT_M,
        0.35f,
        10000,
        0,
        0,
        { 0 },
    },
    {
        "AKM Orientation sensor",
        "Asahi Kasei Microdevices",
        1,
        ID_O,
        SENSOR_TYPE_ORIENTATION,
        360.0f,
        CONVERT_O,
        1.0f,
        10000,
        0,
        0,
        { 0 },
    },
#if 0
    { "AKM Rotation vector sensor",
        "Asahi Kasei Microdevices",
        1,
        ID_R,
        SENSOR_TYPE_ROTATION_VECTOR,
        34.907f,
        CONVERT_R,
        1.0f,
        10000,
        0,
        0,
        { 0 },
    },
#endif
    {
        "L3G4200D Gyroscope sensor",
        "ST Microelectronics",
        1,
        ID_G,
        SENSOR_TYPE_GYROSCOPE,
        MAX_RANGE_G,
        CONVERT_G,
        6.1f,
        2000,
        0,
        0,
        { 0 },
    },
    {
        "L3G4200D Temperature sensor",
        "ST Microelectronics",
        1,
        ID_T,
        SENSOR_TYPE_AMBIENT_TEMPERATURE,
        85,
        1,
        6.1f,
        2000,
        0,
        0,
        { 0 },
    },
    {
        "APDS-9900 Light sensor",
        "Avago Technologies",
        1,
        ID_L,
        SENSOR_TYPE_LIGHT,
        60000.0f,
        0.0125f,
        0.20f,
        1000,
        0,
        0,
        { 0 },
    },
    {
        "APDS-9900 Proximity sensor",
        "Avago Technologies",
        1,
        ID_P,
        SENSOR_TYPE_PROXIMITY,
        1.0f,
        1.0f,
        3.0f,
        1000,
        0,
        0,
        { 0 },
    },
};

static int open_sensors(const struct hw_module_t* module, const char* name,
        struct hw_device_t** device);

static int sensors__get_sensors_list(struct sensors_module_t* module,
        struct sensor_t const** list)
{
    *list = sSensorList;
    return ARRAY_SIZE(sSensorList);
}

static struct hw_module_methods_t sensors_module_methods = {
    .open = open_sensors
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .module_api_version = SENSORS_MODULE_API_VERSION_0_1,
        .hal_api_version = HARDWARE_HAL_API_VERSION,
        .id = SENSORS_HARDWARE_MODULE_ID,
        .name = "U8860 Sensors Module",
        .author = "The Android Open Source Project",
        .methods = &sensors_module_methods,
        .dso = 0,
        .reserved = { 0 },
    },
    .get_sensors_list = sensors__get_sensors_list,
};

struct sensors_poll_context_t {
    struct sensors_poll_device_t device; // must be first

    sensors_poll_context_t();
    ~sensors_poll_context_t();
    int activate(int handle, int enabled);
    int setDelay(int handle, int64_t ns);
    int pollEvents(sensors_event_t* data, int count);

private:
    enum {
        lis3dh_acc = 0,
        akm,
        l3g4200d_gyro,
        apds9900_light,
        apds9900_proximity,
        numSensorDrivers,
        numFds,
    };

    static const size_t wake = numFds - 1;
    static const char WAKE_MESSAGE = 'W';
    struct pollfd mPollFds[numFds];
    int mWritePipeFd;
    SensorBase* mSensors[numSensorDrivers];

    int handleToDriver(int handle) const {
        switch (handle) {
            case ID_A:
                return lis3dh_acc;
            case ID_M:
            case ID_O:
            case ID_R:
                return akm;
            case ID_G:
            case ID_T:
                return l3g4200d_gyro;
            case ID_L:
                return apds9900_light;
            case ID_P:
                return apds9900_proximity;
        }
        return -EINVAL;
    }
};

/*****************************************************************************/

sensors_poll_context_t::sensors_poll_context_t()
{
    mSensors[lis3dh_acc] = new AccelSensor();
    mPollFds[lis3dh_acc].fd = mSensors[lis3dh_acc]->getFd();
    mPollFds[lis3dh_acc].events = POLLIN;
    mPollFds[lis3dh_acc].revents = 0;

    mSensors[akm] = new AkmSensor();
    mPollFds[akm].fd = mSensors[akm]->getFd();
    mPollFds[akm].events = POLLIN;
    mPollFds[akm].revents = 0;

    mSensors[l3g4200d_gyro] = new GyroSensor();
    mPollFds[l3g4200d_gyro].fd = mSensors[l3g4200d_gyro]->getFd();
    mPollFds[l3g4200d_gyro].events = POLLIN;
    mPollFds[l3g4200d_gyro].revents = 0;

    mSensors[apds9900_light] = new LightSensor();
    mPollFds[apds9900_light].fd = mSensors[apds9900_light]->getFd();
    mPollFds[apds9900_light].events = POLLIN;
    mPollFds[apds9900_light].revents = 0;

    mSensors[apds9900_proximity] = new ProximitySensor();
    mPollFds[apds9900_proximity].fd = mSensors[apds9900_proximity]->getFd();
    mPollFds[apds9900_proximity].events = POLLIN;
    mPollFds[apds9900_proximity].revents = 0;

    int wakeFds[2];
    int result = pipe(wakeFds);
    ALOGE_IF(result<0, "error creating wake pipe (%s)", strerror(errno));
    fcntl(wakeFds[0], F_SETFL, O_NONBLOCK);
    fcntl(wakeFds[1], F_SETFL, O_NONBLOCK);
    mWritePipeFd = wakeFds[1];

    mPollFds[wake].fd = wakeFds[0];
    mPollFds[wake].events = POLLIN;
    mPollFds[wake].revents = 0;
}

sensors_poll_context_t::~sensors_poll_context_t() {
    for (int i=0 ; i<numSensorDrivers ; i++) {
        delete mSensors[i];
    }
    close(mPollFds[wake].fd);
    close(mWritePipeFd);
}

int sensors_poll_context_t::activate(int handle, int enabled) {
    int drv = handleToDriver(handle);
    int err;

    if (drv < 0) {
        return drv;
    }

    err = mSensors[drv]->setEnable(handle, enabled);

    if (err) {
        return err;
    }
    /* Accelerometer for fusion data */
    if ((handle == ID_O) ||
        (handle == ID_R)) {
        err = mSensors[lis3dh_acc]->setEnable(handle, enabled);
    }
    if (enabled && !err) {
        const char wakeMessage(WAKE_MESSAGE);
        int result = write(mWritePipeFd, &wakeMessage, 1);
        ALOGE_IF(result<0, "error sending wake message (%s)", strerror(errno));
    }
    return err;
}

int sensors_poll_context_t::setDelay(int handle, int64_t ns) {
    int drv = handleToDriver(handle);
    int err;

    if (drv < 0) {
        return drv;
    }

    err = mSensors[drv]->setDelay(handle, ns);

    if (err) {
        return err;
    }
    /* Accelerometer for fusion data */
    if ((handle == ID_O) ||
        (handle == ID_R)) {
        err = mSensors[lis3dh_acc]->setDelay(handle, ns);
    }
    return err;
}

int sensors_poll_context_t::pollEvents(sensors_event_t* data, int count)
{
    int nbEvents = 0;
    int n = 0;

    do {
        // see if we have some leftover from the last poll()
        for (int i=0 ; count && i<numSensorDrivers ; i++) {
            SensorBase* const sensor(mSensors[i]);
            if ((mPollFds[i].revents & POLLIN) || (sensor->hasPendingEvents())) {
                int nb = sensor->readEvents(data, count);
                if (nb < count) {
                    // no more data for this sensor
                    mPollFds[i].revents = 0;
                }
                if ((0 != nb) && (lis3dh_acc == i)) {
                    static_cast<AkmSensor*>(mSensors[akm])->setAccel(&data[nb-1]);
                }
                count -= nb;
                nbEvents += nb;
                data += nb;
            }
        }

        if (count) {
            // we still have some room, so try to see if we can get
            // some events immediately or just wait if we don't have
            // anything to return
            n = poll(mPollFds, numFds, nbEvents ? 0 : -1);
            if (n<0) {
                ALOGE("poll() failed (%s)", strerror(errno));
                return -errno;
            }
            if (mPollFds[wake].revents & POLLIN) {
                char msg;
                int result = read(mPollFds[wake].fd, &msg, 1);
                ALOGE_IF(result<0, "error reading from wake pipe (%s)", strerror(errno));
                ALOGE_IF(msg != WAKE_MESSAGE, "unknown message on wake queue (0x%02x)", int(msg));
                mPollFds[wake].revents = 0;
            }
        }
        // if we have events and space, go read them
    } while (n && count);

    return nbEvents;
}

/*****************************************************************************/

static int poll__close(struct hw_device_t *dev)
{
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    if (ctx) {
        delete ctx;
    }
    return 0;
}

static int poll__activate(struct sensors_poll_device_t *dev,
        int handle, int enabled) {
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->activate(handle, enabled);
}

static int poll__setDelay(struct sensors_poll_device_t *dev,
        int handle, int64_t ns) {
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->setDelay(handle, ns);
}

static int poll__poll(struct sensors_poll_device_t *dev,
        sensors_event_t* data, int count) {
    sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
    return ctx->pollEvents(data, count);
}

/*****************************************************************************/

/** Open a new instance of a sensor device using name */
static int open_sensors(const struct hw_module_t* module, const char* id,
        struct hw_device_t** device)
{
    int status = -EINVAL;
    sensors_poll_context_t *dev = new sensors_poll_context_t();

    memset(&dev->device, 0, sizeof(sensors_poll_device_t));

    dev->device.common.tag = HARDWARE_DEVICE_TAG;
    dev->device.common.version  = SENSORS_DEVICE_API_VERSION_0_1;
    dev->device.common.module   = const_cast<hw_module_t*>(module);
    dev->device.common.close    = poll__close;
    dev->device.activate        = poll__activate;
    dev->device.setDelay        = poll__setDelay;
    dev->device.poll            = poll__poll;

    *device = &dev->device.common;
    status = 0;

    return status;
}

