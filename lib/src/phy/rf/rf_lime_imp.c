/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2015 Software Radio Systems Limited
 *
 * \section LICENSE
 *
 * This file is part of the srsLTE library.
 *
 * srsLTE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsLTE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */


#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "srslte/srslte.h"
#include "rf_lime_imp.h"
#include "srslte/phy/rf/rf.h"

#include "lime/LimeSuite.h"

typedef struct {
    lms_device_t *device;
    lms_stream_t rx_stream;
    lms_stream_t tx_stream;
    double tx_gain;
    double sampleRate;
    uint64_t timestamp;
} rf_lime_handle_t;

int rf_lime_error() {
    printf("%s\n", LMS_GetLastErrorMessage());
    return SRSLTE_ERROR;
}

void rf_lime_suppress_handler(const char *x)
{
    // not supported
}

void rf_lime_msg_handler(const char *msg)
{
    // not supported
}

void rf_lime_suppress_stdout(void *h)
{
    // not supported
}

void rf_lime_register_error_handler(void *notused, srslte_rf_error_handler_t new_handler)
{
    // not supported
}

char* rf_lime_devname(void* h) {
    return "lime";
}

bool rf_lime_rx_wait_lo_locked(void *h) {
    return true;
}

void rf_lime_set_tx_cal(void *h, srslte_rf_cal_t *cal) {
    printf("Set TX calibration not supported\n");
}

void rf_lime_set_rx_cal(void *h, srslte_rf_cal_t *cal) {
    printf("Set RX calibration not supported\n");
}

int rf_lime_start_rx_stream(void *h) {
    rf_lime_handle_t *handle = (rf_lime_handle_t*) h;
    double bw = handle->sampleRate;
    if (LMS_SetLPFBW(handle->device, LMS_CH_RX, 0, bw) != 0)
        return rf_lime_error();
    if (LMS_Calibrate(handle->device, LMS_CH_RX, 0, bw, 0) != 0)
        return rf_lime_error();
    if (LMS_SetLPFBW(handle->device, LMS_CH_TX, 0, bw) != 0)
        return rf_lime_error();
    LMS_SetGaindB(handle->device, LMS_CH_TX, 0, handle->tx_gain);
    if (LMS_Calibrate(handle->device, LMS_CH_TX, 0, bw, 0) != 0)
        return rf_lime_error();


    handle->rx_stream.channel = 0;
    handle->rx_stream.fifoSize = 1024 * 1024;
    handle->rx_stream.throughputVsLatency = 0.3;
    handle->rx_stream.dataFmt = LMS_FMT_F32;
    handle->rx_stream.isTx = false;

    handle->tx_stream.channel = 0;
    handle->tx_stream.fifoSize = 1024 * 1024;
    handle->tx_stream.throughputVsLatency = 0.3;
    handle->tx_stream.dataFmt = LMS_FMT_F32;
    handle->tx_stream.isTx = true;

    if (LMS_SetupStream(handle->device, &handle->rx_stream) != 0)
        return rf_lime_error();
    if (LMS_StartStream(&handle->rx_stream) != 0)
        return rf_lime_error();

    if (LMS_SetupStream(handle->device, &handle->tx_stream) != 0)
        return rf_lime_error();
    if (LMS_StartStream(&handle->tx_stream) != 0)
        return rf_lime_error();

    return SRSLTE_SUCCESS;
}

int rf_lime_stop_rx_stream(void *h) {
    rf_lime_handle_t *handle = (rf_lime_handle_t*) h;

    if (LMS_StopStream(&handle->rx_stream) != 0)
        return rf_lime_error();
    if (LMS_DestroyStream(handle->device, &handle->rx_stream) != 0)
        return rf_lime_error();

    if (LMS_StopStream(&handle->tx_stream) != 0)
        return rf_lime_error();
    if (LMS_DestroyStream(handle->device, &handle->tx_stream) != 0)
        return rf_lime_error();

    return SRSLTE_SUCCESS;
}

void rf_lime_flush_buffer(void *h) {
}

bool rf_lime_has_rssi(void *h) {
    return false;
}

float rf_lime_get_rssi(void *h) {
    return 0.0f;
}

//TODO: add multi-channel support
int rf_lime_open_multi(char *args, void **h, uint32_t nof_rx_antennas) {

    return rf_lime_open(args, h);
}

int rf_lime_open(char *args, void **h) {

    lms_info_str_t list[8];
    // create handler
    rf_lime_handle_t *handle = (rf_lime_handle_t*) malloc(sizeof (rf_lime_handle_t));
    memset(handle, 0, sizeof (rf_lime_handle_t));

    // Open LMS7002 port
    int n = LMS_GetDeviceList(list);
    if (n < 1) {
        printf("No LMS7002 board found: %d\n", n);
        return SRSLTE_ERROR;
    }

    //Select first device
    //TODO: add ability to select other via args
    if (LMS_Open(&(handle->device), list[0], 0) != 0)
        return rf_lime_error();

    if (LMS_Init(handle->device) != 0)
        return rf_lime_error();
    //TODO: make antenna port selectable by args
    LMS_SetAntenna(handle->device, LMS_CH_TX, 0, 2); //TX1_2 port (none, band1, band2)
    LMS_SetAntenna(handle->device, LMS_CH_RX, 0, 1); //RX1_H port (none, H, L, W)

    *h = handle;
    return SRSLTE_SUCCESS;
}

int rf_lime_close(void *h) {
    rf_lime_handle_t *handle = (rf_lime_handle_t*) h;
    if (LMS_Close(handle->device) != 0)
        return rf_lime_error();
    free(handle);
    return SRSLTE_SUCCESS;
}

void rf_lime_set_master_clock_rate(void *h, double rate) {

}

bool rf_lime_is_master_clock_dynamic(void *h) {
    return true;
}

double rf_lime_set_rx_srate(void *h, double rate) {
    rf_lime_handle_t *handle = (rf_lime_handle_t*) h;
    if (LMS_SetSampleRateDir(handle->device, LMS_CH_RX, rate, 4) != 0)
        return rf_lime_error();
    handle->sampleRate = rate;
    LMS_GetSampleRate(handle->device, LMS_CH_RX, 0, &rate, NULL);
    return rate;
}

double rf_lime_set_tx_srate(void *h, double rate) {
    rf_lime_handle_t *handle = (rf_lime_handle_t*) h;
    if (LMS_SetSampleRateDir(handle->device, LMS_CH_TX, rate, 16) != 0)
        return rf_lime_error();
    LMS_GetSampleRate(handle->device, LMS_CH_TX, 0, &rate, NULL);
    return rate;
}

double rf_lime_set_rx_gain(void *h, double gain) {
    rf_lime_handle_t *handle = (rf_lime_handle_t*) h;
    LMS_SetGaindB(handle->device, LMS_CH_RX, 0, gain);
    return rf_lime_get_rx_gain(h);
}

double rf_lime_set_tx_gain(void *h, double gain) {
    rf_lime_handle_t *handle = (rf_lime_handle_t*) h;
    LMS_SetGaindB(handle->device, LMS_CH_TX, 0, gain);
    handle->tx_gain = gain;
    return rf_lime_get_tx_gain(h);
}

double rf_lime_get_rx_gain(void *h) {
    rf_lime_handle_t *handle = (rf_lime_handle_t*) h;
    unsigned gain;
    LMS_GetGaindB(handle->device, LMS_CH_RX, 0, &gain);
    return gain;
}

double rf_lime_get_tx_gain(void *h) {
    rf_lime_handle_t *handle = (rf_lime_handle_t*) h;
    unsigned gain;
    LMS_GetGaindB(handle->device, LMS_CH_TX, 0, &gain);
    return gain;
}

double rf_lime_set_rx_freq(void *h, double freq) {
    rf_lime_handle_t *handle = (rf_lime_handle_t*) h;
    if (LMS_SetLOFrequency(handle->device, LMS_CH_RX, 0, freq) != 0)
        return rf_lime_error();
    LMS_GetLOFrequency(handle->device, LMS_CH_RX, 0, &freq);
    return freq;
}

double rf_lime_set_tx_freq(void *h, double freq) {
    rf_lime_handle_t *handle = (rf_lime_handle_t*) h;
    if (LMS_SetLOFrequency(handle->device, LMS_CH_TX, 0, freq) != 0)
        return rf_lime_error();
    LMS_GetLOFrequency(handle->device, LMS_CH_TX, 0, &freq);
    return freq;
}

void rf_lime_get_time(void *h, time_t *secs, double *frac_secs) {

    rf_lime_handle_t *handle = (rf_lime_handle_t*) h;
    double totalsecs = (double) handle->timestamp / handle->sampleRate;
    *secs = (time_t) totalsecs;
    *frac_secs = totalsecs - (*secs);
}

//TODO: add multi-channel support
int rf_lime_recv_with_time_multi(void *h,
                                   void **data,
                                   uint32_t nsamples,
                                   bool blocking,
                                   time_t *secs,
                                   double *frac_secs) {
    printf("RCV multi not supported");
    return 0;
}

int rf_lime_recv_with_time(void *h,
                            void *data,
                            uint32_t nsamples,
                            bool blocking,
                            time_t *secs,
                            double *frac_secs) {
    rf_lime_handle_t *handle = (rf_lime_handle_t*) h;

    lms_stream_meta_t meta;
    meta.waitForTimestamp = false;
    meta.flushPartialPacket = false;

    int ret = LMS_RecvStream(&handle->rx_stream, data, nsamples, &meta, 30);
    if (ret != nsamples)
        return SRSLTE_ERROR;

    double totalsecs = (double) meta.timestamp / handle->sampleRate;
    *secs = (time_t) totalsecs;
    *frac_secs = totalsecs - (*secs);
    handle->timestamp = meta.timestamp + ret;

    return ret;
}

int rf_lime_send_timed(void *h,
                        void *data,
                        int nsamples,
                        time_t secs,
                        double frac_secs,
                        bool has_time_spec,
                        bool blocking,
                        bool is_start_of_burst,
                        bool is_end_of_burst) {
    rf_lime_handle_t *handle = (rf_lime_handle_t*) h;

    lms_stream_meta_t meta;
    meta.waitForTimestamp = true; // has_time_spec ?
    meta.flushPartialPacket = false;
    meta.timestamp = handle->sampleRate * ((double) secs + frac_secs) + 0.5;
    meta.timestamp -= 9 * handle->sampleRate / 1e6;

    int ret = LMS_SendStream(&handle->tx_stream, data, nsamples, &meta, 30);
    if (ret != nsamples)
        return SRSLTE_ERROR;
    return ret;
}


