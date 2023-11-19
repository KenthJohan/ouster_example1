/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 */

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include <random>

#include "ouster_client/client.h"
#include "ouster_client/impl/build.h"
#include "ouster_client/lidar_scan.h"
#include "ouster_client/types.h"
#include <Eigen/Geometry>

#include "tigr.h"
#include "tigr_mouse.h"

using namespace ouster;

const size_t N_SCANS = 5;
const size_t UDP_BUF_SIZE = 65536;

void FATAL(const char *msg)
{
    std::cerr << msg << std::endl;
    std::exit(EXIT_FAILURE);
}

typedef struct
{
    const std::string sensor_hostname;
    sensor::sensor_info info;
    std::shared_ptr<ouster::sensor::client> handle;
    int w;
    int h;
    tigr_mouse_t mouse;
    pthread_mutex_t lock;
    Tigr *bmp;
} app_t;



void scan_to_bmp(LidarScan& scan, ouster::sensor::ChanField f, Tigr * bmp)
{
    Eigen::Ref<img_t<uint32_t>> img = scan.field(f);
    int w = scan.w;
    int h = scan.h;

    int32_t min = INT32_MAX;
    int32_t max = INT32_MIN;
    for(int x = 0; x < w; ++x)
    {
        for(int y = 0; y < h; ++y)
        {
           int32_t value = img(y, x);
           if(value < min) {min = value;}
           if(value > max) {max = value;}
        }
    }

    for(int x = 0; x < w; ++x)
    {
        for(int y = 0; y < h; ++y)
        {
            int32_t value = img(y, x); // row, column
            value -= min;
            value *= 255;
            value /= (max - min);
            int i = y * bmp->w + x;
            bmp->pix[i].r = value;
            bmp->pix[i].g = value;
            bmp->pix[i].b = value;
            bmp->pix[i].a = 255;
        }
    }
}


void print_range(LidarScan& scan, ouster::sensor::ChanField f, Tigr * bmp, int x, int y)
{
    if((x < 0) || (x >= scan.w)){return;}
    if((y < 0) || (y >= scan.h)){return;}
    Eigen::Ref<img_t<uint32_t>> img = scan.field(f);
    int i = y * bmp->w + x;
    bmp->pix[i].r = 0xFF;
    bmp->pix[i].g = 0;
    bmp->pix[i].b = 0;
    bmp->pix[i].a = 255;
    int32_t value = img(y, x); // row, column
    printf("%i %i : %i\n", x, y, value);
    fflush(stdout);
}


void *render_thread(void *arg)
{
    app_t *app = (app_t *)arg;

    sensor::sensor_info info = app->info;

    size_t w = info.format.columns_per_frame;
    size_t h = info.format.pixels_per_column;

    ouster::sensor::ColumnWindow column_window = info.format.column_window;

    std::cerr << "  Firmware version:  " << info.fw_rev
              << "\n  Serial number:     " << info.sn
              << "\n  Product line:      " << info.prod_line
              << "\n  Scan dimensions:   " << w << " x " << h
              << "\n  Column window:     [" << column_window.first << ", "
              << column_window.second << "]" << std::endl;

    // A ScanBatcher can be used to batch packets into scans
    sensor::packet_format pf = sensor::get_format(info);
    ScanBatcher batch_to_scan(info.format.columns_per_frame, pf);

    // buffer to store raw packet data
    auto packet_buf = std::make_unique<uint8_t[]>(UDP_BUF_SIZE);

    LidarScan scan = LidarScan{w, h, info.format.udp_profile_lidar};
    size_t cloud_size = w * h;
    XYZLut lut = ouster::make_xyz_lut(info);

    while (1)
    {
        // wait until sensor data is available
        sensor::client_state st = sensor::poll_client(*app->handle);

        // check for error status
        if (st & sensor::CLIENT_ERROR)
            FATAL("Sensor client returned error state!");

        // check for lidar data, read a packet and add it to the current batch
        if (st & sensor::LIDAR_DATA)
        {
            if (!sensor::read_lidar_packet(*app->handle, packet_buf.get(), pf))
            {
                FATAL("Failed to read a packet of the expected size!");
            }

            // batcher will return "true" when the current scan is complete
            if (batch_to_scan(packet_buf.get(), scan))
            {
                // retry until we receive a full set of valid measurements
                // (accounting for azimuth_window settings if any)
                if (scan.complete(info.format.column_window))
                {
                    pthread_mutex_lock(&app->lock);
                    scan_to_bmp(scan, ouster::sensor::ChanField::RANGE, app->bmp);
                    if(app->mouse.btn)
                    {
                        print_range(scan, ouster::sensor::ChanField::RANGE, app->bmp, app->mouse.x, app->mouse.y);
                    }
                    pthread_mutex_unlock(&app->lock);
                    LidarScan::Points p = ouster::cartesian(scan, lut);
                }
            }
        }

        // check if IMU data is available (but don't do anything with it)
        if (st & sensor::IMU_DATA)
        {
            sensor::read_imu_packet(*app->handle, packet_buf.get(), pf);
        }
    }

    return NULL;
}

void init(app_t *app)
{

    // Limit ouster_client log statements to "info" and direct the output to log
    // file rather than the console (default).
    sensor::init_logger("info", "ouster.log");

    std::cerr << "Ouster client example " << ouster::SDK_VERSION << std::endl;

    std::cerr << "Connecting to \"" << app->sensor_hostname << "\"...\n";

    app->handle = sensor::init_client(app->sensor_hostname, "");
    if (!app->handle)
        FATAL("Failed to connect");
    std::cerr << "Connection to sensor succeeded" << std::endl;

    /*
     * Configuration and calibration parameters can be queried directly from the
     * sensor. These are required for parsing the packet stream and calculating
     * accurate point clouds.
     */
    std::cerr << "Gathering metadata..." << std::endl;
    auto metadata = sensor::get_metadata(*app->handle);

    // Raw metadata can be parsed into a `sensor_info` struct
    app->info = sensor::parse_metadata(metadata);
    app->w = app->info.format.columns_per_frame;
    app->h = app->info.format.pixels_per_column;
}

int main(int argc, char *argv[])
{
    pthread_t ptid;
    app_t app = {argv[1]};

    init(&app);


    Tigr *screen = tigrWindow(app.w, app.h, "view_tigr", 0);
    app.bmp = tigrBitmap(app.w, app.h);


    pthread_create(&ptid, NULL, &render_thread, &app);


    while (!tigrClosed(screen))
    {
        int c = tigrReadChar(screen);
        switch (c)
        {
        case 'q':
        default:
            break;
        }
        tigr_mouse_get(screen, &app.mouse);

        tigrClear(screen, tigrRGB(0x80, 0x90, 0xa0));
        pthread_mutex_lock(&app.lock);

        tigrBlit(screen, app.bmp, 0, 0, 0, 0, app.w, app.h);

        pthread_mutex_unlock(&app.lock);
        tigrUpdate(screen);
    }
    tigrFree(screen);

    return EXIT_SUCCESS;
}
