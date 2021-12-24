// MathLibrary.cpp : Defines the exported functions for the DLL.
#include "pch.h" // use stdafx.h in Visual Studio 2017 and earlier
#include <utility>
#include <limits.h>
#include "MathLibrary.h"

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

//#include <k4a/k4a.h>
//#include <k4abt.h>
//#include "packages/Microsoft.Azure.Kinect.Sensor.1.3.0/build/native/include/k4a/k4a.h"
//#include "packages/Microsoft.Azure.Kinect.BodyTracking.1.0.0/build/native/include/k4abt.h"

using namespace std;
#define VERIFY(result, error)                                                                            \
    if(result != K4A_RESULT_SUCCEEDED)                                                                   \
    {                                                                                                    \
        printf("%s \n - (File: %s, Function: %s, Line: %d)\n", error, __FILE__, __FUNCTION__, __LINE__); \
        exit(1);                                                                                         \
    }          

// DLL internal state variables:
static unsigned long long previous_;  // Previous value, if any
static unsigned long long current_;   // Current sequence value
static unsigned index_;               // Current seq. position


//KINECT
static k4a_device_t device;
static k4abt_tracker_t tracker = NULL;
static k4abt_body_t* bodyToReturn = NULL;

void KinectInit() 
{
    //k4a_device_t device;
    //k4abt_tracker_t tracker = NULL;
    k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

    //k4a_device_open(0, &device);
    VERIFY(k4a_device_open(0, &device), "Open K4A Device failed!");
    VERIFY(k4a_device_start_cameras(device, &device_config), "Start K4A cameras failed!");

    k4a_calibration_t sensor_calibration;
    VERIFY(k4a_device_get_calibration(device, device_config.depth_mode, K4A_COLOR_RESOLUTION_OFF, &sensor_calibration),
        "Get depth camera calibration failed!");


    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    VERIFY(k4abt_tracker_create(&sensor_calibration, tracker_config, &tracker), "Body tracker initialization failed!");
}

void KinectFinish()
{
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);
    k4a_device_stop_cameras(device);
    k4a_device_close(device);
}


uint32_t GetNumBodies() {
    int frame_count = 0;
    uint32_t numbodiess = 0;
    do
    {
        k4a_capture_t sensor_capture;
        k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
        if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            frame_count++;

            printf("Start processing frame %d\n", frame_count);

            k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);

            k4a_capture_release(sensor_capture);
            if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                // It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Add capture to tracker process queue timeout!\n");
                break;
            }
            else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
            {
                printf("Error! Add capture to tracker process queue failed!\n");
                break;
            }

            k4abt_frame_t body_frame = NULL;
            k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
            if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
            {
                uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
                numbodiess = num_bodies ;
                printf("%u bodies are detected!\n", num_bodies);

                for (uint32_t i = 0; i < num_bodies; i++)
                {
                    k4abt_body_t body;
                    VERIFY(k4abt_frame_get_body_skeleton(body_frame, i, &body.skeleton), "Get body from body frame failed!");
                    body.id = k4abt_frame_get_body_id(body_frame, i);

                    //print_body_information(body);
                }

                k4a_image_t body_index_map = k4abt_frame_get_body_index_map(body_frame);
                if (body_index_map != NULL)
                {
                    //print_body_index_map_middle_line(body_index_map);
                    k4a_image_release(body_index_map);
                }
                else
                {
                    printf("Error: Fail to generate bodyindex map!\n");
                }

                k4abt_frame_release(body_frame);
            }
            else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
            {
                //  It should never hit timeout when K4A_WAIT_INFINITE is set.
                printf("Error! Pop body frame result timeout!\n");
                break;
            }
            else
            {
                printf("Pop body frame result failed!\n");
                break;
            }
        }
        else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
        {
            // It should never hit time out when K4A_WAIT_INFINITE is set.
            printf("Error! Get depth frame time out!\n");
            break;
        }
        else
        {
            printf("Get depth capture returned error: %d\n", get_capture_result);
            break;
        }

    } while (false);
    return numbodiess;
}

k4abt_body_t* CaptureFrame()
{
    bodyToReturn = NULL;
    bool bodyReady = false;
    k4a_capture_t sensor_capture;
    k4a_wait_result_t get_capture_result = k4a_device_get_capture(device, &sensor_capture, K4A_WAIT_INFINITE);
    if (get_capture_result == K4A_WAIT_RESULT_SUCCEEDED)
    {
        //frame_count++;
        //printf("Start processing frame %d\n", frame_count);

        k4a_wait_result_t queue_capture_result = k4abt_tracker_enqueue_capture(tracker, sensor_capture, K4A_WAIT_INFINITE);

        k4a_capture_release(sensor_capture);
        if (queue_capture_result == K4A_WAIT_RESULT_TIMEOUT)
        {
            // It should never hit timeout when K4A_WAIT_INFINITE is set.
            //printf("Error! Add capture to tracker process queue timeout!\n");
            //break;
        }
        else if (queue_capture_result == K4A_WAIT_RESULT_FAILED)
        {
            //printf("Error! Add capture to tracker process queue failed!\n");
            //break;
        }

        k4abt_frame_t body_frame = NULL;
        k4a_wait_result_t pop_frame_result = k4abt_tracker_pop_result(tracker, &body_frame, K4A_WAIT_INFINITE);
        if (pop_frame_result == K4A_WAIT_RESULT_SUCCEEDED)
        {
            uint32_t num_bodies = k4abt_frame_get_num_bodies(body_frame);
            //printf("%u bodies are detected!\n", num_bodies);

            for (uint32_t i = 0; i < num_bodies; i++)
            {
                k4abt_body_t body;
                VERIFY(k4abt_frame_get_body_skeleton(body_frame, i, &body.skeleton), "Get body from body frame failed!");
                body.id = k4abt_frame_get_body_id(body_frame, i);
                if (!bodyReady)
                {
                    bodyReady = true;
                    bodyToReturn = new k4abt_body_t;
                    *bodyToReturn = body;
                }
                //print_body_information(body);
            }

            k4a_image_t body_index_map = k4abt_frame_get_body_index_map(body_frame);
            if (body_index_map != NULL)
            {
                //print_body_index_map_middle_line(body_index_map);
                k4a_image_release(body_index_map);
            }
            else
            {
                //printf("Error: Fail to generate bodyindex map!\n");
            }

            k4abt_frame_release(body_frame);
        }
        else if (pop_frame_result == K4A_WAIT_RESULT_TIMEOUT)
        {
            //  It should never hit timeout when K4A_WAIT_INFINITE is set.
            //printf("Error! Pop body frame result timeout!\n");
            //break;
        }
        else
        {
            //printf("Pop body frame result failed!\n");
            //break;
        }
    }
    //else if (get_capture_result == K4A_WAIT_RESULT_TIMEOUT)
    //{
    //    // It should never hit time out when K4A_WAIT_INFINITE is set.
    //    //printf("Error! Get depth frame time out!\n");
    //    //break;
    //}
    //else
    //{
    //    //printf("Get depth capture returned error: %d\n", get_capture_result);
    //    //break;
    //}
    return bodyToReturn;
}


Teste* Testa() {
    Teste* Thiago = new Teste;
    Thiago->t[0] = 5;
    Thiago->t[1] = 6;
    return Thiago;
}