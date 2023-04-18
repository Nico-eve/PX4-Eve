#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <uORB/uORB.h>
#include <uORB/topics/server_mission_request.h>
#include <uORB/topics/server_mission_waypoints_request.h>

extern "C" __EXPORT int smr_test_main(int argc, char *argv[]);

int smr_test_main(int argc, char *argv[])
{
    // PX4_INFO("Hello, I am only a test program able to inject REQUEST MISSION messages.");

    // Declare structure to store data that will be sent
    // struct server_mission_request_s requestMission;
    struct server_mission_waypoints_request_s requestMission;

    // Clear the structure by filling it with 0s in memory
    memset(&requestMission, 0, sizeof(requestMission));

    // Create a uORB topic advertisement
    orb_advert_t server_mission_request_pub = orb_advertise(ORB_ID(server_mission_waypoints_request), &requestMission);

    //Sim:
    ////smr_test 47.3980507 8.5456073 498.106 3
    // smr_test 47.3980507 8.5456073 498.106 2
    //smr_test 47.3978507 8.5456073 498.106 2

    ////////////////// smr_test 47.3977507 8.5456074 500.0 500.0 4

    // smr_test 47.3980507 8.5456073 488.106 500 2




    // smr_test 30.3879458 -97.7285388 260.0 //+5.0
    // smr_test 47.3977507 8.5456073 488.106
    // smr_test 30.3879620 -97.7285061 250.390
    // smr_test 30.3877967 -97.7285687 250.390


    // smr_test 30.3879750 -97.7284885 236.390
    // smr_test 30.3877993 -97.7285680 246.390

    // Far Left:
    // smr_test 30.3881632 -97.7283876 246.365

    // Far Right:
    // smr_test 30.3876779 -97.7283699 246.365

    // Real:236.365
    // smr_test 30.3879657 -97.7284860 250.365

    // smr_test 30.388000 -97.7285471 241.8

    // smr_test 30.3879300 -97.728511 241.8
    // smr_test 30.3879485 -97.7285210 250

    // smr_test 30.3878903 -97.7285367 236

    // smr_test 30.3879175 -97.7285252 234 2
    // smr_test 30.3879074 -97.7285399 240 2
    if (argc == 6)
    {
    float latAsim = strtof(argv[1],nullptr);
    float lonAsim = strtof(argv[2],nullptr);
    float altsim = strtof(argv[3],nullptr);
    float c_altsim = strtof(argv[4],nullptr);
    // float latAsim = atof(argv[1]);
    // float lonAsim = atof(argv[2]);
    // float altsim = atof(argv[3]);
    // float c_altsim = atof(argv[4]);

    // float latAsim = 47.3977507f;
    // float lonAsim = 8.5456073f;
    // float altsim = 488.106f;

    // float latAreal = 30.388035f;
    // float lonAreal=-97.7284685f;
    // float latBreal = 30.3876946f;
    // float lonBreal = -97.7284818f;
    // float altreal = 235.39f;
    for (int i=0; i<2; i++)
        {
        // char myStr[]={"Salut !!"}; memcpy(requestMission.info, myStr, 9);
        requestMission.timestamp = hrt_absolute_time();
        // requestMission.lat = latAreal;
        // requestMission.lon = lonAreal;
        // requestMission.alt = altreal+9.0f;
        // requestMission.lat = latAsim+0.0003f;
        // requestMission.lon = lonAsim;
        // requestMission.alt = altsim+10.0f;
        requestMission.lat = latAsim;
        requestMission.lon = lonAsim;
        requestMission.alt = altsim;
        requestMission.yaw = 1.5f;
        requestMission.cruise_alt = c_altsim;

        requestMission.mission_type = strtof(argv[5],nullptr);

        float delta_f = 0.00001f;
        requestMission.waypoints[0] = latAsim+delta_f;
        requestMission.waypoints[1] = lonAsim+delta_f;
        requestMission.waypoints[2] = altsim;
        requestMission.waypoints[3] = latAsim+delta_f;
        requestMission.waypoints[4] = lonAsim-delta_f;
        requestMission.waypoints[5] = altsim;
        requestMission.waypoints[6] = latAsim-delta_f;
        requestMission.waypoints[7] = lonAsim-delta_f;
        requestMission.waypoints[8] = altsim;
        requestMission.waypoints[9] = latAsim;
        requestMission.waypoints[10] = lonAsim;
        requestMission.waypoints[11] = altsim;
        // requestMission.mission_type = atof(argv[5]);

        // requestMission.lat  = i;
        // requestMission.lon  = 12345678;
        // requestMission.alt  = i+5;
        // requestMission.yaw  = 0.369;

        orb_publish(ORB_ID(server_mission_waypoints_request), server_mission_request_pub, &requestMission);

        //sleep for 2s
        usleep (2000000);
        }
    }
    PX4_INFO("smr_test finished!");
    PX4_INFO("argv length: %d",argc);
    return 0;
}
