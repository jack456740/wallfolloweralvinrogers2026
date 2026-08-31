#include <iostream>
#include <cmath>

#include <signal.h>

#include <mbot_bridge/robot.h>
#include <wall_follower/common/utils.h>

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}


int main(int argc, const char *argv[])
{
    signal(SIGINT, ctrlc);
    signal(SIGTERM, ctrlc);

    // Initialize the robot.
    mbot_bridge::MBot robot;
    // Create empty vectors to store the scan data.
    std::vector<float> ranges;
    std::vector<float> thetas;

    /**
     * TODO: Declare any variables you need here.
     */
    
     float desiredDistance  = 0.3048;

    while (true) {
        // This function gets the Lidar scan data.
        robot.readLidarScan(ranges, thetas);

        /**
         * TODO: (P1.2) Write code to follow the nearest wall here.
         *
         * HINT: You should use the functions crossProduct and findMinDist.
         */
         int minIndex = findMinDist(ranges);

        // Get distance and angle of closest ray.
        float minDistance = ranges[minIndex];
        float minTheta = thetas[minIndex];

        // Direction vector pointing toward the closest point.
        std::vector<float> wallNormal = {
            std::cos(minTheta),
            std::sin(minTheta),
            0
        };
        std::vector<float> forward = {
    1,
    0,
    0
};
        std::vector<float> wallDirection =
            crossProduct(wallNormal, forward);

        // Error from desired wall distance.
        float distanceError = minDistance - desiredDistance;

        // Move along the wall while correcting distance.
        float vx = 0.2;
        float vy = wallDirection[1] * 0.2;
        float wz = distanceError;

        robot.drive(vx, vy, wz);




        if (ctrl_c_pressed) break;
    }

    // Stop the robot.
    robot.stop();
    return 0;
}
