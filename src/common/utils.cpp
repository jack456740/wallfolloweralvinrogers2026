#include <wall_follower/common/utils.h>
#include <limits>

std::vector<float> crossProduct(const std::vector<float>& v1, const std::vector<float>& v2)
{
    std::vector<float> res;
    /**
     * TODO: (P1.2) Take the cross product between v1 and v2 and store the
     * result in res. v1, v2, and res should be vectors with 3 elements, which
     * correspond to the vector components [x, y, z].
     **/
     res.push_back(v1[1] * v2[2] - v1[2] * v2[1]);
     res.push_back(v1[2] * v2[0] - v1[0] * v2[2]);
     res.push_back(v1[0] * v2[1] - v1[1] * v2[0]);


    return res;
}

int findMinDist(const std::vector<float>& ranges)
{
    int min_idx =-1;
    float min_distance = std::numeric_limits<float>::max();

    /**
     * TODO: (P1.2) Return the index of the shortest ray in the Lidar scan.
     * For example, if the shortest ray is the third one, at index 2, return 2.
     *
     * HINT: The length of each ray is stored in the vector ranges.
     *
     * HINT: Do not take into account any rays which have 0 distance. This means
     * that the ray returned by the sensor is invalid. Invalid rays will have
     * default range 0, which will always be the minimum if you forget to check
     * for validity.
     **/
     for(int i = 0; i < ranges.size(); i++){
        if(ranges[i] != 0 && ranges[i] < min_distance)
        {
            min_distance = ranges[i];
            min_idx = i;
        }
     }

    return min_idx;
}
