/*
* Jason Hughes
* April 2025
*
* A unit test to get Latitude and Longitude conversion to UTM
*/
#include <iostream>
#include <iomanip>
#include <limits>

#include "localization/util.hpp"

// TODO formalize with gtest

const double lat = 39.941423;
const double lon = -75.199414;

const double gt_easting = 482963.5387620803;
const double gt_northing = 4421274.811364001;

int main()
{

    double easting, northing;
    char zone[4];

    UTM::LLtoUTM(lat, lon, northing, easting, zone);

    std::cout << "Predicted: " << std::fixed << std::setprecision(10) << easting << " " << std::fixed << std::setprecision(10) << northing << " " << zone << std::endl;
    std::cout << "Ground Truth: " << std::fixed << std::setprecision(10) << gt_easting << " " << std::fixed << std::setprecision(10) << gt_northing << " " << zone << std::endl;

    return 0;
}
