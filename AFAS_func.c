/* Copyright 2018 The MathWorks, Inc. */
#include "AFAS_func.h"
#include <math.h>
#include "tmwtypes.h"
#define EARTH_RADIUS 6371000 // Radius of the earth in meters
double add(double u1, double u2)
{
    return u1 + u2;
}

double timesK(double u, double K)
{
    return u * K;
}

void incrementElement(int* u, unsigned int size, unsigned int increment_idx)
{
    if (increment_idx < size) {
        u[increment_idx]++;
    }
}

void generate4DT(int* u,unsigned int size, double *s)
{

}

double latToMeters(double lat1, double lat2) {
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    return dLat * EARTH_RADIUS;
}

double lonToMeters(double lon1, double lon2, double lat) {
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    return dLon * EARTH_RADIUS * cos(lat * M_PI / 180.0);
}

int GetCurrentWaypoint(double x, double y, int* u)
{
    int i = 0;
    return i;
}
TrafficLightColor getNextTrafficLight(TrafficLightColor current)
{
    switch(current) {
        case RED:
            return GREEN;
        case YELLOW:
            return RED;
        case GREEN:
            return YELLOW;
        default:
            return UNKNOWN;
    }
}

void counterbusFcn(const COUNTERBUS *u1, int32_T u2, COUNTERBUS *y1, int32_T *y2)
{
    int32_T limit;
    boolean_T inputGElower;  
    limit = u1->inputsignal.input + u2;
    inputGElower = (limit >= u1->limits.lower_saturation_limit);
    if((u1->limits.upper_saturation_limit >= limit) && inputGElower) {
        *y2 = limit;
    } else {
        if(inputGElower) {
            limit = u1->limits.upper_saturation_limit;
        } else {
          limit = u1->limits.lower_saturation_limit;
        }
        *y2 = limit;
    }
    y1->inputsignal.input = *y2;
    y1->limits = u1->limits;
}