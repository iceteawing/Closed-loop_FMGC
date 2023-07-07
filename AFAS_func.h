/* Copyright 2018 The MathWorks, Inc. */
#ifndef _AFAS_FUNC_H_
#define _AFAS_FUNC_H_

/* define struct and enumeration */
typedef enum
{
    RED,
    YELLOW,
    GREEN,
    UNKNOWN
} TrafficLightColor;

typedef struct {
    int input;
} SIGNALBUS;

typedef struct {
    int upper_saturation_limit;
    int lower_saturation_limit;
} LIMITBUS;

typedef struct {
    SIGNALBUS inputsignal;
    LIMITBUS limits;
} COUNTERBUS;


/* custom c functions */
extern double add(double u1, double u2);
extern double timesK(double u, double K);
extern void incrementElement(int* u, unsigned int size, unsigned int increment_idx);
extern TrafficLightColor getNextTrafficLight(TrafficLightColor current);
extern void counterbusFcn(const COUNTERBUS *u1, int u2, COUNTERBUS *y1, int *y2);
extern void generate4DT(int* u, unsigned int size, double* s);
extern int GetCurrentWaypoint(double x, double y, int* u);


extern double latToMeters(double lat1, double lat2);

extern double lonToMeters(double lon1, double lon2, double lat);
#endif