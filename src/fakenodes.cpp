#ifndef FAKENODES
#define FAKENODES

#include <random>

int some_data() {
    return rand();
}

float throttle() {
    return rand() / (float)RAND_MAX;
}

float adps_implausibility() {
    return rand() / (float)RAND_MAX;
}

#endif
