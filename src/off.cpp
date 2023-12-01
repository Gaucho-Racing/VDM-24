// off.cpp
#include <Arduino.h>
#include <imxrt.h>
#include "main.h"
#include "fakenodes.h"

State off(iCANflex &Car) {
    return OFF;
}

