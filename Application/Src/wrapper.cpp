/*
 * wrapper.cpp
 *
 *  Created on: 2023/06/24
 *      Author: hashimoto
 */
#include "../../Application/Inc/wrapper.hpp"
#include "../../Application/Inc/led.hpp"

extern "C" {
void cpploop(void) {
    LedBlink instance;

    instance.toggle();
}
}




