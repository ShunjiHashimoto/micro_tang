/*
 * wrapper.cpp
 *
 *  Created on: 2023/06/24
 *      Author: hashimoto
 */
#include "../../Application/Inc/wrapper.hpp"
#include "../../Application/Inc/led.hpp"
#include "../../Application/Inc/drivers/motor.hpp"

extern "C" {
void cpploop(void) {
    Motor instance;
    
    //TODO: 
    instance.toggle();
}
}




