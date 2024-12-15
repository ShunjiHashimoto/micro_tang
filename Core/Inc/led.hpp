/*
 * led.hpp
 *
 *  Created on: 2023/06/24
 *      Author: hashimoto
 */

#ifndef INC_LED_HPP_
#define INC_LED_HPP_

class LedBlink {
public:
    void toggle();
    void stop();
};

class LedSensor {
public:
    void blink();
    // void stop();
};


#endif /* INC_LED_HPP_ */
