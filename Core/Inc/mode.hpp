#ifndef MODE_HPP
#define MODE_HPP

#include "params.hpp"
#include "log.hpp"

class ModeManager {
public:
    ModeManager();
    void initializeSwitches();
    void updateCurrentMode();
    Mode::ModeType getCurrentMode() const;

private:
    Mode::ModeType current_mode;
    Mode::ModeType prev_mode;
};

#endif // MODE_HPP
