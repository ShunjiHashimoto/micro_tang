#ifndef LOG_HPP
#define LOG_HPP
#include "main.h"
#include <vector>
#include "params.hpp"
#include "encoder.h"

class Log {
public:
    void saveLog(const std::vector<float>& new_vals);
    void printLog() const;

private:
    std::vector<std::vector<float>> logMatrix_;
};

#endif // LOG_HPP
