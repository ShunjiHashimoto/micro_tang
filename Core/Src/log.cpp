#include "log.hpp"

void Log::saveLog(const std::vector<float>& new_vals) {
    logMatrix_.push_back(new_vals);
    return;
}

void Log::printLog() const {
    for (const auto& row : logMatrix_) {
            for(int i=0; i < (int)row.size(); ++i) {
                printf("%lf", row[i]);
                if(i < (int)row.size() - 1) {
                    printf(", ");
                }
        }
        printf("\n\r");
    }
    return;
}
