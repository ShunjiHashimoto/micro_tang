#include "utils.hpp"

// 度をラジアンに変換する関数の実装
float degToRad(int degree) {
    float result = degree * M_PI / 180.0;
    return result;
}

// ラジアンを度に変換する関数の実装
int radToDeg(double radian) {
    int result = radian * 180 / M_PI;
    return result;
}