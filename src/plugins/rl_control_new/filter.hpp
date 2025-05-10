#pragma once
#include <Eigen/Dense>

template<typename T>
class EMAFilter {
public:
    EMAFilter(float alpha = 0.1f)
        : alpha_(alpha), initialized_(false) {}

    void reset() { initialized_ = false; }

    T filter(const T& input) {
        if (!initialized_) {
            state_ = input;
            initialized_ = true;
        } else {
            state_ = alpha_ * input + (1.0f - alpha_) * state_;
        }
        return state_;
    }

private:
    float alpha_;
    bool initialized_;
    T state_;
};
