// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <cstddef>
#include <deque>
#include <vector>

/**
 * Bounded queue of samples from the odometry thread. Equivalent to the ArrayBlockingQueue used by
 * the Java template, except that every access must happen while holding Drive::odometryLock.
 */
class OdometryQueue {
public:
    /** Adds a sample, dropping it if the queue is full. */
    void Offer(double value) {
        if (values_.size() < kCapacity) values_.push_back(value);
    }

    /** Returns all queued samples and clears the queue. */
    std::vector<double> Drain() {
        std::vector<double> drained(values_.begin(), values_.end());
        values_.clear();
        return drained;
    }

private:
    static constexpr size_t kCapacity = 20;
    std::deque<double> values_;
};
