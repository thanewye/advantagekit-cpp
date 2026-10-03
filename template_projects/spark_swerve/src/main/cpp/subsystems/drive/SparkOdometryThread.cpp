// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/SparkOdometryThread.h"

#include <mutex>

#include <rev/REVLibError.h>
#include <wpi/system/RobotController.hpp>
#include <wpi/units/time.hpp>

#include "subsystems/drive/Drive.h"
#include "subsystems/drive/DriveConstants.h"

SparkOdometryThread& SparkOdometryThread::GetInstance() {
    static SparkOdometryThread instance;
    return instance;
}

SparkOdometryThread::SparkOdometryThread() {
    notifier_.SetName("OdometryThread");
}

void SparkOdometryThread::Start() {
    if (!timestampQueues_.empty()) {
        notifier_.StartPeriodic(wpi::units::second_t{1.0 / DriveConstants::odometryFrequency});
    }
}

std::shared_ptr<OdometryQueue> SparkOdometryThread::RegisterSignal(std::function<rev::util::Signal<double>()> signal) {
    auto queue = std::make_shared<OdometryQueue>();
    std::lock_guard lock{Drive::odometryLock};
    sparkSignals_.push_back(std::move(signal));
    sparkQueues_.push_back(queue);
    return queue;
}

std::shared_ptr<OdometryQueue> SparkOdometryThread::RegisterSignal(std::function<double()> signal) {
    auto queue = std::make_shared<OdometryQueue>();
    std::lock_guard lock{Drive::odometryLock};
    genericSignals_.push_back(std::move(signal));
    genericQueues_.push_back(queue);
    return queue;
}

std::shared_ptr<OdometryQueue> SparkOdometryThread::MakeTimestampQueue() {
    auto queue = std::make_shared<OdometryQueue>();
    std::lock_guard lock{Drive::odometryLock};
    timestampQueues_.push_back(queue);
    return queue;
}

void SparkOdometryThread::Run() {
    // Save new data to queues
    std::lock_guard lock{Drive::odometryLock};

    // Get sample timestamp
    double timestamp = wpi::RobotController::GetMonotonicTime() / 1e9;

    // Read Spark values, mark invalid in case of error
    std::vector<double> sparkValues(sparkSignals_.size());
    bool isValid = true;
    for (size_t i = 0; i < sparkSignals_.size(); i++) {
        auto signal = sparkSignals_[i]();
        sparkValues[i] = signal.Get();
        if (!signal.IsValid()) {
            isValid = false;
        }
    }

    // If valid, add values to queues
    if (isValid) {
        for (size_t i = 0; i < sparkSignals_.size(); i++) {
            sparkQueues_[i]->Offer(sparkValues[i]);
        }
        for (size_t i = 0; i < genericSignals_.size(); i++) {
            genericQueues_[i]->Offer(genericSignals_[i]());
        }
        for (size_t i = 0; i < timestampQueues_.size(); i++) {
            timestampQueues_[i]->Offer(timestamp);
        }
    }
}
