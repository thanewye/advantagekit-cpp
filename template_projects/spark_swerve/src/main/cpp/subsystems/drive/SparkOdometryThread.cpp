// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/SparkOdometryThread.h"

#include <mutex>

#include <frc/RobotController.h>
#include <rev/REVLibError.h>
#include <units/time.h>

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
        notifier_.StartPeriodic(units::second_t{1.0 / DriveConstants::odometryFrequency});
    }
}

std::shared_ptr<OdometryQueue> SparkOdometryThread::RegisterSignal(rev::spark::SparkBase& spark, std::function<double()> signal) {
    auto queue = std::make_shared<OdometryQueue>();
    std::lock_guard lock{Drive::odometryLock};
    sparks_.push_back(&spark);
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
    double timestamp = frc::RobotController::GetFPGATime() / 1e6;

    // Read Spark values, mark invalid in case of error
    std::vector<double> sparkValues(sparkSignals_.size());
    bool isValid = true;
    for (size_t i = 0; i < sparkSignals_.size(); i++) {
        sparkValues[i] = sparkSignals_[i]();
        if (sparks_[i]->GetLastError() != rev::REVLibError::kOk) {
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
