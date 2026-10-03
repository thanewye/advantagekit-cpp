// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#include "subsystems/drive/PhoenixOdometryThread.h"

#include <chrono>

#include <wpi/system/RobotController.hpp>
#include <wpi/units/time.hpp>

#include "generated/TunerConstants.h"
#include "subsystems/drive/Drive.h"

PhoenixOdometryThread& PhoenixOdometryThread::GetInstance() {
    static PhoenixOdometryThread instance;
    return instance;
}

PhoenixOdometryThread::PhoenixOdometryThread()
    : isCANFD_(TunerConstants::kCANBus.IsNetworkFD()) {}

PhoenixOdometryThread::~PhoenixOdometryThread() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void PhoenixOdometryThread::Start() {
    if (!timestampQueues_.empty() && !running_.exchange(true)) {
        thread_ = std::thread{[this] { Run(); }};
    }
}

std::shared_ptr<OdometryQueue> PhoenixOdometryThread::RegisterPhoenixSignal(std::unique_ptr<ctre::phoenix6::BaseStatusSignal> signal) {
    auto queue = std::make_shared<OdometryQueue>();
    std::scoped_lock lock{signalsLock_, Drive::odometryLock};
    phoenixSignals_.push_back(signal.get());
    ownedPhoenixSignals_.push_back(std::move(signal));
    phoenixQueues_.push_back(queue);
    return queue;
}

std::shared_ptr<OdometryQueue> PhoenixOdometryThread::RegisterSignal(std::function<double()> signal) {
    auto queue = std::make_shared<OdometryQueue>();
    std::scoped_lock lock{signalsLock_, Drive::odometryLock};
    genericSignals_.push_back(std::move(signal));
    genericQueues_.push_back(queue);
    return queue;
}

std::shared_ptr<OdometryQueue> PhoenixOdometryThread::MakeTimestampQueue() {
    auto queue = std::make_shared<OdometryQueue>();
    std::lock_guard lock{Drive::odometryLock};
    timestampQueues_.push_back(queue);
    return queue;
}

void PhoenixOdometryThread::Run() {
    while (running_) {
        // Wait for updates from all signals
        {
            std::lock_guard lock{signalsLock_};
            if (isCANFD_ && !phoenixSignals_.empty()) {
                ctre::phoenix6::BaseStatusSignal::WaitForAll(wpi::units::second_t{2.0 / Drive::GetOdometryFrequency()}, phoenixSignals_);
            } else {
                // "waitForAll" does not support blocking on multiple signals with a bus
                // that is not CAN FD, regardless of Pro licensing. No reasoning for this
                // behavior is provided by the documentation.
                std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / Drive::GetOdometryFrequency()));
                if (!phoenixSignals_.empty()) ctre::phoenix6::BaseStatusSignal::RefreshAll(phoenixSignals_);
            }
        }

        // Save new data to queues
        std::lock_guard lock{Drive::odometryLock};

        // Sample timestamp is current monotonic time minus average CAN latency
        // Default timestamps from Phoenix are NOT compatible with
        // FPGA timestamps, this solution is imperfect but close
        double timestamp = wpi::RobotController::GetMonotonicTime() / 1e9;
        double totalLatency = 0.0;
        for (const auto* signal : phoenixSignals_) {
            totalLatency += signal->GetTimestamp().GetLatency().value();
        }
        if (!phoenixSignals_.empty()) {
            timestamp -= totalLatency / phoenixSignals_.size();
        }

        // Add new samples to queues
        for (size_t i = 0; i < phoenixSignals_.size(); i++) {
            phoenixQueues_[i]->Offer(phoenixSignals_[i]->GetValueAsDouble());
        }
        for (size_t i = 0; i < genericSignals_.size(); i++) {
            genericQueues_[i]->Offer(genericSignals_[i]());
        }
        for (size_t i = 0; i < timestampQueues_.size(); i++) {
            timestampQueues_[i]->Offer(timestamp);
        }
    }
}
