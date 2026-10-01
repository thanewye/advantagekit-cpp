// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <ctre/phoenix6/StatusSignal.hpp>

#include "subsystems/drive/OdometryQueue.h"

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using
 * a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent sampling.
 * This also allows Phoenix Pro users to benefit from lower latency between devices using CANivore
 * time synchronization.
 */
class PhoenixOdometryThread {
public:
    static PhoenixOdometryThread& GetInstance();

    ~PhoenixOdometryThread();

    void Start();

    /** Registers a Phoenix signal to be read from the thread. */
    template<typename Unit> std::shared_ptr<OdometryQueue> RegisterSignal(ctre::phoenix6::StatusSignal<Unit> signal) {
        return RegisterPhoenixSignal(std::make_unique<ctre::phoenix6::StatusSignal<Unit>>(std::move(signal)));
    }

    /** Registers a generic signal to be read from the thread. */
    std::shared_ptr<OdometryQueue> RegisterSignal(std::function<double()> signal);

    /** Returns a new queue that returns timestamp values for each sample. */
    std::shared_ptr<OdometryQueue> MakeTimestampQueue();

private:
    PhoenixOdometryThread();

    std::shared_ptr<OdometryQueue> RegisterPhoenixSignal(std::unique_ptr<ctre::phoenix6::BaseStatusSignal> signal);

    void Run();

    std::mutex signalsLock_; // Prevents conflicts when registering signals
    std::vector<std::unique_ptr<ctre::phoenix6::BaseStatusSignal>> ownedPhoenixSignals_;
    std::vector<ctre::phoenix6::BaseStatusSignal*> phoenixSignals_;
    std::vector<std::function<double()>> genericSignals_;
    std::vector<std::shared_ptr<OdometryQueue>> phoenixQueues_;
    std::vector<std::shared_ptr<OdometryQueue>> genericQueues_;
    std::vector<std::shared_ptr<OdometryQueue>> timestampQueues_;

    bool isCANFD_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};
