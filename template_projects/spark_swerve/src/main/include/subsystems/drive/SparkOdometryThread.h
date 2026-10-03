// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

#pragma once

#include <functional>
#include <memory>
#include <vector>

#include <rev/SparkBase.h>
#include <wpi/system/Notifier.hpp>

#include "subsystems/drive/OdometryQueue.h"

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version includes an overload for Spark signals, which checks for errors to ensure that
 * all measurements in the sample are valid.
 */
class SparkOdometryThread {
public:
    static SparkOdometryThread& GetInstance();

    void Start();

    /** Registers a Spark signal to be read from the thread. */
    std::shared_ptr<OdometryQueue> RegisterSignal(std::function<rev::util::Signal<double>()> signal);

    /** Registers a generic signal to be read from the thread. */
    std::shared_ptr<OdometryQueue> RegisterSignal(std::function<double()> signal);

    /** Returns a new queue that returns timestamp values for each sample. */
    std::shared_ptr<OdometryQueue> MakeTimestampQueue();

private:
    SparkOdometryThread();

    void Run();

    std::vector<std::function<rev::util::Signal<double>()>> sparkSignals_;
    std::vector<std::function<double()>> genericSignals_;
    std::vector<std::shared_ptr<OdometryQueue>> sparkQueues_;
    std::vector<std::shared_ptr<OdometryQueue>> genericQueues_;
    std::vector<std::shared_ptr<OdometryQueue>> timestampQueues_;

    wpi::Notifier notifier_{[this] { Run(); }};
};
