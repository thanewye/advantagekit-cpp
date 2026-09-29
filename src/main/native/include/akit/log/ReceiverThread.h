#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include "akit/log/LogDataReceiver.h"
#include "akit/log/LogStorage.h"
#include "akit/log/LogTable.h"

namespace akit {
    /** Background dispatch thread for LogDataReceivers, ported from Java's ReceiverThread. */
    class ReceiverThread {
    public:
        /** Queue capacity in snapshots. */
        static constexpr size_t kQueueCapacity = 500;

        ReceiverThread() = default;

        /** Calls End() defensively so the underlying std::thread is never destroyed while joinable. */
        ~ReceiverThread();

        ReceiverThread(const ReceiverThread&) = delete;
        ReceiverThread& operator=(const ReceiverThread&) = delete;

        /**
         * Adds a receiver for setup-time registration before Start().
         *
         * @param receiver Receiver to register.
         * @return True if the receiver was added.
         */
        bool AddDataReceiver(LogDataReceiver* receiver);

        /**
         * Adds a receiver and synchronously delivers a catch-up snapshot.
         *
         * Safe while the background thread is running.
         *
         * @param receiver Receiver to register.
         * @param currentSnapshot Snapshot to deliver immediately to the new receiver.
         * @return True if the receiver was added and caught up successfully.
         */
        bool AddDataReceiverAndCatchUp(LogDataReceiver* receiver, const LogTable& currentSnapshot);

        /** Test support: removes all receivers and clears fault state. */
        void ClearDataReceivers();

        /** Spawns the background dispatch thread. */
        void Start();

        /** Signals stop, drains the queue, and joins the thread. */
        void End();

        /**
         * Enqueues a deep-copied snapshot for background dispatch.
         *
         * Drops the snapshot and sets the fault state if the queue is full.
         *
         * @param snapshot Snapshot to enqueue.
         * @return True if the snapshot was queued.
         */
        bool Enqueue(LogStorage snapshot);

        /**
         * Returns the number of snapshots currently queued.
         *
         * @return Current queue size.
         */
        [[nodiscard]] size_t QueueSize() const;

        /**
         * Returns whether the queue has dropped a snapshot due to capacity.
         *
         * @return True if a snapshot has been dropped.
         */
        [[nodiscard]] bool HasFault() const;

    private:
        void Run(std::vector<LogDataReceiver*> initialReceivers);
        void Dispatch(LogStorage& entry);
        [[nodiscard]] std::optional<LogStorage> WaitAndPop();
        [[nodiscard]] std::vector<LogDataReceiver*> SnapshotReceivers() const;

        mutable std::mutex mutex_;
        std::condition_variable cv_;
        std::vector<LogDataReceiver*> dataReceivers_;
        std::deque<LogStorage> queue_;
        bool stopRequested_ = false;
        bool faulted_ = false;

        std::thread thread_;
    };
} // namespace akit
