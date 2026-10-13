#pragma once

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>
#include <utility>
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

        /** Starts receivers on the calling thread; each cycle is then written through DispatchNow instead of the queue. */
        void StartSynchronous();

        /** Signals stop, drains the queue, and joins the thread, or ends receivers directly in synchronous mode. */
        void End();

        /** Writes the table to every receiver on the calling thread. */
        void DispatchNow(const LogTable& table);

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
        void Dispatch(const LogTable& table, uint64_t sequence);
        void EndReceivers();
        [[nodiscard]] std::optional<std::pair<uint64_t, LogStorage>> WaitAndPop();
        [[nodiscard]] std::vector<LogDataReceiver*> SnapshotReceivers() const;
        /** Receivers registered before the snapshot with this sequence number was enqueued. */
        [[nodiscard]] std::vector<LogDataReceiver*> ReceiversFor(uint64_t sequence) const;

        mutable std::mutex mutex_;
        std::condition_variable cv_;
        std::vector<LogDataReceiver*> dataReceivers_;
        // First queue sequence each receiver should see; late receivers skip snapshots already
        // covered by their catch-up snapshot.
        std::unordered_map<LogDataReceiver*, uint64_t> firstSequence_;
        std::deque<std::pair<uint64_t, LogStorage>> queue_;
        uint64_t nextSequence_ = 0;
        bool stopRequested_ = false;
        bool faulted_ = false;
        bool synchronousRunning_ = false;

        std::thread thread_;
    };
} // namespace akit
