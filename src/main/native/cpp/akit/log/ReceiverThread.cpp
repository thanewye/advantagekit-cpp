#include "pch.h"

#include "akit/log/ReceiverThread.h"

#include <algorithm>
#include <exception>
#include <limits>
#include <utility>

#include <frc/Errors.h>

namespace akit {
    ReceiverThread::~ReceiverThread() {
        End();
    }

    bool ReceiverThread::AddDataReceiver(LogDataReceiver* receiver) {
        std::lock_guard lock(mutex_);
        if (std::ranges::find(dataReceivers_, receiver) != dataReceivers_.end()) return false;
        dataReceivers_.push_back(receiver);
        firstSequence_[receiver] = 0;
        return true;
    }

    bool ReceiverThread::AddDataReceiverAndCatchUp(LogDataReceiver* receiver, const LogTable& currentSnapshot) {
        std::lock_guard lock(mutex_);
        if (std::ranges::find(dataReceivers_, receiver) != dataReceivers_.end()) return false;
        dataReceivers_.push_back(receiver);
        // currentSnapshot already contains everything queued so far, so only later snapshots
        // should be dispatched to this receiver.
        firstSequence_[receiver] = nextSequence_;
        receiver->Start();
        receiver->PutTable(currentSnapshot);
        return true;
    }

    void ReceiverThread::ClearDataReceivers() {
        std::lock_guard lock(mutex_);
        dataReceivers_.clear();
        firstSequence_.clear();
        faulted_ = false;
    }

    void ReceiverThread::Start() {
        if (thread_.joinable()) return;
        std::vector<LogDataReceiver*> initialReceivers;
        {
            std::lock_guard lock(mutex_);
            stopRequested_ = false;
            initialReceivers = dataReceivers_;
        }
        thread_ = std::thread(&ReceiverThread::Run, this, std::move(initialReceivers));
    }

    void ReceiverThread::StartSynchronous() {
        if (thread_.joinable() || synchronousRunning_) return;
        synchronousRunning_ = true;
        for (auto* receiver : SnapshotReceivers())
            receiver->Start();
    }

    void ReceiverThread::End() {
        if (synchronousRunning_) {
            synchronousRunning_ = false;
            EndReceivers();
            return;
        }
        if (!thread_.joinable()) return;
        {
            std::lock_guard lock(mutex_);
            stopRequested_ = true;
        }
        cv_.notify_all();
        thread_.join();
    }

    bool ReceiverThread::Enqueue(LogStorage snapshot) {
        std::lock_guard lock(mutex_);
        if (queue_.size() >= kQueueCapacity) {
            faulted_ = true;
            return false;
        }
        queue_.emplace_back(nextSequence_++, std::move(snapshot));
        faulted_ = false;
        cv_.notify_one();
        return true;
    }

    size_t ReceiverThread::QueueSize() const {
        std::lock_guard lock(mutex_);
        return queue_.size();
    }

    bool ReceiverThread::HasFault() const {
        std::lock_guard lock(mutex_);
        return faulted_;
    }

    std::vector<LogDataReceiver*> ReceiverThread::SnapshotReceivers() const {
        std::lock_guard lock(mutex_);
        return dataReceivers_;
    }

    std::vector<LogDataReceiver*> ReceiverThread::ReceiversFor(uint64_t sequence) const {
        std::lock_guard lock(mutex_);
        std::vector<LogDataReceiver*> receivers;
        for (auto* receiver : dataReceivers_) {
            if (firstSequence_.at(receiver) <= sequence) receivers.push_back(receiver);
        }
        return receivers;
    }

    /** Blocks for the next snapshot; returns std::nullopt once stopped and drained. */
    std::optional<std::pair<uint64_t, LogStorage>> ReceiverThread::WaitAndPop() {
        std::unique_lock lock(mutex_);
        cv_.wait(lock, [this] { return !queue_.empty() || stopRequested_; });
        if (queue_.empty()) return std::nullopt;
        auto entry = std::move(queue_.front());
        queue_.pop_front();
        return entry;
    }

    void ReceiverThread::DispatchNow(const LogTable& table) {
        Dispatch(table, std::numeric_limits<uint64_t>::max());
    }

    void ReceiverThread::Dispatch(const LogTable& table, uint64_t sequence) {
        for (auto* receiver : ReceiversFor(sequence)) {
            try {
                receiver->PutTable(table);
            } catch (const std::exception& e) {
                FRC_ReportError(frc::err::Error, "[AdvantageKit] Data receiver threw an exception: {}", e.what());
            } catch (...) {
                FRC_ReportError(frc::err::Error, "[AdvantageKit] Data receiver threw an unknown exception.");
            }
        }
    }

    void ReceiverThread::Run(std::vector<LogDataReceiver*> initialReceivers) {
        for (auto* receiver : initialReceivers)
            receiver->Start();

        while (true) {
            auto entry = WaitAndPop();
            if (!entry) break;
            Dispatch(LogTable(entry->second), entry->first);
        }

        EndReceivers();
    }

    void ReceiverThread::EndReceivers() {
        for (auto* receiver : SnapshotReceivers())
            receiver->End();
    }
} // namespace akit
