#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "akit/log/LogDataReceiver.h"
#include "akit/log/LogStorage.h"
#include "akit/log/LogTable.h"
#include "akit/log/ReceiverThread.h"

namespace {

    using akit::LogDataReceiver;
    using akit::LogStorage;
    using akit::LogTable;
    using akit::LogValue;
    using akit::ReceiverThread;

    // Records Start/PutTable/End calls in order. The blocking variant stalls inside PutTable
    // until Release() is called, which lets a test deterministically control how far the
    // dispatch loop has progressed. HasEnteredPutTable() flips true the instant PutTable is
    // entered (before any blocking), which is the signal tests use to know that a given
    // dispatch round's receiver snapshot has already been captured.
    class RecordingReceiver : public LogDataReceiver {
    public:
        explicit RecordingReceiver(std::string name, bool blockUntilReleased = false)
            : name_(std::move(name))
            , blockUntilReleased_(blockUntilReleased) {}

        void Start() override {
            std::lock_guard lock(logMutex_);
            log_.push_back("Start:" + name_);
        }

        void End() override {
            std::lock_guard lock(logMutex_);
            log_.push_back("End:" + name_);
        }

        void PutTable(const LogTable& table) override {
            enteredPutTable_.store(true, std::memory_order_relaxed);
            if (blockUntilReleased_) {
                std::unique_lock lock(blockMutex_);
                blockCv_.wait(lock, [this] { return released_; });
            }
            const LogValue* marker = table.Get("Marker");
            std::lock_guard lock(logMutex_);
            log_.push_back("PutTable:" + name_ + ":" + (marker != nullptr ? std::to_string(std::get<int64_t>(marker->value)) : "?"));
        }

        void Release() {
            std::lock_guard lock(blockMutex_);
            released_ = true;
            blockCv_.notify_all();
        }

        [[nodiscard]] bool HasEnteredPutTable() const { return enteredPutTable_.load(std::memory_order_relaxed); }

        [[nodiscard]] std::vector<std::string> Log() {
            std::lock_guard lock(logMutex_);
            return log_;
        }

    private:
        std::string name_;
        bool blockUntilReleased_;

        std::mutex logMutex_;
        std::vector<std::string> log_;

        std::mutex blockMutex_;
        std::condition_variable blockCv_;
        bool released_ = false;

        std::atomic<bool> enteredPutTable_{false};
    };

    LogStorage MakeSnapshot(int64_t marker) {
        LogStorage storage;
        storage.values.emplace("/Marker", LogValue{marker});
        return storage;
    }

    // Spin-waits up to ~1s for `condition` to become true. Used instead of a fixed sleep so
    // tests aren't flaky under slow CI but also don't pay a fixed delay in the common case.
    template<typename Predicate> bool WaitUntil(Predicate condition) {
        for (int i = 0; i < 200; ++i) {
            if (condition()) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return condition();
    }

    TEST(ReceiverThreadTest, DispatchPreservesOrderAcrossReceiversAndSnapshots) {
        ReceiverThread thread;
        RecordingReceiver r1("A");
        RecordingReceiver r2("B");
        thread.AddDataReceiver(&r1);
        thread.AddDataReceiver(&r2);
        thread.Start();

        ASSERT_TRUE(thread.Enqueue(MakeSnapshot(1)));
        ASSERT_TRUE(thread.Enqueue(MakeSnapshot(2)));
        thread.End();

        EXPECT_EQ(r1.Log(), (std::vector<std::string>{"Start:A", "PutTable:A:1", "PutTable:A:2", "End:A"}));
        EXPECT_EQ(r2.Log(), (std::vector<std::string>{"Start:B", "PutTable:B:1", "PutTable:B:2", "End:B"}));
    }

    TEST(ReceiverThreadTest, OverflowPastCapacityDropsAndSetsFault) {
        ReceiverThread thread;
        RecordingReceiver blocker("Blocker", /*blockUntilReleased=*/true);
        thread.AddDataReceiver(&blocker);
        thread.Start();

        ASSERT_TRUE(thread.Enqueue(MakeSnapshot(0)));
        ASSERT_TRUE(WaitUntil([&] { return blocker.HasEnteredPutTable(); }));

        for (size_t i = 0; i < ReceiverThread::kQueueCapacity; ++i) {
            ASSERT_TRUE(thread.Enqueue(MakeSnapshot(static_cast<int64_t>(i) + 1))) << "i=" << i;
        }
        EXPECT_FALSE(thread.HasFault());

        EXPECT_FALSE(thread.Enqueue(MakeSnapshot(9999))); // capacity exceeded: dropped
        EXPECT_TRUE(thread.HasFault());

        blocker.Release();
        thread.End();
    }

    TEST(ReceiverThreadTest, EndDrainsRemainingQueueBeforeCallingEndOnReceivers) {
        ReceiverThread thread;
        RecordingReceiver receiver("R");
        thread.AddDataReceiver(&receiver);
        thread.Start();

        for (int64_t i = 0; i < 10; ++i) {
            ASSERT_TRUE(thread.Enqueue(MakeSnapshot(i)));
        }
        thread.End();

        const auto log = receiver.Log();
        ASSERT_EQ(log.size(), 12u); // Start + 10 PutTable + End
        EXPECT_EQ(log.front(), "Start:R");
        EXPECT_EQ(log.back(), "End:R");
        for (int64_t i = 0; i < 10; ++i) {
            EXPECT_EQ(log[static_cast<size_t>(i) + 1], "PutTable:R:" + std::to_string(i));
        }
    }

    // Logger only ever adds a receiver while the thread is already running via
    // AddDataReceiverAndCatchUp (plain AddDataReceiver is setup-time-only — it never calls
    // Start(), so a receiver added through it after Start() would never get one). This test
    // exercises that actual late-registration path for race-freedom against an in-flight round.
    TEST(ReceiverThreadTest, AddDataReceiverAndCatchUpWhileRunningIsRaceFree) {
        ReceiverThread thread;
        RecordingReceiver early("Early", /*blockUntilReleased=*/true);
        thread.AddDataReceiver(&early);
        thread.Start();

        ASSERT_TRUE(thread.Enqueue(MakeSnapshot(1)));
        // Once Early has entered PutTable for snapshot 1, this round's receiver-list snapshot
        // has already been captured (without "late" in it), so adding "late" now cannot affect
        // this round no matter how the two threads are subsequently scheduled.
        ASSERT_TRUE(WaitUntil([&] { return early.HasEnteredPutTable(); }));

        RecordingReceiver late("Late");
        LogStorage catchUpStorage = MakeSnapshot(99);
        LogTable catchUpView(catchUpStorage);
        EXPECT_TRUE(thread.AddDataReceiverAndCatchUp(&late, catchUpView));
        EXPECT_FALSE(thread.AddDataReceiverAndCatchUp(&late, catchUpView));                  // duplicate: no re-delivery
        EXPECT_EQ(late.Log(), (std::vector<std::string>{"Start:Late", "PutTable:Late:99"})); // synchronous, no wait needed

        early.Release();
        ASSERT_TRUE(thread.Enqueue(MakeSnapshot(2)));
        thread.End();

        EXPECT_EQ(early.Log(), (std::vector<std::string>{"Start:Early", "PutTable:Early:1", "PutTable:Early:2", "End:Early"}));
        // "late" was registered (with its own synchronous Start()+catch-up) only after snapshot
        // 1's dispatch round had already started, so it must never see snapshot 1 itself.
        EXPECT_EQ(late.Log(), (std::vector<std::string>{"Start:Late", "PutTable:Late:99", "PutTable:Late:2", "End:Late"}));
    }

    TEST(ReceiverThreadTest, AddDataReceiverAndCatchUpDeliversSynchronouslyAndIsNotDuplicated) {
        ReceiverThread thread;
        thread.Start();

        LogStorage catchUpStorage;
        catchUpStorage.values.emplace("/Marker", LogValue{int64_t{42}});
        LogTable catchUpView(catchUpStorage);

        RecordingReceiver receiver("R");
        EXPECT_TRUE(thread.AddDataReceiverAndCatchUp(&receiver, catchUpView));
        // Synchronous: visible immediately, no waiting needed.
        EXPECT_EQ(receiver.Log(), (std::vector<std::string>{"Start:R", "PutTable:R:42"}));

        EXPECT_FALSE(thread.AddDataReceiverAndCatchUp(&receiver, catchUpView)); // duplicate: no re-delivery
        EXPECT_EQ(receiver.Log(), (std::vector<std::string>{"Start:R", "PutTable:R:42"}));

        ASSERT_TRUE(thread.Enqueue(MakeSnapshot(7)));
        thread.End(); // blocks until snapshot 7 is dispatched and End() is called

        EXPECT_EQ(receiver.Log(), (std::vector<std::string>{"Start:R", "PutTable:R:42", "PutTable:R:7", "End:R"}));
    }

} // namespace
