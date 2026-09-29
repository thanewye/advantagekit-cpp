#include "pch.h"

#include "akit/autolog/AutoLogOutputManager.h"

#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace akit {
    namespace {
        struct Entry {
            std::string key;
            std::function<void()> flush;
        };

        struct State {
            size_t generation = 0;
            std::vector<std::optional<Entry>> entries;
            std::unordered_map<std::string, size_t> keys;
        };

        State& GetState() {
            static State state;
            return state;
        }
    } // namespace

    AutoLogOutputManager::RegistrationId AutoLogOutputManager::Register(std::string key, std::function<void()> flush) {
        auto& state = GetState();
        if (state.keys.contains(key)) {
            detail::FailAutoLogRegistration("Duplicate autolog output key", key);
        }

        const size_t slot = state.entries.size();
        state.keys.emplace(key, slot);
        state.entries.emplace_back(Entry{std::move(key), std::move(flush)});
        return RegistrationId{state.generation, slot};
    }

    void AutoLogOutputManager::Unregister(const RegistrationId id) {
        auto& state = GetState();
        if (id.generation != state.generation || id.slot >= state.entries.size()) return;
        auto& entry = state.entries[id.slot];
        if (!entry.has_value()) return;
        state.keys.erase(entry->key);
        entry.reset();
    }

    void AutoLogOutputManager::Periodic() {
        auto& state = GetState();
        for (auto& entry : state.entries) {
            if (entry.has_value()) {
                entry->flush();
            }
        }
    }

    void AutoLogOutputManager::Reset() {
        auto& state = GetState();
        state.generation++;
        state.entries.clear();
        state.keys.clear();
    }
} // namespace akit
