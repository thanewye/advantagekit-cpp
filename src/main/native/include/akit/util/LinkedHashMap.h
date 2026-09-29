#pragma once

#include <list>
#include <unordered_map>
#include <utility>

namespace akit::util {
    template<typename Key, typename Value> class LinkedHashMap {
    public:
        void put(const Key& key, Value value) {
            auto it = index_.find(key);
            if (it != index_.end()) {
                it->second->second = std::move(value);
                return;
            }
            items_.emplace_back(key, std::move(value));
            index_[key] = std::prev(items_.end());
        }

        Value* get(const Key& key) {
            auto it = index_.find(key);
            return it == index_.end() ? nullptr : &it->second->second;
        }

        const Value* get(const Key& key) const {
            auto it = index_.find(key);
            return it == index_.end() ? nullptr : &it->second->second;
        }

        bool erase(const Key& key) {
            auto it = index_.find(key);
            if (it == index_.end()) return false;
            items_.erase(it->second);
            index_.erase(it);
            return true;
        }

        bool contains(const Key& key) const { return index_.contains(key); }

        std::size_t size() const { return items_.size(); }

        std::list<std::pair<Key, Value>>::iterator begin() { return items_.begin(); }
        std::list<std::pair<Key, Value>>::iterator end() { return items_.end(); }

        std::list<std::pair<Key, Value>>::const_iterator begin() const { return items_.begin(); }
        std::list<std::pair<Key, Value>>::const_iterator end() const { return items_.end(); }

        std::list<std::pair<Key, Value>>::const_iterator cbegin() const { return items_.cbegin(); }
        std::list<std::pair<Key, Value>>::const_iterator cend() const { return items_.cend(); }

    private:
        std::list<std::pair<Key, Value>> items_;
        std::unordered_map<Key, typename std::list<std::pair<Key, Value>>::iterator> index_;
    };
} // namespace akit::util