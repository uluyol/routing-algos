// Forked from github.com/ngvozdiev/ncode-common. Original license below:
//
// MIT License
//
// Copyright (c) 2017 ngvozdiev
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef NCODE_NET_TRIE_H
#define NCODE_NET_TRIE_H

#include <arpa/inet.h>

#include "absl/strings/substitute.h"
#include "glog/logging.h"
#include "third_party/ncode-common/net_common.h"
#include "third_party/ncode-common/stats.h"

namespace nc {
namespace net {

struct TrieStats {
  size_t size_bytes = 0;
  size_t num_nodes = 0;
  std::vector<size_t> children_per_node_percentiles;

  std::string ToString() {
    return absl::Substitute(
        "bytes: $0, nodes: $1, children: $2/$3/$4", size_bytes, num_nodes,
        children_per_node_percentiles[10], children_per_node_percentiles[50],
        children_per_node_percentiles[90]);
  }
};

template <typename T, typename V>
struct TrieNode {
  // The values that share the same prefix.
  std::vector<V> values;

  // This node's children.
  std::vector<std::pair<T, TrieNode<T, V>>> children;
};

template <typename T, typename V>
class Trie {
 public:
  // Adds a sequence of elements to this trie. The sequence is identified by a
  // value, which will later be returned by calls to SequencesWithPrefix.
  void Add(const std::vector<T>& sequence, V value) {
    CHECK(!sequence.empty());
    AddRecursive(sequence, value, 0, &root_);
  }

  // Returns the values that have a given prefix. Reference only valid until
  // next call to Add.
  const std::vector<V>& SequencesWithPrefix(
      const std::vector<T>& prefix) const {
    if (prefix.empty()) {
      return root_.values;
    }

    CHECK(!prefix.empty());
    return HasPrefixRecursive(prefix, 0, &root_);
  }

  TrieStats GetStats() const {
    TrieStats stats;
    stats.size_bytes = sizeof(std::vector<V>) + sizeof(TrieNode<T, V>);

    std::vector<size_t> children_counts;
    PopulateStatsRecursive(root_, &stats, &children_counts);
    stats.children_per_node_percentiles = Percentiles(&children_counts);
    return stats;
  }

 private:
  void AddRecursive(const std::vector<T>& sequence, V value, size_t from,
                    TrieNode<T, V>* at) {
    at->values.emplace_back(value);
    if (from == sequence.size()) {
      return;
    }

    const T& element = sequence[from];
    TrieNode<T, V>* node_ptr = nullptr;
    for (auto& t_and_node : at->children) {
      if (t_and_node.first == element) {
        node_ptr = &t_and_node.second;
        break;
      }
    }

    if (node_ptr == nullptr) {
      at->children.emplace_back(std::piecewise_construct,
                                std::forward_as_tuple(element),
                                std::forward_as_tuple());
      node_ptr = &at->children.back().second;
    }

    AddRecursive(sequence, value, from + 1, node_ptr);
  }

  const std::vector<V>& HasPrefixRecursive(const std::vector<T>& prefix,
                                           size_t from,
                                           const TrieNode<T, V>* at) const {
    CHECK(from != prefix.size());

    const T& element = prefix[from];
    const TrieNode<T, V>* in_trie = nullptr;
    for (const auto& t_and_node : at->children) {
      if (t_and_node.first == element) {
        in_trie = &t_and_node.second;
        break;
      }
    }

    if (in_trie == nullptr) {
      return empty_;
    }

    if (from == prefix.size() - 1) {
      return in_trie->values;
    }

    return HasPrefixRecursive(prefix, from + 1, in_trie);
  }

  void PopulateStatsRecursive(const TrieNode<T, V>& at, TrieStats* stats,
                              std::vector<size_t>* children_counts) const {
    stats->size_bytes +=
        at.children.size() * sizeof(std::pair<T, TrieNode<T, V>>);
    stats->size_bytes += at.values.size() * sizeof(V);
    children_counts->emplace_back(at.children.size());
    stats->num_nodes += at.children.size();

    for (const auto& child : at.children) {
      PopulateStatsRecursive(child.second, stats, children_counts);
    }
  }

  std::vector<V> empty_;

  TrieNode<T, V> root_;
};

template <typename T>
class IPRangeTrie {
 public:
  void Add(const IPRange& ip_range, const T& value) {
    trie_.Add(ToBoolVector(ip_range), value);
  }

  const std::vector<T>& ValuesWithPrefix(const IPRange& ip_range) const {
    return trie_.SequencesWithPrefix(ToBoolVector(ip_range));
  }

 private:
  static std::vector<bool> ToBoolVector(const IPRange& ip_range) {
    uint32_t range_net_order = htonl(ip_range.base_address().Raw());
    std::vector<bool> bits_vector(ip_range.mask_len());
    for (size_t i = 0; i < ip_range.mask_len(); ++i) {
      bool bit_value = range_net_order & (1 << i);
      bits_vector[i] = bit_value;
    }

    return bits_vector;
  }

  Trie<bool, T> trie_;
};

}  // namespace net
}  // namespace nc

#endif
