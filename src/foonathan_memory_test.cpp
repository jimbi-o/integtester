#include <set>
#include <foonathan/memory/container.hpp>
#include <foonathan/memory/memory_pool.hpp>
#include <foonathan/memory/namespace_alias.hpp>
#include <foonathan/memory/static_allocator.hpp>
#include "doctest/doctest.h"
TEST_CASE("null test") {
  memory::static_allocator_storage<4096u> storage;
  using static_pool = memory::memory_pool<memory::node_pool, memory::static_allocator>;
  static_pool pool(memory::set_node_size<int>::value, 4096, storage);
  memory::set<int, decltype(pool)> set(pool);
  set.insert(3);
  CHECK(set.find(3) != set.end());
}
