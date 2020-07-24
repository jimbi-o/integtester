#include <cstdint>
#include <set>
#include <vector>
#include <unordered_map>
namespace illuminate {
template <typename T>
struct allocator_t {
 public:
  typedef T value_type;
  typedef size_t size_type;
  typedef ptrdiff_t difference_type;
  typedef std::true_type propagate_on_container_move_assignment;
  typedef std::true_type is_always_equal;
  allocator_t() throw() { }
  allocator_t(const allocator_t& __a) throw() {}
  template<typename T1>
  allocator_t(const allocator_t<T1>&) throw() {}
  ~allocator_t() throw() { }
  [[nodiscard]] constexpr T* allocate(std::size_t n) { return nullptr; }
  constexpr void deallocate(T* p, std::size_t n) {}
};
template <typename T>
using set = std::set<T, allocator_t<T>>;
template <typename T>
using unordered_map = std::unordered_map<T, allocator_t<T>>;
template <typename T>
using vector = std::vector<T, allocator_t<T>>;
class MemoryManager {
 public:
  template <typename T>
  allocator_t<T> GetAllocatorLocal();
  template <typename T>
  allocator_t<T> GetAllocatorOneFrame();
  template <typename T>
  allocator_t<T> GetAllocatorFrameBuffered();
  void SucceedToNextFrame(); // for frame buffered
  void* GetCurrentHead(); // frame buffered
  void* GetPrevHead(); // frame buffered
 private:
};
}
#include "doctest/doctest.h"
TEST_CASE("inside a function") {
  using namespace illuminate;
  MemoryManager memory;
  vector<uint32_t> vec(memory.GetAllocatorLocal<uint32_t>());
  for (uint32_t i = 0; i < 10; i++) {
    vec.push_back(i);
  }
  CHECK(vec.back() == 9);
}
TEST_CASE("passing pointer from a function to another") {
  using namespace illuminate;
  MemoryManager memory;
  uint32_t* ptr = nullptr;
  {
    vector<uint32_t> vec(memory.GetAllocatorOneFrame<uint32_t>());
    for (uint32_t i = 0; i < 10; i++) {
      vec.push_back(i);
    }
    ptr = vec.data();
  }
  uint32_t* ptr2 = nullptr;
  {
    vector<uint32_t> vec(memory.GetAllocatorOneFrame<uint32_t>());
    for (uint32_t i = 0; i < 10; i++) {
      vec.push_back(100);
    }
    ptr2 = &vec.back();
  }
  CHECK(ptr[9] == 9);
  CHECK(*ptr2 == 100);
}
TEST_CASE("frame buffered") {
  using namespace illuminate;
  MemoryManager memory;
  static_cast<uint32_t*>(memory.GetPrevHead())[9] = 255;
  CHECK(static_cast<uint32_t*>(memory.GetCurrentHead())[9] == 255);
  memory.SucceedToNextFrame();
  CHECK(static_cast<uint32_t*>(memory.GetPrevHead())[9] == 255);
  uint32_t* ptr = nullptr;
  {
    vector<uint32_t> vec(memory.GetAllocatorFrameBuffered<uint32_t>());
    for (uint32_t i = 0; i < 10; i++) {
      vec.push_back(i);
    }
  }
  CHECK(static_cast<uint32_t*>(memory.GetCurrentHead())[9] == 9);
  memory.SucceedToNextFrame();
  CHECK(static_cast<uint32_t*>(memory.GetPrevHead())[9] == 9);
}
