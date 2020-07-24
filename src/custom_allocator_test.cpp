#include <cstdint>
#include <set>
#include <vector>
#include <unordered_map>
namespace illuminate::memory {
constexpr std::uintptr_t AlignAddress(const std::uintptr_t addr, const size_t align) {
  const auto mask = align - 1;
  //assert((align & mask) == 0);
  return (addr + mask) & ~mask;
}
void* AlignAddress(const void* ptr, const size_t align) {
  auto addr = AlignAddress(reinterpret_cast<std::uintptr_t>(ptr), align);
  return reinterpret_cast<void*>(addr);
}
class LinearAllocator {
 public:
  explicit LinearAllocator(void* const buffer, const size_t size_in_bytes) : head_(reinterpret_cast<uintptr_t>(buffer)), offset_(0), size_in_bytes_(size_in_bytes) {}
  ~LinearAllocator() {}
  void* Alloc(const size_t size_in_bytes, const size_t alignment_in_bytes) {
    auto aligned_address = AlignAddress(head_ + offset_, alignment_in_bytes);
    offset_ = aligned_address - head_ + size_in_bytes;
    // assert(offset_ <= size_in_bytes_);
    return reinterpret_cast<void*>(aligned_address);
  }
  constexpr void Free() { offset_ = 0; }
  constexpr size_t GetOffset() const { return offset_; }
 private:
  std::uintptr_t head_;
  size_t offset_;
  size_t size_in_bytes_;
};
template <typename T>
class Allocator {
 public:
  typedef T value_type;
  typedef size_t size_type;
  typedef ptrdiff_t difference_type;
  typedef std::true_type propagate_on_container_move_assignment;
  typedef std::true_type is_always_equal;
  Allocator() throw() { }
  Allocator(const Allocator& __a) throw() {}
  template<typename T1>
  Allocator(const Allocator<T1>&) throw() {}
  ~Allocator() throw() { }
  [[nodiscard]] constexpr T* allocate(std::size_t n) { return nullptr; }
  constexpr void deallocate(T* p, std::size_t n) {}
};
template <typename T>
using set = std::set<T, Allocator<T>>;
template <typename T>
using unordered_map = std::unordered_map<T, Allocator<T>>;
template <typename T>
using vector = std::vector<T, Allocator<T>>;
class MemoryManager {
 public:
  template <typename T>
  Allocator<T> GetAllocatorLocal();
  template <typename T>
  Allocator<T> GetAllocatorOneFrame();
  template <typename T>
  Allocator<T> GetAllocatorFrameBuffered();
  void SucceedToNextFrame(); // for frame buffered
  void* GetCurrentHead(); // frame buffered
  void* GetPrevHead(); // frame buffered
 private:
};
}
#include "doctest/doctest.h"
TEST_CASE("align address") {
  using namespace illuminate::memory;
  for (size_t align = 1; align <= 256; align *= 2) {
    CAPTURE(align);
    std::uintptr_t addr = align * 16;
    do {
      CAPTURE(addr);
      const auto aligned_addr = AlignAddress(addr, align);
      CHECK(addr <= aligned_addr);
      CHECK(aligned_addr - addr <= align);
      CHECK(aligned_addr % align == 0);
      ++addr;
    } while(addr % align != 0);
  }
}
TEST_CASE("linear allocator") {
  using namespace illuminate::memory;
  uint8_t buffer[1024]{};
  LinearAllocator allocator(buffer, 1024);
  size_t alloc_size = 4, alignment = 64;
  auto ptr = allocator.Alloc(alloc_size, alignment);
  CHECK(allocator.GetOffset() >= alloc_size);
  CHECK(reinterpret_cast<std::uintptr_t>(ptr) % alignment == 0);
  CHECK(ptr);
  auto prev_offset = allocator.GetOffset();
  auto prev_ptr = reinterpret_cast<std::uintptr_t>(ptr);
  auto prev_alloc_size = alloc_size;
  alloc_size = 32, alignment = 8;
  ptr = allocator.Alloc(alloc_size, alignment);
  CHECK(reinterpret_cast<std::uintptr_t>(ptr) - prev_ptr >= prev_alloc_size);
  CHECK(allocator.GetOffset() - prev_offset >= alloc_size);
  CHECK(reinterpret_cast<std::uintptr_t>(ptr) % alignment == 0);
  CHECK(ptr);
  prev_offset = allocator.GetOffset();
  prev_ptr = reinterpret_cast<std::uintptr_t>(ptr);
  prev_alloc_size = alloc_size;
  alloc_size = 256, alignment = 16;
  ptr = allocator.Alloc(alloc_size, alignment);
  CHECK(reinterpret_cast<std::uintptr_t>(ptr) - prev_ptr >= prev_alloc_size);
  CHECK(allocator.GetOffset() - prev_offset >= alloc_size);
  CHECK(reinterpret_cast<std::uintptr_t>(ptr) % alignment == 0);
  CHECK(ptr);
  allocator.Free();
  CHECK(allocator.GetOffset() == 0);
  alloc_size = 4, alignment = 64;
  ptr = allocator.Alloc(alloc_size, alignment);
  CHECK(reinterpret_cast<std::uintptr_t>(ptr) - prev_ptr >= prev_alloc_size);
  CHECK(allocator.GetOffset() >= alloc_size);
  CHECK(reinterpret_cast<std::uintptr_t>(ptr) % alignment == 0);
  CHECK(ptr);
  prev_offset = allocator.GetOffset();
  prev_ptr = reinterpret_cast<std::uintptr_t>(ptr);
  prev_alloc_size = alloc_size;
  alloc_size = 32, alignment = 8;
  ptr = allocator.Alloc(alloc_size, alignment);
  CHECK(reinterpret_cast<std::uintptr_t>(ptr) - prev_ptr >= prev_alloc_size);
  CHECK(allocator.GetOffset() - prev_offset >= alloc_size);
  CHECK(reinterpret_cast<std::uintptr_t>(ptr) % alignment == 0);
  CHECK(ptr);
  prev_offset = allocator.GetOffset();
  prev_ptr = reinterpret_cast<std::uintptr_t>(ptr);
  prev_alloc_size = alloc_size;
  alloc_size = 256, alignment = 16;
  ptr = allocator.Alloc(alloc_size, alignment);
  CHECK(reinterpret_cast<std::uintptr_t>(ptr) - prev_ptr >= prev_alloc_size);
  CHECK(allocator.GetOffset() - prev_offset >= alloc_size);
  CHECK(reinterpret_cast<std::uintptr_t>(ptr) % alignment == 0);
  CHECK(ptr);
}
#if 0
TEST_CASE("Allocator class") {
  using namespace illuminate::memory;
  Allocator<uint32_t> a;
}
TEST_CASE("inside a function") {
  using namespace illuminate::memory;
  MemoryManager memory;
  vector<uint32_t> vec(memory.GetAllocatorLocal<uint32_t>());
  for (uint32_t i = 0; i < 10; i++) {
    vec.push_back(i);
  }
  CHECK(vec.back() == 9);
}
TEST_CASE("passing pointer from a function to another") {
  using namespace illuminate::memory;
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
  using namespace illuminate::memory;
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
#endif
