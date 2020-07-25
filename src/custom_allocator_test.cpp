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
  inline void* Alloc(const size_t size_in_bytes, const size_t alignment_in_bytes) {
    auto aligned_address = AlignAddress(head_ + offset_, alignment_in_bytes);
    offset_ = aligned_address - head_ + size_in_bytes;
    // assert(offset_ <= size_in_bytes_);
    return reinterpret_cast<void*>(aligned_address);
  }
  inline constexpr void Free() { offset_ = 0; }
  inline constexpr size_t GetOffset() const { return offset_; }
 private:
  LinearAllocator() = delete;
  LinearAllocator(const LinearAllocator&) = delete;
  LinearAllocator& operator=(const LinearAllocator&) = delete;
  std::uintptr_t head_;
  size_t offset_;
  size_t size_in_bytes_;
};
template <typename T>
class MemoryJanitor {
 public:
  explicit MemoryJanitor(T* allocator) : allocator_(allocator) {};
  ~MemoryJanitor() { allocator_->Free(); }
 private:
  T* allocator_ = nullptr;
};
using memory_janitor_t = MemoryJanitor<LinearAllocator>;
template <typename T, typename A, size_t size_in_bytes, size_t align>
class Allocator {
 public:
  typedef T value_type;
  typedef size_t size_type;
  typedef ptrdiff_t difference_type;
  typedef std::true_type propagate_on_container_move_assignment;
  typedef std::true_type is_always_equal;
  explicit Allocator(A* const allocator) throw() : allocator_(allocator) { }
  explicit Allocator(const Allocator& a) throw() : allocator_(a.allocator_) {}
  template<typename U>
  explicit Allocator(const Allocator<U, A, sizeof(U), _Alignof(U)>& a) throw() : allocator_(a.GetA()) {}
  ~Allocator() throw() { }
  [[nodiscard]] constexpr T* allocate(std::size_t n) {
    auto ptr = allocator_->Alloc(size_in_bytes * n, align);
    return static_cast<T*>(ptr);
  }
  constexpr void deallocate(T* p, std::size_t n) {}
  template<typename U>
  struct rebind { typedef Allocator<U, A, size_in_bytes, align> other; };
  constexpr A* GetA() const { return allocator_; }
 private:
  Allocator() = delete;
  A* allocator_ = nullptr;
};
template <typename T>
using allocator_t = Allocator<T, LinearAllocator, sizeof(T), _Alignof(T)>;
// Calling resize() or reserve() is recommended before inserting elements due to LinearAllocator.
template <typename T>
using set = std::set<T, allocator_t<T>>;
template <typename T>
using unordered_map = std::unordered_map<T, allocator_t<T>>;
template <typename T>
using vector = std::vector<T, allocator_t<T>>;
class MemoryManager {
 public:
  struct MemoryManagerConfig {
    uint32_t frame_num;
    void* buffer_tmp;
    void* buffer_one_frame;
    void** buffer_frame_buffered;
    size_t size_in_bytes_tmp;
    size_t size_in_bytes_one_frame;
    size_t size_in_bytes_frame_buffered;
  };
  explicit MemoryManager(MemoryManagerConfig&& config)
      : config_(std::move(config)), prev_frame_index_(config.frame_num - 1)
  {
    allocator_one_frame_ = new LinearAllocator(config_.buffer_one_frame, config_.size_in_bytes_one_frame);
    allocator_frame_buffered_ = new LinearAllocator*[config_.frame_num];
    for (uint32_t i = 0; i < config_.frame_num; i++) {
      allocator_frame_buffered_[i] = new LinearAllocator(config_.buffer_frame_buffered[i], config_.size_in_bytes_frame_buffered);
    }
  }
  ~MemoryManager() {
    delete allocator_one_frame_;
    for (uint32_t i = 0; i < config_.frame_num; i++) {
      delete allocator_frame_buffered_[i];
    }
    delete[] allocator_frame_buffered_;
  }
  template <typename T>
  constexpr allocator_t<T> GetAllocatorOneFrame() const { return allocator_t<T>(allocator_one_frame_); }
  template <typename T>
  constexpr allocator_t<T> GetAllocatorFrameBuffered() const { return allocator_t<T>(allocator_frame_buffered_[frame_index_]); }
  constexpr uint32_t GetFrameIndex() const { return frame_index_; }
  constexpr uint32_t GetPrevFrameIndex() const { return prev_frame_index_; }
  constexpr void SucceedToNextFrame() { // for frame buffered
    prev_frame_index_ = frame_index_;
    frame_index_++;
    if (frame_index_ >= config_.frame_num) frame_index_ = 0;
    allocator_frame_buffered_[frame_index_]->Free();
    allocator_one_frame_->Free();
  }
  constexpr void* GetCurrentHead() const { // for frame buffered
    return config_.buffer_frame_buffered[frame_index_];
  }
  constexpr void* GetPrevHead() const { // for frame buffered
    return config_.buffer_frame_buffered[prev_frame_index_];
  }
 private:
  MemoryManagerConfig config_;
  LinearAllocator* allocator_one_frame_ = nullptr;
  LinearAllocator** allocator_frame_buffered_ = nullptr;
  uint32_t frame_index_ = 0;
  uint32_t prev_frame_index_ = 0;
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
TEST_CASE("allocator_t") {
  using namespace illuminate::memory;
  uint8_t buffer[1024]{};
  LinearAllocator la(buffer, 1024);
  allocator_t<uint32_t> a(&la);
  auto ptr = a.allocate(1);
  auto ptr2 = a.allocate(2);
  auto ptr3 = a.allocate(5);
  *ptr = 1;
  ptr2[0] = 2;
  ptr2[1] = 3;
  ptr3[0] = 4;
  ptr3[1] = 5;
  ptr3[2] = 6;
  ptr3[3] = 7;
  ptr3[4] = 8;
  for (uint32_t i = 0; i < 8; i++) {
    CAPTURE(i);
    CHECK(ptr[i] == i + 1);
  }
}
TEST_CASE("allocator_t with single vector") {
  using namespace illuminate::memory;
  uint8_t buffer[1024]{};
  LinearAllocator la(buffer, 1024);
  allocator_t<uint32_t> a(&la);
  vector<uint32_t> v(a);
  for (uint32_t i = 0; i < 64; i++) {
    v.push_back(i);
  }
  for (uint32_t i = 0; i < 64; i++) {
    CAPTURE(i);
    CHECK(v[i] == i);
  }
}
TEST_CASE("allocator_t with multiple vector") {
  using namespace illuminate::memory;
  uint8_t buffer[10 * 1024]{};
  LinearAllocator la(buffer, 10 * 1024);
  allocator_t<uint32_t> a(&la);
  vector<uint32_t> v1(a);
  vector<uint32_t> v2(a);
  for (uint32_t i = 0; i < 64; i++) {
    v1.push_back(i);
    v2.push_back(64-i);
  }
  for (uint32_t i = 0; i < 64; i++) {
    CAPTURE(i);
    CHECK(v1[i] == i);
    CHECK(v2[i] == 64-i);
  }
  while (!v1.empty()) {
    CHECK(v1.back() == v1.size() - 1);
    v1.pop_back();
  }
  while (!v2.empty()) {
    CHECK(v2.back() == 65 - v2.size());
    v2.pop_back();
  }
}
namespace {
illuminate::memory::MemoryManager::MemoryManagerConfig CreateTestMemoryManagerConfig() {
  const uint32_t size_in_bytes_tmp = 16 * 1024;
  const uint32_t size_in_bytes_one_frame = 16 * 1024;
  const uint32_t size_in_bytes_frame_buffered = 16 * 1024;
  static uint8_t buffer_tmp[size_in_bytes_tmp]{};
  static uint8_t buffer_one_frame[size_in_bytes_one_frame]{};
  static uint8_t buffer_frame_buffered1[size_in_bytes_frame_buffered]{};
  static uint8_t buffer_frame_buffered2[size_in_bytes_frame_buffered]{};
  static void* buffer_frame_buffered[2]{buffer_frame_buffered1, buffer_frame_buffered2};
  using namespace illuminate::memory;
  MemoryManager::MemoryManagerConfig config{};
  config.frame_num = 2;
  config.buffer_tmp = buffer_tmp;
  config.buffer_one_frame = buffer_one_frame;
  config.buffer_frame_buffered = buffer_frame_buffered;
  config.size_in_bytes_tmp = size_in_bytes_tmp;
  config.size_in_bytes_one_frame = size_in_bytes_one_frame;
  config.size_in_bytes_frame_buffered = size_in_bytes_frame_buffered;
  return config;
}
}
TEST_CASE("buffer released when out of scope") {
  using namespace illuminate::memory;
  uint8_t buffer[16 * 1024]{};
  LinearAllocator linear_allocator(buffer, 16 * 1024);
  void* ptr = nullptr;
  {
    memory_janitor_t janitor(&linear_allocator);
    allocator_t<uint32_t> a(&linear_allocator);
    vector<uint32_t> vec(a);
    for (uint32_t i = 0; i < 10; i++) {
      vec.push_back(i);
    }
    CHECK(vec.back() == 9);
    ptr = vec.data();
  }
  {
    memory_janitor_t janitor(&linear_allocator);
    allocator_t<uint32_t> a(&linear_allocator);
    vector<uint32_t> vec(a);
    for (uint32_t i = 0; i < 10; i++) {
      vec.push_back(i*10);
    }
    CHECK(vec.back() == 90);
    CHECK(ptr == vec.data());
    vector<uint32_t> vec2(a);
    for (uint32_t i = 0; i < 10; i++) {
      vec2.push_back(i*10);
    }
    CHECK(vec2.back() == 90);
    CHECK(vec2.data() > vec.data());
  }
}
#if 0
TEST_CASE("inside a function") {
  using namespace illuminate::memory;
  MemoryManager memory(CreateTestMemoryManagerConfig());
  void* ptr = nullptr;
  {
    vector<uint32_t> vec(memory.GetAllocatorLocal<uint32_t>());
    for (uint32_t i = 0; i < 10; i++) {
      vec.push_back(i);
    }
    CHECK(vec.back() == 9);
    ptr = vec.data();
  }
  {
    auto a = memory.GetAllocatorLocal<uint32_t>();
    vector<uint32_t> vec(a);
    for (uint32_t i = 0; i < 10; i++) {
      vec.push_back(i*10);
    }
    CHECK(vec.back() == 90);
    CHECK(ptr == vec.data());
    vector<uint32_t> vec2(a);
    for (uint32_t i = 0; i < 10; i++) {
      vec2.push_back(i*10);
    }
    CHECK(vec2.back() == 90);
    CHECK(vec2.data() > vec.data());
  }
}
#endif
TEST_CASE("one frame allocator") {
  using namespace illuminate::memory;
  MemoryManager memory(CreateTestMemoryManagerConfig());
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
TEST_CASE("frame success - one frame") {
  using namespace illuminate::memory;
  MemoryManager memory(CreateTestMemoryManagerConfig());
  uint32_t* ptr = nullptr;
  {
    vector<uint32_t> vec(memory.GetAllocatorOneFrame<uint32_t>());
    for (uint32_t i = 0; i < 10; i++) {
      vec.push_back(i);
    }
    ptr = vec.data();
  }
  {
    vector<uint32_t> vec(memory.GetAllocatorOneFrame<uint32_t>());
    for (uint32_t i = 0; i < 10; i++) {
      vec.push_back(i);
    }
    CHECK(ptr < vec.data());
  }
  memory.SucceedToNextFrame();
  {
    vector<uint32_t> vec(memory.GetAllocatorOneFrame<uint32_t>());
    for (uint32_t i = 0; i < 10; i++) {
      vec.push_back(i);
    }
    CHECK(ptr == vec.data());
  }
}
TEST_CASE("frame success - frame buffered") {
  using namespace illuminate::memory;
  MemoryManager memory(CreateTestMemoryManagerConfig());
  static_cast<uint32_t*>(memory.GetCurrentHead())[9] = 255;
  CHECK(static_cast<uint32_t*>(memory.GetCurrentHead())[9] == 255);
  memory.SucceedToNextFrame();
  static_cast<uint32_t*>(memory.GetCurrentHead())[9] = 1024;
  CHECK(static_cast<uint32_t*>(memory.GetCurrentHead())[9] == 1024);
  CHECK(static_cast<uint32_t*>(memory.GetPrevHead())[9] == 255);
  memory.SucceedToNextFrame();
  CHECK(static_cast<uint32_t*>(memory.GetPrevHead())[9] == 1024);
}
TEST_CASE("frame success - frame buffered with allocator") {
  using namespace illuminate::memory;
  MemoryManager memory(CreateTestMemoryManagerConfig());
  uint32_t* ptr = nullptr;
  {
    vector<uint32_t> vec(memory.GetAllocatorFrameBuffered<uint32_t>());
    for (uint32_t i = 0; i < 10; i++) {
      vec.push_back(i);
    }
    ptr = vec.data();
  }
  {
    vector<uint32_t> vec(memory.GetAllocatorFrameBuffered<uint32_t>());
    for (uint32_t i = 0; i < 10; i++) {
      vec.push_back(i);
    }
    CHECK(ptr < vec.data());
  }
  memory.SucceedToNextFrame();
  {
    vector<uint32_t> vec(memory.GetAllocatorFrameBuffered<uint32_t>());
    for (uint32_t i = 0; i < 10; i++) {
      vec.push_back(i);
    }
    CHECK(ptr != vec.data());
  }
  memory.SucceedToNextFrame();
  {
    vector<uint32_t> vec(memory.GetAllocatorFrameBuffered<uint32_t>());
    for (uint32_t i = 0; i < 10; i++) {
      vec.push_back(i);
    }
    CHECK(ptr == vec.data());
  }
}
