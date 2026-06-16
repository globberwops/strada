#pragma once

#include <cstddef>
#include <cstdlib>
#include <new>

namespace strada::cpm {

constexpr std::size_t kDefaultAlignment = 64;

template <typename T, std::size_t Alignment = kDefaultAlignment>
struct AlignedAllocator {
  using value_type = T;
  using pointer = T*;
  using const_pointer = const T*;
  using reference = T&;
  using const_reference = const T&;
  using size_type = std::size_t;
  using difference_type = std::ptrdiff_t;

  template <typename U>
  // NOLINTNEXTLINE(readability-identifier-naming)
  struct rebind {
    using other = AlignedAllocator<U, Alignment>;
  };

  AlignedAllocator() noexcept = default;
  ~AlignedAllocator() = default;

  AlignedAllocator(const AlignedAllocator&) noexcept = default;
  auto operator=(const AlignedAllocator&) noexcept -> AlignedAllocator& = default;
  AlignedAllocator(AlignedAllocator&&) noexcept = default;
  auto operator=(AlignedAllocator&&) noexcept -> AlignedAllocator& = default;

  template <typename U>
  // NOLINTNEXTLINE(google-explicit-constructor)
  constexpr AlignedAllocator(const AlignedAllocator<U, Alignment>& /*unused*/) noexcept {}

  // NOLINTNEXTLINE(readability-identifier-naming)
  auto allocate(std::size_t count) -> T* {
    if (count == 0) {
      return nullptr;
    }
    if (count > static_cast<std::size_t>(-1) / sizeof(T)) {
      throw std::bad_array_new_length();
    }
    void* allocated_ptr = nullptr;
#ifdef _MSC_VER
    allocated_ptr = _aligned_malloc(count * sizeof(T), Alignment);
#else
    if (posix_memalign(&allocated_ptr, Alignment, count * sizeof(T)) != 0) {
      allocated_ptr = nullptr;
    }
#endif
    if (allocated_ptr == nullptr) {
      throw std::bad_alloc();
    }
    return static_cast<T*>(allocated_ptr);
  }

  // NOLINTNEXTLINE(readability-identifier-naming)
  void deallocate(T* ptr, std::size_t /*unused*/) noexcept {
#ifdef _MSC_VER
    _aligned_free(ptr);
#else
    // NOLINTNEXTLINE(cppcoreguidelines-no-malloc,cppcoreguidelines-owning-memory)
    std::free(ptr);
#endif
  }
};

template <typename T, typename U, std::size_t Alignment>
auto operator==(const AlignedAllocator<T, Alignment>& /*unused*/,
                const AlignedAllocator<U, Alignment>& /*unused*/) noexcept -> bool {
  return true;
}

template <typename T, typename U, std::size_t Alignment>
auto operator!=(const AlignedAllocator<T, Alignment>& /*unused*/,
                const AlignedAllocator<U, Alignment>& /*unused*/) noexcept -> bool {
  return false;
}

}  // namespace strada::cpm
