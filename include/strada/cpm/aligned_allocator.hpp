#pragma once

#include <cstddef>
#include <cstdlib>
#include <new>

namespace strada::cpm {

/// Default cache-line alignment bytes (64 bytes).
constexpr std::size_t kDefaultAlignment = 64;

/// STL-compatible allocator that guarantees memory is allocated on aligned boundaries.
///
/// \tparam T Element type to allocate.
/// \tparam Alignment Boundary alignment in bytes.
template <typename T, std::size_t Alignment = kDefaultAlignment>
struct AlignedAllocator {
  using value_type = T;
  using pointer = T*;
  using const_pointer = const T*;
  using reference = T&;
  using const_reference = const T&;
  using size_type = std::size_t;
  using difference_type = std::ptrdiff_t;

  /// Rebind structure to allocate elements of different type.
  template <typename U>
  // NOLINTNEXTLINE(readability-identifier-naming)
  struct rebind {
    using other = AlignedAllocator<U, Alignment>;
  };

  /// Default constructor.
  AlignedAllocator() noexcept = default;

  /// Destructor.
  ~AlignedAllocator() = default;

  /// Copy constructor.
  AlignedAllocator(const AlignedAllocator&) noexcept = default;

  /// Copy assignment.
  auto operator=(const AlignedAllocator&) noexcept -> AlignedAllocator& = default;

  /// Move constructor.
  AlignedAllocator(AlignedAllocator&&) noexcept = default;

  /// Move assignment.
  auto operator=(AlignedAllocator&&) noexcept -> AlignedAllocator& = default;

  /// Constructor from another allocator type.
  template <typename U>
  // NOLINTNEXTLINE(google-explicit-constructor)
  constexpr AlignedAllocator(const AlignedAllocator<U, Alignment>& /*unused*/) noexcept {}

  /// Allocates aligned raw memory for count elements.
  ///
  /// \param count The number of elements.
  /// \return Pointer to the allocated memory.
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

  /// Deallocates the aligned memory.
  ///
  /// \param ptr Pointer to the memory block.
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

/// Equality operator for AlignedAllocator.
template <typename T, typename U, std::size_t Alignment>
auto operator==(const AlignedAllocator<T, Alignment>& /*unused*/,
                const AlignedAllocator<U, Alignment>& /*unused*/) noexcept -> bool {
  return true;
}

/// Inequality operator for AlignedAllocator.
template <typename T, typename U, std::size_t Alignment>
auto operator!=(const AlignedAllocator<T, Alignment>& /*unused*/,
                const AlignedAllocator<U, Alignment>& /*unused*/) noexcept -> bool {
  return false;
}

}  // namespace strada::cpm
