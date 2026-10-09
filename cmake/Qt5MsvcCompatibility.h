#pragma once
// Qt 5.15 checked iterators were removed by recent MSVC standard libraries.
#if defined(_MSC_VER) && _MSC_VER >= 1950
#include <cstddef>
namespace stdext
{
template <typename T> constexpr T* make_checked_array_iterator(T* p, std::size_t) noexcept
{
    return p;
}
template <typename T> constexpr T* make_unchecked_array_iterator(T* p) noexcept
{
    return p;
}
} // namespace stdext
#endif
