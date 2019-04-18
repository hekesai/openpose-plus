#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <numeric>
#include <utility>

namespace ttl
{
namespace internal
{
template <size_t off, typename T, size_t r, size_t... Is>
constexpr std::array<T, r - 1> shift_idx(const std::array<T, r> &a,
                                         std::index_sequence<Is...>)
{
    return std::array<T, r - 1>{std::get<Is + off>(a)...};
}

using rank_t = uint8_t;

template <rank_t r, typename dim_t = uint32_t> class basic_shape
{
  public:
    static constexpr rank_t rank = r;

    constexpr explicit basic_shape(const std::array<dim_t, r> &dims)
        : dims(dims)
    {
    }

    template <typename... D>
    constexpr explicit basic_shape(D... d) : dims({static_cast<dim_t>(d)...})
    {
        static_assert(sizeof...(D) == r, "invalid number of dims");
    }

    template <typename... I> dim_t offset(I... args) const
    {
        static_assert(sizeof...(I) == r, "invalid number of indexes");

        // TODO: expand the expression
        const std::array<dim_t, r> offs({static_cast<dim_t>(args)...});
        dim_t off = 0;
        for (rank_t i = 0; i < r; ++i) { off = off * dims[i] + offs[i]; }
        return off;
    }

    dim_t size() const
    {
        return std::accumulate(dims.begin(), dims.end(), 1,
                               std::multiplies<dim_t>());
    }

    dim_t subspace_size() const
    {
        return std::accumulate(dims.begin() + 1, dims.end(), 1,
                               std::multiplies<dim_t>());
    }

    template <rank_t corank = 1>
    using subshape_t = basic_shape<r - corank, dim_t>;

    template <rank_t corank = 1> basic_shape<r - corank, dim_t> subshape() const
    {
        static_assert(0 <= corank && corank <= r, "invalid corank");
        constexpr rank_t s = r - corank;
        return basic_shape<s, dim_t>(
            shift_idx<corank>(dims, std::make_index_sequence<s>()));
    }

    //   private:
    const std::array<dim_t, r> dims;
};
}  // namespace internal
}  // namespace ttl
