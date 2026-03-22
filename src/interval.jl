# Copyright (c) 2025: Joaquim Dias Garcia, Oscar Dowson and contributors
# Copyright (c) 2014-2021: David P. Sanders & Luis Benet
#
# Use of this source code is governed by an MIT-style license that can be found
# in the LICENSE.md file or at https://opensource.org/licenses/MIT.

# This type and the associated functions were inspired by IntervalArithmetic.jl

struct _Interval{T<:Real}
    lo::T
    hi::T

    function _Interval(lo::T, hi::T) where {T<:Real}
        @assert lo <= hi
        return new{T}(lo, hi)
    end
end

function Base.:+(a::_Interval{T}, b::_Interval{T}) where {T<:Real}
    return _Interval(a.lo + b.lo, a.hi + b.hi)
end

function Base.:*(x::T, a::_Interval{T}) where {T<:Real}
    if x >= zero(T)
        return _Interval(a.lo * x, a.hi * x)
    end
    return _Interval(a.hi * x, a.lo * x)
end
