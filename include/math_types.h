#pragma once

#include <glm/glm.hpp>
#include <array>

template<int N, typename F, typename T>
struct _bbox_base
{
	typedef _bbox_base<N,F,T> this_type;
	typedef T container_type;
	typedef F value_type;

	T max;
	T min;

	_bbox_base<N,F,T>()
	{
		clear();
	}

	_bbox_base<N, F, T>(const T& tmin, const T& tmax)
	: min(tmin)
	, max(tmax)
	{}

	_bbox_base<N, F, T>(const T& tsingle)
	: min(tsingle)
	, max(tsingle)
	{}

	this_type& clear()
	{
		max = T(value_type(INT_MIN));
		min = T(value_type(INT_MAX));
		return (*this);
	}

	this_type& extend(const T& t)
	{
		for (int i = 0; i < N; ++i) 
		{
			max[i] = std::max(max[i], t[i]);
			min[i] = std::min(min[i], t[i]);
		}
		return (*this);
	}

	value_type center()
	{
		T t;
		for (int i = 0; i < N; ++i) t[i] = (max[i] + min[i]) / 2.0;


		return t;
	}

	container_type extent()
	{
		T e;
		for (int i = 0; i < N; ++i) e[i] = max[i] - min[i];
		return e;
	}

	value_type diagonal()
	{ 
				T d = extent();
		double diagonalSqr = 0.0;
		for (int i = 0; i < N; ++i) diagonalSqr += double(d[i]) * double(d[i]);
		return value_type(sqrt(diagonalSqr));
	}

	this_type operator*(float rhs)
	{
		this_type lhs = *this;
		for (int i = 0; i < N; ++i) lhs.max[i] *= rhs;
		for (int i = 0; i < N; ++i) lhs.min[i] *= rhs;

		return lhs;
	}

	this_type& operator*=(float rhs)
	{
		(*this) = (*this) * rhs;
		return (*this);
	}

	this_type operator/(float rhs)
	{
		this_type lhs = *this;
		for (int i = 0; i < N; ++i) lhs.max[i] /= rhs;
		for (int i = 0; i < N; ++i) lhs.min[i] /= rhs;

		return lhs;
	}

	this_type& operator/=(float rhs)
	{
		(*this) = (*this) / rhs;
		return (*this);
	}

};

template<int N, typename F>
using _bbox_vec = _bbox_base < N, F, glm::vec<N, F> >;

template<int N, typename F>
using _bbox_array = _bbox_base < N, F, std::array<F, N> >;

typedef _bbox_vec<int(4), float> bbox4;
typedef _bbox_vec<int(3), float> bbox3;
typedef _bbox_vec<int(2), float> bbox2;
