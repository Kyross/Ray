#ifndef _Geometry_LightSource_H
#define _Geometry_LightSource_H

#include <random>
#include <Geometry/Triangle.h>
#include <Geometry/Geometry.h>
#include <Geometry/PointLight.h>

namespace Geometry
{
	/// <summary>
	/// Abstract class
	/// </summary>
	class LightSource : public Geometry
	{
	protected:
		/// <summary> A random number generator. </summary>
		mutable std::mt19937_64 randomGenerator;
		::std::vector<double> surfaceSum;
		::std::vector<Triangle> allTriangles;
		double currentSum;

	public:
		/// <summary>
		/// Constructor
		/// </summary>
		LightSource()
			: currentSum(0.0), Geometry()
		{}


		/// <summary>
		/// Genates a point light by sampling the kept triangles.
		/// </summary>
		/// <returns></returns>
		virtual PointLight generate() const = 0 {}

		/// <summary>
		/// Adds a triangle to the light sampler.
		/// </summary>
		/// <param name="triangle"></param>
		void add(const Triangle & triangle)
		{
			if (!triangle.material()->getEmissive().isBlack())
			{
				currentSum += triangle.surface()/**it->material()->getEmissive().grey()*/;
				surfaceSum.push_back(currentSum);
				allTriangles.push_back(triangle);
			}
		}

		/// <summary>
		/// Adds a geometry to the light sampler.
		/// </summary>
		/// <param name="geometry"></param>
		void add(const Geometry & geometry)
		{
			auto & triangles = geometry.getTriangles();
			add(triangles.begin(), triangles.end());
		}

		/// <summary>
		/// Adds a range of triangles or geometries to the light sampler.
		/// </summary>
		template <class iterator>
		void add(iterator begin, iterator end)
		{
			for (iterator it = begin; it != end; ++it)
			{
				add(*it);
			}
		}

		/// <summary>
		/// Generates nb point lights by sampling the kept triangles.
		/// </summary>
		template <class iterator>
		iterator generate(iterator output, size_t nb)
		{
			for (size_t cpt = 0; cpt < nb; ++cpt)
			{
				(*output) = generate();
				++output;
			}
			return output;
		}

		/// <summary>
		/// Generates end-begin point lights by sampling the kept triangles.
		/// </summary>
		template <class iterator>
		iterator generate(iterator begin, iterator end)
		{
			for (; begin != end; ++begin)
			{
				(*begin) = generate();
				++begin;
			}
			return begin;
		}

		/// <summary>
		/// Returns true if the light sampler can sample lights.
		/// </summary>
		/// <returns></returns>
		bool hasLights() const
		{
			return allTriangles.size() > 0;
		}
	};
}
#endif