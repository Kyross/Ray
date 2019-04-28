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
		//::std::vector<Triangle> allTriangles; //Faire vector de const Triangle * 
		::std::vector<const Triangle * > allTriangles;
		double currentSum;
		Math::Vector3f m_position;
		RGBColor m_color;
		::std::vector< std::pair< std::pair<double, double>, std::pair<double, double> > > m_computedIntervals; //Intervalles pour la stratification
		int m_compteurStratif;
		int m_lightSamples;

	public:
		/// <summary>
		/// Constructor
		/// </summary>
		LightSource(Math::Vector3f position, int lightSamples, Material * ematerial)
			: m_position(position),currentSum(0.0), m_compteurStratif(0), m_lightSamples(lightSamples), Geometry()
		{
			m_color = ematerial->getEmissive();
			//Creation des intervalles pour la stratification, de la forme paire( paire(a,b) , paire(c,d) ) 
			double interval = 1.0 / sqrt(double(m_lightSamples));
			for (double i = 0.0; i < 1.0; i += interval) {

				std::pair<double, double > interval1(i, i + interval);

				for (double j = 0.0; j <= 1.0-interval; j += interval) {

					std::pair<double, double > interval2(j, j + interval);

					std::pair< std::pair<double, double>, std::pair<double, double> > currentInterval(interval1, interval2);
					m_computedIntervals.push_back(currentInterval);
					
				}
			}	
		}
		

		/// <summary>
		/// Genates a point light by sampling the kept triangles.
		/// </summary>
		/// <returns></returns>
		virtual PointLight generate() = 0 {}

		//Echantillonage a graine unique
		void setSeed(int newSeed) {
			randomGenerator.seed(newSeed);
			std::cout << "seed set to : " << newSeed << std::endl;
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////
		/// \fn	const Math::Vector3 & PointLight::position() const
		///
		/// \brief	Gets the position of the light.
		///
		/// \author	F. Lamarche, Universit� de Rennes 1
		/// \date	04/12/2013
		///
		/// \return	.
		////////////////////////////////////////////////////////////////////////////////////////////////////
		const Math::Vector3f & position() const
		{
			return m_position;
		}

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
				allTriangles.push_back(&triangle);
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