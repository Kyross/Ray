#ifndef _GEOMETRY_RECTANGLELIGHTSOURCE_H

#include "LightSource.h"
#include <random>
#include <chrono>
namespace Geometry
{
	class RectangleLightSource : public LightSource
	{
	protected:
		double m_witdh;
		double m_height;
	public:
		RectangleLightSource::RectangleLightSource(double height, double width, Math::Vector3f const & position = Math::makeVector(0.0, 0.0, 0.0), RGBColor const & color = RGBColor())
			: m_height(height), m_witdh(width), LightSource(position, color)
		{
		}
		virtual ~RectangleLightSource();

		PointLight & samplePointLight() {
			//P=C+random1*A+random2*B --sample a rectangle
			unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
			std::default_random_engine generator(seed);
			std::uniform_real_distribution<double> distribution(0.0, 1.0);
			double random1 = distribution(generator);
			double random2 = distribution(generator);

			PointLight p(Math::makeVector(0.0f, 0.f, 2.0f), color());

		}
	};
}
#endif // !_GEOMETRY_RECTANGLELIGHTSOURCE_H
