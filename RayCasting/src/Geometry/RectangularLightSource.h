#ifndef _GEOMETRY_RECTANGLELIGHTSOURCE_H

#include "LightSource.h"

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


		RectangleLightSource::~RectangleLightSource()
		{
		}
		virtual ~RectangleLightSource();
	};
}
#endif // !_GEOMETRY_RECTANGLELIGHTSOURCE_H
