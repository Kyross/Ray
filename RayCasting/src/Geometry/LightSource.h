#ifndef _GEOMETRY_LIGHTSOURCE_H
#include <Geometry/Material.h>

namespace Geometry
{
	class LightSource
	{
	protected:
		/// \brief	The light position.
		Math::Vector3f m_position;

		/// \brief	The light color.
		RGBColor m_color;

	public:
		LightSource(Math::Vector3f const & position = Math::makeVector(0.0, 0.0, 0.0), RGBColor const & color = RGBColor())
			: m_position(position), m_color(color)
		{}


		virtual void samplePointLight() = 0;

		////////////////////////////////////////////////////////////////////////////////////////////////////
		/// \fn	const RGBColor & PointLight::color() const
		///
		/// \brief	Gets the light color.
		///
		/// \author	F. Lamarche, Université de Rennes 1
		/// \date	04/12/2013
		///
		/// \return	.
		////////////////////////////////////////////////////////////////////////////////////////////////////
		const RGBColor & color() const
		{
			return m_color;
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////
		/// \fn	const Math::Vector3 & PointLight::position() const
		///
		/// \brief	Gets the position of the light.
		///
		/// \author	F. Lamarche, Université de Rennes 1
		/// \date	04/12/2013
		///
		/// \return	.
		////////////////////////////////////////////////////////////////////////////////////////////////////
		const Math::Vector3f & position() const
		{
			return m_position;
		}
	};
}
#endif