#ifndef _Geometry_LightSphere_H
#define _Geometry_LightSphere_H

#include <Geometry/LightSource.h>

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795
#endif

namespace Geometry
{
	class LightSphere : public LightSource
	{
	protected:
		double m_radius;
		//RGBColor m_color;
	public:

		LightSphere(Math::Vector3f position, double radius = 1, int nbDiv = 10, Material * ematerial = nullptr, int lightSamples = 1)
			: LightSource(position, lightSamples, ematerial), m_radius(radius)
		{

			unsigned int center = addVertex(Math::Vector3f());
			::std::vector<unsigned int> vertices;
			for (int cpt = 0; cpt < nbDiv; cpt++)
			{
				double angle = double((2.0f*M_PI / nbDiv)*cpt);
				int i = addVertex(Math::makeVector(0.1 * cos(angle), 0.1 * sin(angle), 0.0));
				vertices.push_back(i);
			}
			for (int cpt = 0; cpt < nbDiv; cpt++)
			{
				addTriangle(center, vertices[cpt], vertices[(cpt + 1) % nbDiv], ematerial);
			}
			this->translate(m_position);

			auto & triangles = getTriangles();
			add(triangles.begin(), triangles.end());
		}

		// Hérité via SourceLight
		PointLight generate()
		{
			double inf1 = m_computedIntervals[m_compteurStratif].first.first;
			double sup1 = m_computedIntervals[m_compteurStratif].first.second;
			double inf2 = m_computedIntervals[m_compteurStratif].second.first;
			double sup2 = m_computedIntervals[m_compteurStratif].second.second;

			double xi1 = Math::RandomDirection::random(inf1, sup1);
			double xi2 = Math::RandomDirection::random(inf2, sup2);

			double theta = acos(sqrt(xi1));
			double phi = 2 * M_PI * xi2;

			double x = m_radius * sin(phi) * cos(theta);
			double y = m_radius * sin(phi) * sin(theta);
			double z = m_radius * cos(phi);

			Math::Vector3f pos = Math::makeVector(x + m_position[0], y + m_position[1], z + m_position[2]);

			PointLight light(pos, m_color);

			m_compteurStratif = (m_compteurStratif + 1) % m_lightSamples;
			return light;

		}

	};
}
#endif