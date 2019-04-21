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
		RGBColor m_color;
	public:

		LightSphere(Math::Vector3f position, double radius = 1, int nbDiv = 10, Material * ematerial = nullptr)
			: LightSource(position), m_radius(radius)
		{
			
			ematerial = new Material(0, 0, 0, 0, { 1,1,1 });

			m_color = ematerial->getEmissive();
			/*
			unsigned int center = addVertex(Math::Vector3f());
			::std::vector<unsigned int> vertices;
			for (int cpt = 0; cpt < nbDiv; cpt++)
			{
				double angle = double((2.0f*M_PI / nbDiv)*cpt);
				int i = addVertex(Math::makeVector(m_radius * cos(angle), m_radius * sin(angle), 0.0));
				vertices.push_back(i);
			}
			for (int cpt = 0; cpt < nbDiv; cpt++)
			{
				addTriangle(center, vertices[cpt], vertices[(cpt + 1) % nbDiv], ematerial);
			}
			this->translate(m_position);

			auto & triangles = getTriangles();
			add(triangles.begin(), triangles.end());
			*/
		}

		// Hérité via SourceLight
		std::pair<PointLight, const Triangle * > generate() const
		{
			double xi1 = double(randomGenerator()) / double(randomGenerator.max());
			double xi2 = double(randomGenerator()) / double(randomGenerator.max());

			double theta = acos(sqrt(xi1));
			double phi = 2 * M_PI * xi2;

			double x = m_radius * sin(phi) * cos(theta);
			double y = m_radius * sin(phi) * sin(theta);
			double z = m_radius * cos(phi);

			Math::Vector3f pos = Math::makeVector(x + m_position[0], y + m_position[1], z + m_position[2]);

			PointLight light(pos, m_color);


			std::pair<PointLight, const Triangle * > res(light, nullptr);
			return res;

		}

	};
}
#endif