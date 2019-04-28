#ifndef _Geometry_LightRectangle_H
#define _Geometry_LightRectangle_H

#include <Geometry/LightSource.h>

namespace Geometry
{
	class LightRectangle : public LightSource
	{
	protected:
		double m_height;
		double m_width;
	public:

		LightRectangle(Math::Vector3f position, Math::Quaternion<double> const & q,double height = 1, double width = 1, Material * ematerial = nullptr, int lightSamples = 1)
			: LightSource(position, lightSamples, ematerial), m_height(height), m_width(width)
		{
			int p0 = addVertex(Math::makeVector(0.5, 0.5, 0.0)); //D
			int p1 = addVertex(Math::makeVector(0.5, -0.5, 0.0)); //C
			int p2 = addVertex(Math::makeVector(-0.5, 0.5, 0.0)); //B
			int p3 = addVertex(Math::makeVector(-0.5, -0.5, 0.0)); //A
			addTriangle(p0, p1, p2, ematerial);
			addTriangle(p1, p2, p3, ematerial);
			
			this->scaleX(width);
			this->scaleY(height);
			this->translate(m_position);
			this->rotate(q);

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

			//double xi1 = (inf1 + sup1) / 2.0;
			//double xi2 = (inf2 + sup2) / 2.0;

			Math::Vector3f A = Math::makeVector(- 0.5 + m_position[0], - 0.5 + m_position[1], m_position[2]);
			Math::Vector3f B = Math::makeVector(- 0.5 + m_position[0], 0.5 + m_position[1], m_position[2]);
			Math::Vector3f C = Math::makeVector(0.5 + m_position[0], 0.5 + m_position[1], m_position[2]);
			Math::Vector3f D = Math::makeVector(0.5 + m_position[0], - 0.5 + m_position[1], m_position[2]);
			Math::Vector3f AB = Math::makeVector(B[0] - A[0], B[1] - A[1], B[2] - A[2]);
			Math::Vector3f AD = Math::makeVector(D[0] - A[0], D[1] - A[1], D[2] - A[2]);

			Math::Vector3f pos = A + AB * xi1 + AD * xi2;
			PointLight light(pos, m_color);

			m_compteurStratif = (m_compteurStratif + 1) % m_lightSamples;
			return light;
		}
	};
}
#endif
