#ifndef _Geometry_LightSurface_H
#define _Geometry_LightSurface_H

#include <Geometry/LightSource.h>

namespace Geometry
{
	class LightSurface : public LightSource
	{
	public:

		LightSurface(Math::Vector3f position, Math::Quaternion<double> const & q, double height = 1, double width = 1, Material * ematerial = nullptr, int lightSamples = 1)
			: LightSource(position, lightSamples, ematerial)
		{
			int p0 = addVertex(Math::makeVector(0.5, 0.5, 0.0));
			int p1 = addVertex(Math::makeVector(0.5, -0.5, 0.0));
			int p2 = addVertex(Math::makeVector(-0.5, 0.5, 0.0));
			int p3 = addVertex(Math::makeVector(-0.5, -0.5, 0.0));
			addTriangle(p0, p1, p2, ematerial);
			addTriangle(p1, p2, p3, ematerial);
			this->scaleX(width);
			this->scaleY(height);
			this->rotate(q);
			this->translate(m_position);
			

			auto & triangles = getTriangles();
			add(triangles.begin(), triangles.end());
		}

		// Hérité via SourceLight
		PointLight generate()
		{
			double random = double(randomGenerator()) / double(randomGenerator.max());
			random = random * currentSum;
			auto found = std::lower_bound(surfaceSum.begin(), surfaceSum.end(), random);
			size_t index = found - surfaceSum.begin();
			Math::Vector3f barycentric = allTriangles[index]->randomBarycentric();
			Math::Vector3f point = allTriangles[index]->pointFromBraycentric(barycentric);
			RGBColor color = allTriangles[index]->sampleTexture(barycentric);

			PointLight light(point, allTriangles[index]->material()->getEmissive()*color);

			m_compteurStratif = (m_compteurStratif + 1) % m_lightSamples;
			return light;
		}
	};
}
#endif
