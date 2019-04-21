#ifndef _Geometry_LightSurface_H
#define _Geometry_LightSurface_H

#include <Geometry/LightSource.h>

namespace Geometry
{
	class LightSurface : public LightSource
	{
	public:

		LightSurface(Math::Vector3f position, double height = 1, double witdh = 1, Material * ematerial = nullptr)
			: LightSource(position)
		{
			ematerial = new Material(0, 0, 0, 0, { 1,1,1 });
			int p0 = addVertex(Math::makeVector(0.5, 0.5, 0.0));
			int p1 = addVertex(Math::makeVector(0.5, -0.5, 0.0));
			int p2 = addVertex(Math::makeVector(-0.5, 0.5, 0.0));
			int p3 = addVertex(Math::makeVector(-0.5, -0.5, 0.0));
			addTriangle(p0, p1, p2, ematerial);
			addTriangle(p1, p2, p3, ematerial);
			this->scaleX(witdh);
			this->scaleY(height);
			this->translate(m_position);

			auto & triangles = getTriangles();
			add(triangles.begin(), triangles.end());

		}

		// Hérité via SourceLight
		std::pair<PointLight, const Triangle * > generate() const
		{
			double random = double(randomGenerator()) / double(randomGenerator.max());
			random = random * currentSum;
			auto found = std::lower_bound(surfaceSum.begin(), surfaceSum.end(), random);
			size_t index = found - surfaceSum.begin();
			Math::Vector3f barycentric = allTriangles[index]->randomBarycentric();
			Math::Vector3f point = allTriangles[index]->pointFromBraycentric(barycentric);
			RGBColor color = allTriangles[index]->sampleTexture(barycentric);

			const Triangle * toIgnore = allTriangles[index];
			PointLight light(point, allTriangles[index]->material()->getEmissive()*color);

			std::pair<PointLight, const Triangle * > res(light, toIgnore);
			return res;
		}
	};
}
#endif
