#ifndef _Geometry_SURFACELIGHT_H
#define _Geometry_SURFACELIGHT_H

#include <Geometry/Geometry.h>

namespace Geometry
{
	class SurfaceLight : public Geometry
	{
	public:

		SurfaceLight(Math::Vector3f position,double height=1,double witdh=1, Material * ematerial= new Material(0, 0, 0, 0, { 1,1,1 }))
			: Geometry()
		{
			int p0 = addVertex(Math::makeVector(0.5, 0.5, 0.0));
			int p1 = addVertex(Math::makeVector(0.5, -0.5, 0.0));
			int p2 = addVertex(Math::makeVector(-0.5, 0.5, 0.0));
			int p3 = addVertex(Math::makeVector(-0.5, -0.5, 0.0));
			addTriangle(p0, p1, p2, ematerial);
			addTriangle(p1, p2, p3, ematerial);
			this->scaleX(witdh);
			this->scaleY(height);
			this->translate(position);
		}
	};
}

#endif
