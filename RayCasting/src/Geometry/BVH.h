#ifndef _Geometry_BVH_H
#define _Geometry_BVH_H

#include <Geometry/Geometry.h>
#include <Geometry/Scene.h>
#include <iostream>
namespace Geometry {

	class BVH : public Geometry
	{
	protected:
		class BVHNode {
		public:
			BoundingBox m_boundingVolume;
			std::deque<const Triangle*>  m_primitives;
			BVHNode * m_filsGauche;
			BVHNode * m_filsDroit;

			BVHNode(std::deque<const Triangle*>  &primitives, BVHNode * filsGauche = nullptr, BVHNode * filsDroit = nullptr)
				:m_primitives(primitives), m_filsGauche(filsGauche), m_filsDroit(filsDroit), m_boundingVolume(){
	
				for (const Triangle  *t : m_primitives) {
					//create boundinx box arounf the triangle(s)
					m_boundingVolume.update(*(t));
				}
				//eviter les erreurs affichage de plan unique
				m_boundingVolume.bump(pow(10, -11));
			}

			virtual ~BVHNode()
			{
				delete m_filsGauche;
				delete m_filsDroit;
			}

			bool isLeaf() {
				return m_filsGauche == nullptr && m_filsDroit == nullptr;
			}

		};
		
		void sortTriangleList(::std::deque <const Triangle*> &list,int axis) {
			std::sort(list.begin(), list.end(), [axis](const auto& t1, const auto& t2) {
				return t1->center()[axis] < t2->center()[axis];
			});
		}
		


	//attributs de BVH.h
	BVHNode *m_root;

	public:
		BVH(std::deque<std::pair < BoundingBox, Geometry>>&geometries, BoundingBox &sceneBoundingBox) {
			//Initialisation de l'arbre depuis la scene
			::std::deque <const Triangle*> geometrieslist;
			//on recuperes l'ensemble des triangles de la scene
			for (::std::pair < BoundingBox, Geometry > &p : geometries) {
				for (const Triangle &t : p.second.getTriangles()) {
					geometrieslist.push_back(&t);
				}
			}
		
			m_root = new BVHNode(geometrieslist); 

			computeBVH(m_root);
		}

		virtual ~BVH()
		{
			delete m_root;
		}

		void path(CastedRay &cray) {
			double t0 = 0.0;
			double t1 = 100000.0;
			double entry, exit;
			//box de la scene intersecte par le rayon
			if (m_root->m_boundingVolume.intersect(cray, t0, t1, entry, exit)) {
				//on parcours recusirvement les box jusqu'a buff sur le triangle le plus proche
					checkNode(m_root, cray, entry, exit);	
			}
		}

	protected:
		void checkNode(BVHNode *current, CastedRay &cray, double t0, double t1) {
			double  l_entry, l_exit, r_entry, r_exit;
			if (current->isLeaf()) {
				for (const Triangle * t : current->m_primitives) {
						cray.intersect(t);
				}
			}
			else {
				bool isIntersectFilsGauche = current->m_filsGauche->m_boundingVolume.intersect(cray, t0, t1, l_entry, l_exit);
				bool isIntersectFilsDroit = current->m_filsDroit->m_boundingVolume.intersect(cray, t0, t1, r_entry, r_exit);

				if (!isIntersectFilsGauche && !isIntersectFilsDroit) {}
				else if (isIntersectFilsGauche && !isIntersectFilsDroit)
					checkNode(current->m_filsGauche, cray, l_entry, l_exit);
				else if (isIntersectFilsDroit && !isIntersectFilsGauche)
					checkNode(current->m_filsDroit, cray, r_entry, r_exit);
				else if (l_entry < r_entry)
				{
					checkNode(current->m_filsGauche, cray, l_entry, l_exit);

					if (!cray.validIntersectionFound())
					{
						checkNode(current->m_filsDroit, cray, r_entry, r_exit);
					}
					else {
						Math::Vector3f ti = cray.intersectionFound().intersection() - cray.source();
						if (ti.norm() > r_entry) {
							checkNode(current->m_filsDroit, cray, r_entry, ti.norm());
						}
					}
				}
				else
				{
					checkNode(current->m_filsDroit, cray, r_entry, r_exit);

					if (!cray.validIntersectionFound())
					{
						checkNode(current->m_filsGauche, cray, l_entry, l_exit);
					}
					else {
						Math::Vector3f ti = cray.intersectionFound().intersection() - cray.source();
						if (ti.norm() > l_entry) {
							checkNode(current->m_filsGauche, cray, l_entry, ti.norm());
						}
					}
				}
			}
		}

		void computeBVH(BVHNode * current) {
			::std::deque <const Triangle*> left_list, right_list, leftListMin, rightListMin;
			double ct = 1;
			double ci = 0.5;
			double cmin = 100.0;

			for (int axis = 0; axis < 3; axis++) {
				left_list.clear();
				right_list.clear();

				sortTriangleList(current->m_primitives,axis);

				unsigned int compteur_triangles = 0;			
				for (const Triangle *t : current->m_primitives) {
					if (compteur_triangles < current->m_primitives.size() / 2) {
						left_list.push_back(t);
					}
					else {
						right_list.push_back(t);
					}
					compteur_triangles++;
				}
				BVHNode filsGaucheTemp = BVHNode(left_list);
				BVHNode filsDroitTemp = BVHNode(right_list);

				double Sb = computeSurface(current->m_boundingVolume);
				double SbL = computeSurface(filsGaucheTemp.m_boundingVolume);
				double SbR = computeSurface(filsDroitTemp.m_boundingVolume);

				double c = ct + (SbL / Sb) * ci + (SbR / Sb) * ci;

				if (c >= current->m_primitives.size()*ci) {
					return;
				}

				if (c < cmin) {
					cmin = c;
					leftListMin = left_list;
					rightListMin = right_list;
				}
					
			}
			
				current->m_filsGauche = new BVHNode(leftListMin);
				current->m_filsDroit = new BVHNode(rightListMin);
				computeBVH(current->m_filsGauche);
				computeBVH(current->m_filsDroit);
		}

		double computeSurface(BoundingBox bbox) {
			double dx = bbox.max()[0] - bbox.min()[0];
			double dy = bbox.max()[1] - bbox.min()[1];
			double dz = bbox.max()[2] - bbox.min()[2];
		
			return 2.0 * (dx * dy + dx * dz + dy * dz);
		}
	};
}
#endif