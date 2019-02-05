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
					//on englobe les triangles
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
				return m_filsDroit == nullptr && m_filsDroit == nullptr;
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
			for (::std::pair < BoundingBox, Geometry > &p : geometries) { //OK
				for (const Triangle &t : p.second.getTriangles()) {
					geometrieslist.push_back(&t);
				}
			}
		
			m_root = new BVHNode(geometrieslist);
			m_root->m_boundingVolume = sceneBoundingBox;

			computeBVH(m_root, 2);
			
			std::cout << "Build BVH : success" << std::endl;
		}

		virtual ~BVH()
		{
			delete m_root;
		}

		void path(CastedRay &cray, double t0 = 0.0, double t1 = 100000.0) {
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
				bool l = current->m_filsGauche->m_boundingVolume.intersect(cray, t0, t1, l_entry, l_exit);
				bool r = current->m_filsDroit->m_boundingVolume.intersect(cray, t0, t1, r_entry, r_exit);

				if (!l && !r) {}
				else if ((l && !r) || (!l && r)) {
					if (l) checkNode(current->m_filsGauche, cray, l_entry, l_exit);
					if (r) checkNode(current->m_filsDroit, cray, r_entry, r_exit);
				}
				else {

					BVHNode * filsGauche_temp = current->m_filsGauche;
					BVHNode * filsDroit_temp = current->m_filsDroit;
					if (!(l_entry < r_entry)) {
						filsGauche_temp = current->m_filsGauche;
						filsDroit_temp = current->m_filsDroit;
					}

					checkNode(filsGauche_temp, cray, l_entry, l_exit);


					if (!cray.validIntersectionFound()) {
						checkNode(filsDroit_temp, cray, r_entry, r_exit);
					}
					else {
						Math::Vector3f ti = cray.intersectionFound().intersection() - cray.source();
						if (ti.norm() > r_entry) {
							checkNode(filsDroit_temp, cray, r_entry, ti.norm());
						}
					}
				}
			}
		}

		void computeBVH(BVHNode * current, unsigned int threshold, int compteur = 0) {
			//changement d'axe pour le trie (pas opti)
			sortTriangleList(current->m_primitives, compteur % 2);
			
			//On construit une feuille si le nombre de triangles est inferieur a un seuil
			if (current->m_primitives.size() < threshold) {
				//std::cout << "Feuille cree, profondeur : " << compteur << std::endl;
				return;
			}
			else {
				//Sinon, separation des triangles puis appel recursif
				unsigned int compteur_triangles = 0;
				::std::deque <const Triangle*> left_list,right_list;
				for (const Triangle  *t : current->m_primitives) {
					if (compteur_triangles < current->m_primitives.size() / 2) {
						left_list.push_back(t);
					}
					else {
						right_list.push_back(t);
					}
					compteur_triangles++;
				}
				current->m_filsGauche = new BVHNode(left_list);
				current->m_filsDroit = new BVHNode(right_list);
				computeBVH(current->m_filsGauche, threshold, compteur + 1);
				computeBVH(current->m_filsDroit, threshold, compteur + 1);
			}
		}


		
	};
}
#endif