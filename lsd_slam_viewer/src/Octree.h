#define GL_GLEXT_PROTOTYPES 1

#ifndef Octree_H
#define Octree_H

#include <GL/glu.h>


#include <cstddef>
#include <vector>
#include <Eigen/Core>
#include "settings.h"

/* Derived from the Octree class as coded by Brandon Pelfrey.
 *
 */
class Octree {
	Eigen::Vector3f origin;         //! The physical center of this node
	Eigen::Vector3f halfDimension;  //! Half the width/height/depth of this node
	unsigned int maxLeafPoints;
	float leafSideLength;
	Octree *children[8];

	public:
		std::vector<Eigen::Vector3f> data;

	public:
	Octree(const Eigen::Vector3f& origin, const Eigen::Vector3f& halfDimension, unsigned int maxLeafPoints)
		: origin(origin), halfDimension(halfDimension), maxLeafPoints(maxLeafPoints), leafSideLength(0) {

			for(int i=0; i<8; ++i)
				children[i] = NULL;
		}

	Octree(const Eigen::Vector3f& origin, const Eigen::Vector3f& halfDimension, float leafSideLength)
		: origin(origin), halfDimension(halfDimension), maxLeafPoints(0), leafSideLength(leafSideLength) {

			for(int i=0; i<8; ++i)
				children[i] = NULL;
		}

	Octree(const Octree& copy)
		: origin(copy.origin), halfDimension(copy.halfDimension), data(copy.data) {

		}

	~Octree() {
		for(int i = 0; i<8; i++) delete children[i];
	}

	int getOctantContainingPoint(const Eigen::Vector3f& point) const {
		int oct = 0;
		if( point[0] >= origin[0] ) oct |= 4;
		if( point[1] >= origin[1] ) oct |= 2;
		if( point[2] >= origin[2] ) oct |= 1;
		return oct;
	}

	bool isLeafNode() const {
		return children[0] == NULL;
	}

	void insert(const Eigen::Vector3f &point) {
		if(isLeafNode()) {
			if(data.size()<maxLeafPoints) {
				data.push_back(point);
				return;
			} else {
				for(int i=0; i<8; ++i) {
					Eigen::Vector3f newOrigin = origin;
					newOrigin[0] += halfDimension[0] * (i&4 ? .5f : -.5f);
					newOrigin[1] += halfDimension[1] * (i&2 ? .5f : -.5f);
					newOrigin[2] += halfDimension[2] * (i&1 ? .5f : -.5f);
					children[i] = new Octree(newOrigin, halfDimension*.5f, maxLeafPoints);
				}

				for ( auto &i : data ) {
					children[getOctantContainingPoint(i)]->insert(i);
				}

				//Free the memory.
				std::vector<Eigen::Vector3f>().swap(data);

				children[getOctantContainingPoint(point)]->insert(point);
			}
		} else {
			int octant = getOctantContainingPoint(point);
			children[octant]->insert(point);
		}
	}

	void insertSidelenghtBased(const Eigen::Vector3f &point) {
			if(isLeafNode()) {
				if(halfDimension.norm()<=leafSideLength) {
					data.push_back(point);
					return;
				} else {

					for(int i=0; i<8; ++i) {
						Eigen::Vector3f newOrigin = origin;
						newOrigin[0] += halfDimension[0] * (i&4 ? .5f : -.5f);
						newOrigin[1] += halfDimension[1] * (i&2 ? .5f : -.5f);
						newOrigin[2] += halfDimension[2] * (i&1 ? .5f : -.5f);
						children[i] = new Octree(newOrigin, halfDimension*.5f, leafSideLength);
					}

					for ( auto &i : data ) {
						children[getOctantContainingPoint(i)]->insertSidelenghtBased(i);
					}

					//Free the memory.
					std::vector<Eigen::Vector3f>().swap(data);

					children[getOctantContainingPoint(point)]->insertSidelenghtBased(point);
				}
			} else {
				int octant = getOctantContainingPoint(point);
				children[octant]->insertSidelenghtBased(point);
			}
		}

	Octree * getLeafContainingPoint(const Eigen::Vector3f &point){
		if(isLeafNode()){
			return this;
		} else {
			return children[getOctantContainingPoint(point)]->getLeafContainingPoint(point);
		}
	}

	void drawOctree(){
		glColor3f(1, 0, 0);

		glBegin(GL_LINES);

		float x1 = origin[0] + halfDimension[0];
		float x2 = origin[0] - halfDimension[0];
		float y1 = origin[1] + halfDimension[1];
		float y2 = origin[1] - halfDimension[1];
		float z1 = origin[2] + halfDimension[2];
		float z2 = origin[2] - halfDimension[2];

		glVertex3f(x1, y1, z1);
		glVertex3f(x1, y1, z2);

		glVertex3f(x1, y1, z1);
		glVertex3f(x2, y1, z1);

		glVertex3f(x1, y1, z1);
		glVertex3f(x1, y2, z1);

		glVertex3f(x1, y1, z2);
		glVertex3f(x1, y2, z2);

		glVertex3f(x1, y1, z2);
		glVertex3f(x2, y1, z2);

		glVertex3f(x2, y1, z1);
		glVertex3f(x2, y2, z1);

		glVertex3f(x2, y1, z1);
		glVertex3f(x2, y1, z2);

		glVertex3f(x1, y2, z1);
		glVertex3f(x1, y2, z2);

		glVertex3f(x1, y2, z1);
		glVertex3f(x2, y2, z1);

		glVertex3f(x2, y2, z2);
		glVertex3f(x1, y2, z2);

		glVertex3f(x2, y2, z2);
		glVertex3f(x2, y2, z1);

		glVertex3f(x2, y2, z2);
		glVertex3f(x2, y1, z2);

		glEnd();
		glColor3f(1, 1, 1);

		if(isLeafNode()){
			return;
		} else {
			for(int i = 0; i<8; i++){
				children[i]->drawOctree();
			}
		}
	}


};

#endif
