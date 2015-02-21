#ifndef OctreePoint_H
#define OctreePoint_H

#include <Eigen/Core>

// Simple point data type to insert into the tree.
// Have something with more interesting behavior inherit
// from this in order to store other attributes in the tree.
class OctreePoint {
	Eigen::Vector3f position;
public:
	OctreePoint() { }
	OctreePoint(const Eigen::Vector3f& position) : position(position) { }
	inline const Eigen::Vector3f& getPosition() const { return position; }
	inline void setPosition(const Eigen::Vector3f& p) { position = p; }
};

#endif
