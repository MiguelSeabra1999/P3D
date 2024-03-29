#include "rayAccelerator.h"
#include "macros.h"
using namespace std;

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB& bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_) {
	this->leaf = true;
	this->index = index_; 
	this->n_objs = n_objs_; 
}

void BVH::BVHNode::makeNode(unsigned int left_index_) {
	this->leaf = false;
	this->index = left_index_; 
			//this->n_objs = n_objs_; 
}


BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }


void BVH::Build(vector<Object *> &objs) 
{
	BVHNode *root = new BVHNode();

	Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(FLT_MIN, FLT_MIN, FLT_MIN);
	AABB world_bbox = AABB(min, max);

	for (Object* obj : objs) {
		AABB bbox = obj->GetBoundingBox();
		world_bbox.extend(bbox);
		objects.push_back(obj);
	}
	world_bbox.min.x -= EPSILON; world_bbox.min.y -= EPSILON; world_bbox.min.z -= EPSILON;
	world_bbox.max.x += EPSILON; world_bbox.max.y += EPSILON; world_bbox.max.z += EPSILON;
	root->setAABB(world_bbox);
	nodes.push_back(root);
	build_recursive(0, objects.size(), root); // -> root node takes all the 
	int i = 0;
}

int BVH::GetLargestAxis(AABB aabb, float& midPoint, int left_index, int right_index) {
	float tx_min, tx_max, ty_min, ty_max, tz_min, tz_max;
	float axisSizeX, axisSizeY, axisSizeZ;
	Vector Objcentroid;

	tx_min = FLT_MAX;
	ty_min = FLT_MAX;
	tz_min = FLT_MAX;
	tx_max = -FLT_MAX;
	ty_max = -FLT_MAX;
	tz_max = -FLT_MAX;

	for (int i = left_index; i < right_index; i++) {
		Objcentroid = objects.at(i)->GetBoundingBox().centroid();

		if (Objcentroid.x < tx_min)
			tx_min = Objcentroid.x;
		if (Objcentroid.x > tx_max)
			tx_max = Objcentroid.x;
		if (Objcentroid.y < ty_min)
			ty_min = Objcentroid.y;
		if (Objcentroid.y > ty_max)
			ty_max = Objcentroid.y;
		if (Objcentroid.z < tz_min)
			tz_min = Objcentroid.z;
		if (Objcentroid.z > tz_max)
			tz_max = Objcentroid.z;
	}
	
	axisSizeX = tx_max - tx_min;
	axisSizeY = ty_max - ty_min;
	axisSizeZ = tz_max - tz_min;

	if (axisSizeX > axisSizeY && axisSizeX > axisSizeZ) {
		midPoint = axisSizeX / 2;
		return 0;
	} else if (axisSizeY > axisSizeZ) {
		midPoint = axisSizeY / 2;
		return 1;
	}
	else {
		midPoint = axisSizeZ / 2;
		return 2;
	}
	
}

void BVH::sortByAxis(int largestAxis, int left_index, int right_index) {
	Comparator comp;
	comp.dimension = largestAxis;
	std::sort(objects.begin() + left_index, objects.begin() + right_index, comp);
}

int BVH::getSplitIndex(float midPoint, int largestAxis, int left_index, int right_index) {
	for (int i  = left_index; i < right_index; i++) {
		if (objects.at(i)->GetBoundingBox().centroid().getAxisValue(largestAxis) > midPoint) {
			return i;
		}
	}
	return -1;
}

AABB BVH::GetNodeBB(int left_index, int right_index)
{
	Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(FLT_MIN, FLT_MIN, FLT_MIN);
	AABB node_bb = AABB(min, max);
	int start = left_index;
	int end = right_index;
	for (int i = start; i < right_index; i++) {
		AABB bb = objects.at(i)->GetBoundingBox();
		node_bb.extend(bb);
	}
	node_bb.min.x -= EPSILON; node_bb.min.y -= EPSILON; node_bb.min.z -= EPSILON;
	node_bb.max.x += EPSILON; node_bb.max.y += EPSILON; node_bb.max.z += EPSILON;

	return node_bb;
}

void BVH::build_recursive(int left_index, int right_index, BVHNode *node) {
	float midPoint = 0.0f;
	BVHNode* left = new BVHNode();
	BVHNode *right =  new BVHNode();
	int largestAxis, split_index;

	if (right_index - left_index <= Threshold) { //leaf node 
		node->makeLeaf(left_index, right_index - left_index);
		return;
	}
	else {
		largestAxis = GetLargestAxis(node->getAABB(), midPoint, left_index, right_index);
		sortByAxis(largestAxis, left_index, right_index);
		split_index = getSplitIndex(midPoint, largestAxis, left_index, right_index);
		if (split_index == left_index || split_index == right_index || split_index == -1) split_index = round((right_index + left_index) / 2); //make sure that neither left or right is completely empty

		left->setAABB(GetNodeBB(left_index, split_index));
		right->setAABB(GetNodeBB(split_index, right_index));

		node->makeNode(nodes.size());
		
		nodes.push_back(left);
		nodes.push_back(right);

		build_recursive(left_index, split_index, left);
		build_recursive(split_index, right_index, right);
		
	}

		//right_index, left_index and split_index refer to the indices in the objects vector
	   // do not confuse with left_nodde_index and right_node_index which refer to indices in the nodes vector. 
	    // node.index can have a index of objects vector or a index of nodes vector
			
}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {

	float t_closest = FLT_MAX;  //contains the closest primitive intersection
	bool aux = true;
	Object* closestHit = nullptr;
	ray.direction.normalize();
	Ray localRay = ray;
	BVHNode* currentNode = nodes[0];
	AABB closestBB;
	float t_left, t_right, t;

	bool left_hit, right_hit;
	int leftChild, rightChild;

	if (!nodes[0]->getAABB().intercepts(localRay, t))
	{

		return false;

	}
	while (true)
	{
		
		if (!currentNode->isLeaf()) {
			leftChild = currentNode->getIndex();
			rightChild = currentNode->getIndex() + 1;
			left_hit = nodes[leftChild]->getAABB().intercepts(localRay, t_left) ;
			right_hit =  nodes[rightChild]->getAABB().intercepts(localRay, t_right);

			
			if (left_hit && right_hit){

				int stackNode, nextNode;
				bool leftCloser = t_left < t_right;
				stackNode = leftCloser ? leftChild : rightChild;
				t = leftCloser ? t_left : t_right;
				nextNode = leftCloser ? rightChild : leftChild;

				currentNode = nodes[nextNode];
				StackItem stackitem = StackItem(nodes[stackNode], t);
				hit_stack.push(stackitem);
				continue;
			}
			else if (left_hit && !right_hit) {
				currentNode = nodes[leftChild];			
				continue;
			}
			else if (right_hit && !left_hit) {
				currentNode = nodes[rightChild];
				continue;
			}
		}
		else {  //isleaf
			for (int i = currentNode->getIndex(); i < currentNode->getIndex() + currentNode->getNObjs(); i++) {
				if (objects.at(i)->intercepts(localRay, t) && t < t_closest ) {
					t_closest = t;
					closestHit = objects.at(i);
					closestBB = currentNode->getAABB();
				}
			}
		}
		while (true) {
			if (hit_stack.empty()) {
				if (closestHit == nullptr)
					return false;
				else {
					*hit_obj = closestHit;
					hit_point = ray.origin + ray.direction * t_closest;
					return true;
				}
			}
			else {
				StackItem item = hit_stack.top();
				hit_stack.pop();
				if (item.t < t_closest || item.ptr->getAABB().intercepts( closestBB)) /*best  BB overlaps this new itm's BB*/ {
					currentNode = item.ptr;
					break;
				}
			}
				
		}
	}
}

bool BVH::Traverse(Ray& ray) {  //shadow ray

	double length = ray.direction.length(); //distance between light and intersection point
	ray.direction.normalize();

	Ray localRay = ray;
	BVHNode* currentNode = nodes[0];
	float t_left, t_right, t_closest, t;
	int leftChild, rightChild;
	bool left_hit, right_hit;

	

	if (!nodes[0]->getAABB().intercepts(localRay, t))
		return false;
	while (true)
	{
		if (!currentNode->isLeaf()) {
			leftChild = currentNode->getIndex();
			rightChild = currentNode->getIndex() + 1;
			left_hit = nodes[leftChild]->getAABB().intercepts(localRay, t_left);
			right_hit =  nodes[rightChild]->getAABB().intercepts(localRay, t_right);


			if (left_hit && right_hit) {
				currentNode = nodes[leftChild];
				StackItem stackitem = StackItem(nodes[rightChild], t);
				hit_stack.push(stackitem);
				continue;
			}
			else if (left_hit && !right_hit) {
				currentNode = nodes[leftChild];
				continue;
			}
			else if (right_hit && !left_hit) {
				currentNode = nodes[rightChild];
				continue;
			}
		}
		else {  //isleaf
			for (int i = currentNode->getIndex(); i < currentNode->getIndex() + currentNode->getNObjs(); i++) {
				if (objects.at(i)->intercepts(localRay, t) && t < length) {
					while (!hit_stack.empty()) {
						hit_stack.pop();
					}
					return true;
				}
			}
		}
		while (true) {
			if (hit_stack.empty()) {
				return false;
			}
			else {
				StackItem item = hit_stack.top();
				hit_stack.pop();
				currentNode = item.ptr;
				break;
			}
		}

	}
	}		
