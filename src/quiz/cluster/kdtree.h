/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	std::unique_ptr<Node> left;
	std::unique_ptr<Node> right;

	Node(const std::vector<float> &arr, const int &setId) : point(arr), id(setId)
	{
		left.reset();
		right.reset();
	}

	Node(const Node &other) : point(other.point), id(other.id)
	{
		left.reset(other.left.get());
		right.reset(other.right.get());
	}
};

struct KdTree
{
	std::unique_ptr<Node> root;

	KdTree() { root.reset(); }

	void insert(const std::vector<float> &point, const int &id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		insert(root, Node(point, id), 0);
	}

	void insert(std::unique_ptr<Node> &parent, const Node & node, int depth)
	{
		if(!parent)
		{
			parent = std::make_unique<Node>(node);
		}
		else
		{
			const auto dx = depth % 2;
			if(node.point[dx] < parent->point[dx])
			{
				insert(parent->left, node, ++depth);
			}
			else
			{
				insert(parent->right, node, ++depth);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(const std::vector<float> &target, const float &distanceTol)
	{
		std::vector<int> ids;
		search(target, root, 0, distanceTol, ids);
		return ids;
	}

	void search(const std::vector<float> &target, const std::unique_ptr<Node> &node, const int &depth, const float &distanceTol, std::vector<int> &ids )
	{
		if(node)
		{
			if(node->point[0] >= target[0] - distanceTol && node->point[0] <= target[0] + distanceTol && node->point[1] >= target[1] - distanceTol && node->point[1] <= target[1] + distanceTol )
			{
				const auto distance = sqrt((node->point[0] - target[0]) * (node->point[0] - target[0]) + (node->point[1] - target[1]) * (node->point[1] - target[1]));
				if (distance <= distanceTol)
					ids.push_back(node->id);

				if(target[depth % 2] - distanceTol < node->point[depth % 2])
					search(target, node->left, depth + 1, distanceTol, ids);
				if(target[depth % 2] + distanceTol > node->point[depth % 2])
					search(target, node->right, depth + 1, distanceTol + 1, ids);
			}
		}
	}

};




