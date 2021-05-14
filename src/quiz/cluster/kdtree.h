/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node *left;
	Node *right;

	Node(std::vector<float> arr, int setId)
		: point(arr), id(setId), left(nullptr), right(nullptr)
	{
	}
};

struct KdTree
{
	Node *root;

	KdTree()
		: root(nullptr)
	{
	}

	static int incrementIndexToCheck(int indexToCheck)
	{
		return ((indexToCheck + 1) % 2);
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
		Node **nodeIndexToInsert = findInsertionPoint(point, &root);

		*nodeIndexToInsert = new Node(point, id);
	}

	Node **findInsertionPoint(std::vector<float> &pointToCheck, Node **root, int indexToCheck = 0)
	{
		if (*root == nullptr)
		{
			return root;
		}
		else if (pointToCheck[indexToCheck] < (*root)->point[indexToCheck])
		{
			return findInsertionPoint(pointToCheck, &((*root)->left), incrementIndexToCheck(indexToCheck));
		}
		else
		{
			return findInsertionPoint(pointToCheck, &((*root)->right), incrementIndexToCheck(indexToCheck));
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		return ids;
	}
};
