/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

#include <queue>

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
	Node *root{nullptr};
	static const int startingIndex{0};

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

	Node **findInsertionPoint(std::vector<float> &pointToCheck, Node **root, int indexToCheck = startingIndex)
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
	bool isWithinDistance(std::vector<float> point1, std::vector<float> point2, float distanceTol, int index)
	{
		bool retVal{false};

		float x1 = point1.at(0);
		float y1 = point1.at(1);
		float x2 = point2.at(0);
		float y2 = point2.at(1);

		if (fabs(point1.at(index) - point2.at(index)) <= distanceTol)
		{
			auto dist = sqrtf(pow((x2 - x1), 2) + pow((y2 - y1), 2));
			retVal = (dist <= distanceTol);
		}
		return retVal;
	}

	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		std::queue<std::pair<Node *, int>> indicesToSearch;
		int searchIndex = startingIndex;
		indicesToSearch.emplace(root, searchIndex);

		while (!indicesToSearch.empty())
		{
			std::pair<Node *, int> entryToCheck = indicesToSearch.front();
			indicesToSearch.pop();
			Node *nodeToCheck = entryToCheck.first;
			int argToCheck = entryToCheck.second;

			if (nodeToCheck == nullptr)
			{
				continue;
			}

			if (isWithinDistance(target, nodeToCheck->point, distanceTol, argToCheck))
			{
				ids.push_back(nodeToCheck->id);
				indicesToSearch.emplace(nodeToCheck->left, incrementIndexToCheck(argToCheck));
				indicesToSearch.emplace(nodeToCheck->right, incrementIndexToCheck(argToCheck));
			}
			else
			{
				indicesToSearch.emplace(nodeToCheck->left, incrementIndexToCheck(argToCheck));
			}
		}
		return ids;
	}
};
