/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}
    void insertHelper(Node *&node, std::vector<float> point, int id, int depth)
    {
        if(node == NULL)
        {
            node = new Node(point, id);
        }
        else if(point[depth % 2] < node->point[depth % 2])
        {
            depth++;
            insertHelper(node->left, point, id, depth);
        }
        else
        {
            depth++;
            insertHelper(node->right, point, id, depth);
        }
    }

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
        insertHelper(root, point, id, 0);

	}
    void searchHelper(Node *node, std::vector<float> target, std::vector<int>& ids, int depth, float distanceTol)
    {
        if(node)
        {
            // If close
            if(fabs(target[depth % 2] - node->point[depth % 2]) < distanceTol)
            {
                // Check if point is in circle and add it
                if(fabs(target[(depth+1)% 2] - node->point[(depth+1) % 2]) < distanceTol)
                {
                    float dist_sq = powf((target[0]-node->point[0]),2)+powf((target[1]-node->point[1]),2);
                    if(dist_sq < distanceTol*distanceTol)
                    {
                        ids.push_back(node->id);
                    }
                }
                // In this case also go down both branches
                searchHelper(node->left, target, ids, depth+1, distanceTol);
                searchHelper(node->right, target, ids, depth+1, distanceTol);
            }
            // If not close than only go down on one of the two branches
            else
            {
                if(target[depth % 2] < node->point[depth % 2])
                {
                    searchHelper(node->left, target, ids, depth+1, distanceTol);
                }
                else
                {
                    searchHelper(node->right, target, ids, depth+1, distanceTol);
                }
            }
        }
    }

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
        searchHelper(root, target, ids, 0, distanceTol);
		return ids;
	}
	

};




