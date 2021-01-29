/* \author Aaron Brown */
// Quiz on implementing kd tree
#ifndef kdtree3D_h
#define kdtree3D_h

#include <iostream> 
#include <string>  
#include <vector>

// Structure to represent node of kd tree
template<typename PointT>
struct PtNode
{
	PointT point;
	int id;
	PtNode<PointT>* left;
	PtNode<PointT>* right;
	// func to create a new point node
	PtNode<PointT>(PointT arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

template <typename PointT>
struct KdTree
{
	PtNode<PointT>* root;

	KdTree()
	: root(NULL)
	{}
	
	// 1. build tree - double pointer method
	void insertHelper1(PtNode<PointT>** node,  int depth, PointT point, int id)
	{
		if (*node == NULL) // dereference the double pointer to see node
		{
			*node = new PtNode<PointT>(point, id); // assign new members
		}
		else 
		{
			// cd either is 0,1 or 3, nums tells if compairing x, y or z values
			uint cd = depth % 3;
 
			if ((cd == 0 && point.x < (*node)->point.x) || 
			    (cd == 1 && point.y < (*node)->point.y) || 
			    (cd == 2 && point.z < (*node)->point.z))
				insertHelper1(&((*node)->left), depth+1, point, id);
			else
				insertHelper1(&((*node)->right), depth+1, point, id);	
		}
	}
	 
	// 2. build tree - pointer reference method
	void insertHelper2(PtNode<PointT> *&node,  int depth, PointT point, int id)
	{
		// node inside recursion is an alias of the node outside recursion. 
		// So if we change any of these two to hold some other address, the other pointer will also change. 
		if (node == NULL) 
		{
			node = new PtNode<PointT>(point, id); // assign new members
		}
		else 
		{
			// cd either is 0,1 or 3, nums tells if compairing x, y or z values
			uint cd = depth % 3;

 			if ((cd == 0 && point.x < node->point.x) || 
			    (cd == 1 && point.y < node->point.y) || 
			    (cd == 2 && point.z < node->point.z))
				insertHelper2(node->left, depth+1, point, id);
			else
				insertHelper2(node->right, depth+1, point, id);	
		}
	}
	
		
	
	void insert(PointT point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root
 
		// insertHelper1(&root, 0, point, id); // pass in the memo address of root (a pointer to root)
		 insertHelper2(root, 0, point, id);// pass in pointer reference

	}

	
	void searchHelper(PointT target, PtNode<PointT> * node, int depth, float distanceTol, std::vector<int> &ids)
	{
		if (node == NULL)
			return;

		// check IF within the cube
		if ((node->point.x >= target.x - distanceTol) && (node->point.x <= target.x + distanceTol) &&
		    (node->point.y >= target.y - distanceTol) && (node->point.y <= target.y + distanceTol) &&
		    (node->point.z >= target.z - distanceTol) && (node->point.z <= target.z + distanceTol))
		{
			// check IF within the sphere
			float dis = (node->point.x - target.x) * (node->point.x - target.x) + 
				    (node->point.y - target.y) * (node->point.y - target.y) +
				    (node->point.z - target.z) * (node->point.z - target.z);
			if (dis <= distanceTol * distanceTol)
				ids.push_back(node->id);
		}
		
		int cd = depth % 3; //cd either is 0 or 1 or 2 (x,y or z)

		// IF within left boundary
		if ((cd == 0 && node->point.x > target.x - distanceTol) || 
		    (cd == 1 && node->point.y > target.y - distanceTol) ||
	 	    (cd == 2 && node->point.z > target.z - distanceTol))
			searchHelper(target, node->left, depth+1, distanceTol, ids);

		// IF within right boundary
		if ((cd == 0 && node->point.x < target.x + distanceTol)	||
		    (cd == 1 && node->point.y < target.y + distanceTol) ||
		    (cd == 2 && node->point.z < target.z + distanceTol))
			searchHelper(target, node->right, depth+1, distanceTol, ids);		
	}	

	// return a list of point's ids in the tree that are within distance of target point
	std::vector<int> search(PointT target, float distanceTol)
	{
		std::vector<int> ids;
		
		searchHelper(target, root, 0, distanceTol, ids); 

		return ids;
	}
	
	
};

#endif 


