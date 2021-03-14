#ifndef KDTREE_HPP_
#define KDTREE_HPP_

#include <vector>

// --- Struct type to represent a node inside KdTree
struct Node {
    
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId) : point(arr), id(setId), left(NULL), right(NULL) {}

};

// --- Struct type and functions to build KdTree
struct KdTree {
    
    // Tree initialization with a root node
    Node* root;
    KdTree() : root(NULL) {}

    // Functions that insert leaves into the tree
    void insertHelper(Node** node, int depth, std::vector<float> point, int id) {

        if (*node == NULL) {
            *node = new Node(point, id);
        } else {
            int cd = depth % 3;
            
            if (point[cd] < (*node)->point[cd]) {
                insertHelper(&((*node)->left), depth + 1, point, id);
            } else {
                insertHelper(&((*node)->right), depth + 1, point, id);
            }

        }

    }

    void insert(std::vector<float> point, int id) {
        insertHelper(&root, 0, point, id);
    }

    // Functions that returns a list of points that are within distance of target
    void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int> &ids) {
        
        if (node != NULL) {

            float delta_x = node->point.x - target.x;
            float delta_y = node->point.y - target.y;
            float delta_z = node->point.z - target.z;

            if ((delta_x >= -distanceTol && delta_x <= distanceTol) &&
                (delta_y >= -distanceTol && delta_y <= distanceTol) &&
                (delta_z >= -distanceTol && delta_z <= distanceTol)) {
                float distance = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
                if (distance <= distanceTol) {
                    ids.push_back(node->id);
                }
            }

            // Check across boundary
            if ((target[depth % 2] - distanceTol) < node->point[depth % 2]) {
				searchHelper(target,node->left,depth+1,distanceTol,ids); 
			}
			if ((target[depth % 2] + distanceTol) > node->point[depth % 2]) {
				searchHelper(target, node->right, depth + 1, distanceTol, ids);
			}

        }
    }

    std::vector<int> search(std::vector<float> target, float distanceTol) {

        std::vector<int> ids;
        searchHelper(target, root, 0, distanceTol, ids);
        return ids;

    }

};

#endif