#include <queue>
#include <iostream>
#include <algorithm>
#include <math.h>
#include <cstdlib>

#include "a_star2d.h"

namespace game_engine {
  // Anonymous namespace. 
  namespace {
    // Helper struct that functions as a linked list with data. The linked
    // list represents a path. Data members are a node, a cost to reach that
    // node, and a heuristic cost from the current node to the destination.
    struct NodeWrapper {
      std::shared_ptr<struct NodeWrapper> parent;
      std::shared_ptr<Node2D> node_ptr;

      // True cost to this node
      double cost;

      // Heuristic to end node
      double heuristic;

      // Equality operator
      bool operator==(const NodeWrapper& other) const {
        return *(this->node_ptr) == *(other.node_ptr);
      }
    };

    using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;
    bool is_present(const NodeWrapperPtr nwPtr,
		    const std::vector<NodeWrapperPtr>& nwPtrVec) {
      for (auto n: nwPtrVec) {
	if (*n == *nwPtr) {
	  return true;
	}
      }
      return false;
    }

    // Helper function. Compares the values of two NodeWrapper pointers.
    // Necessary for the priority queue.
    bool NodeWrapperPtrCompare(
        const std::shared_ptr<NodeWrapper>& lhs, 
        const std::shared_ptr<NodeWrapper>& rhs) {
      return lhs->cost + lhs->heuristic > rhs->cost + rhs->heuristic;
    }

    // HEURISTIC FUNCTION
    double Heuristic(
        const std::shared_ptr<Node2D>& current_ptr,
        const std::shared_ptr<Node2D>& end_ptr) {
      double deltax = end_ptr->Data().x() - current_ptr->Data().x();
      double deltay = end_ptr->Data().y() - current_ptr->Data().y();
      float euclidean = pow(((deltax * deltax) + (deltay * deltay)), 0.5);
      return euclidean;
    }

    // OVERESTIMATE
    /*
    double Heuristic(
		     const std::shared_ptr<Node2D>& current_ptr,
		     const std::shared_ptr<Node2D>& end_ptr) {
      double deltax = end_ptr->Data().x() - current_ptr->Data().x();
      double deltay = end_ptr->Data().y() - current_ptr->Data().y();
      float manhattan = abs(deltax) + abs(deltay);
      return manhattan;
    }
    */
    
    // DISTINCT FUNCTION
    /*
    double Heuristic(
		     const std::shared_ptr<Node2D>& current_ptr,
		     const std::shared_ptr<Node2D>& end_ptr) {
      double deltax = end_ptr->Data().x() - current_ptr->Data().x();
      double deltay = end_ptr->Data().y() - current_ptr->Data().y();
      float euclidean = pow(((deltax * deltax) + (deltay * deltay)), 0.5);
      float distinct = euclidean/2;
      return distinct;
    }
    */

    // DISTINCT FUNCTION 2
    /*
    double Heuristic(
		     const std::shared_ptr<Node2D>& current_ptr,
		     const std::shared_ptr<Node2D>& end_ptr) {
      double deltax = end_ptr->Data().x() - current_ptr->Data().x();
      double deltay = end_ptr->Data().y() - current_ptr->Data().y();
      float manhattan = abs(deltax) + abs(deltay);
      float distinct = manhattan*2;
      return distinct;
    }
    */
    
    // ZERO FUNCTION
    /*
    double Heuristic(
		     const std::shared_ptr<Node2D>& current_ptr,
		     const std::shared_ptr<Node2D>& end_ptr) {
      return 0;
    }
    */
  }

  PathInfo AStar2D::Run(
      const Graph2D& graph, 
      const std::shared_ptr<Node2D> start_ptr, 
      const std::shared_ptr<Node2D> end_ptr) {
    using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;

    // SETUP
    Timer timer;
    timer.Start();

    // Use these data structures
    std::priority_queue<
      NodeWrapperPtr,
      std::vector<NodeWrapperPtr>,
      std::function<bool(
          const NodeWrapperPtr&, 
          const NodeWrapperPtr& )>> 
      to_explore(NodeWrapperPtrCompare); // Priority queue
    std::vector<NodeWrapperPtr> explored; // Explored vector
    PathInfo path_info; // Output structure

    // Create a NodeWrapperPtr
    NodeWrapperPtr nw_ptr = std::make_shared<NodeWrapper>();
    nw_ptr->parent = nullptr;
    nw_ptr->node_ptr = start_ptr;
    nw_ptr->cost = 0;
    nw_ptr->heuristic = Heuristic(start_ptr, end_ptr);
    to_explore.push(nw_ptr);

    /////////////// Edits Begin ////////////////////

    // Check if the exploration pointer is empty
    while (!to_explore.empty()) {
      
      // Find the item to explore using first value of to_explore and remove
      NodeWrapperPtr item_to_explore = to_explore.top(); 
      to_explore.pop();

      // Check if item_to_explore has already been explored    
      if (is_present(item_to_explore, explored)) {
	continue;
      }

      // Add node to explored vector
      explored.push_back(item_to_explore);
      
      // Check if agent reached the end node
      if (*item_to_explore->node_ptr == *end_ptr) {

	// Stop timer
	path_info.details.run_time = timer.Stop();

	// Set up PathInfo
	path_info.path = {};
	path_info.details.path_cost = item_to_explore->cost;
	path_info.details.num_nodes_explored = explored.size();
	path_info.path.push_back(item_to_explore->node_ptr);
	path_info.details.path_length = 1;
	
	// Creating output path list 
	while ((*item_to_explore->node_ptr != *start_ptr)) {
	  // Push current node then update to parent
	  item_to_explore = item_to_explore->parent;
	  path_info.path.push_back(item_to_explore->node_ptr);
	  path_info.details.path_length += 1;
	}    
  
	// You must return a reversed PathInfo
	std::reverse(path_info.path.begin(), path_info.path.end());
	path_info.details.path_length = path_info.path.size();

	// Exit while loop since path is found
	break;
      }

      // If not the end then find next path
      
      // Use graph to find the neighbor nodes of the node_to_explore 
      auto edges = graph.Edges(item_to_explore->node_ptr);
	
      // Push all neighbors nodes to to_explore
      for(auto edge : edges) {

	// Create a NodeWrapperPtr for each neighbor node
	NodeWrapperPtr neighbor_ptr = std::make_shared<NodeWrapper>();

	// Update parent node to exploring node
	neighbor_ptr->parent = item_to_explore;

	// Set neighbor node pointer coordinates
	neighbor_ptr->node_ptr = edge.Sink();

	// Update heuristic value
	neighbor_ptr->heuristic = Heuristic(neighbor_ptr->node_ptr, end_ptr);

	// Set cost of neighbor exploration
	neighbor_ptr->cost = item_to_explore->cost + edge.Cost();
	
	// Push neighbor instance to the to_explore stack for next iteration
	to_explore.push(neighbor_ptr);	
      }
    }

    return path_info;
    //////////////Edits End///////////////
    
  }
}
