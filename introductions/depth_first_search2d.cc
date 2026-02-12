#include <stack>
#include <iostream>
#include <algorithm>

#include "depth_first_search2d.h"

namespace game_engine {
  // Anonymous namespace. Put any file-local functions or variables in here
  namespace {
    // Helper struct that functions as a linked list with data. The linked
    // list represents a path. Data members are a node and a cost to reach
    // that node.
    struct NodeWrapper {
      std::shared_ptr<struct NodeWrapper> parent;
      std::shared_ptr<Node2D> node_ptr;
      double cost;

      // Equality operator
      bool operator==(const NodeWrapper& other) const {
        return *(this->node_ptr) == *(other.node_ptr);
      }
    };

    using NodeWrapperPtr =std::shared_ptr<NodeWrapper>;
    bool is_present(const NodeWrapperPtr nwPtr,
		    const std::vector<NodeWrapperPtr>& nwPtrVec) {
      for (auto n: nwPtrVec) {
	if (*n == *nwPtr) {
	  return true;
	}
      }
      return false;
    }
  }
  
  PathInfo DepthFirstSearch2D::Run(
      const Graph2D& graph, 
      const std::shared_ptr<Node2D> start_ptr, 
      const std::shared_ptr<Node2D> end_ptr) {
    using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;

    // SETUP
    Timer timer;
    timer.Start();
    
    // Use these data structures
    std::stack<NodeWrapperPtr> to_explore; // nodes to explore
    std::vector<NodeWrapperPtr> explored; // expored nodes
    PathInfo path_info; // Output structure
    
    // Create a NodeWrapperPtr
    NodeWrapperPtr nw_ptr = std::make_shared<NodeWrapper>();
    nw_ptr->parent = nullptr;
    nw_ptr->node_ptr = start_ptr;
    nw_ptr->cost = 0;
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

	// Update exploring node to parent node
	neighbor_ptr->parent = item_to_explore;

	// Set neighbor node pointer coordinates
	neighbor_ptr->node_ptr = edge.Sink();

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
