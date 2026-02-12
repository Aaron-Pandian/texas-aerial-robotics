#include <cstdlib>
#include <vector>
#include <fstream>
#include <string.h>

#include "a_star2d.h"
#include "occupancy_grid2d.h"
#include "path_info.h"
#include "polynomial_solver.h"
#include "polynomial_sampler.h"
#include "gnuplot-iostream.h"
#include "gui2d.h"

using namespace game_engine;

// PROTOTYPE
PathInfo RunAStar(
    const Graph2D& graph,
    const OccupancyGrid2D* occupancy_grid,
    const std::shared_ptr<Node2D>& start_node,
    const std::shared_ptr<Node2D>& end_node);

int main(int argc, char** argv) {
  if(argc != 6) {
    std::cerr << "Usage: ./full_stack_planning occupancy_grid_file row1 col1 row2 col2" << std::endl;
    return EXIT_FAILURE;
  }

  // Parsing input
  const std::string occupancy_grid_file = argv[1];
  const std::shared_ptr<Node2D> start_node = std::make_shared<Node2D>(
      Eigen::Vector2d(std::stoi(argv[2]),std::stoi(argv[3])));
  const std::shared_ptr<Node2D> end_node = std::make_shared<Node2D>(
      Eigen::Vector2d(std::stoi(argv[4]),std::stoi(argv[5])));

  // Load an occupancy grid from a file
  OccupancyGrid2D occupancy_grid;
  occupancy_grid.LoadFromFile(occupancy_grid_file);

  // Transform an occupancy grid into a graph
  const Graph2D graph = occupancy_grid.AsGraph();

  /////////////////////////////////////////////////////////////////////////////
  // RUN A STAR
  // TODO: Run your A* implementation over the graph and nodes defined above.
  //       This section is intended to be more free-form. Using previous
  //       problems and examples, determine the correct commands to complete
  //       this problem. You may want to take advantage of some of the plotting
  //       and graphing utilities in previous problems to check your solution on
  //       the way.
  /////////////////////////////////////////////////////////////////////////////

  auto path_info_ret = RunAStar(graph, &occupancy_grid, start_node, end_node);
  auto node_path = path_info_ret.path;
  
  /////////////////////////////////////////////////////////////////////////////
  // RUN THE POLYNOMIAL PLANNER
  // TODO: Convert the A* solution to a problem the polynomial solver can
  //       solve. Solve the polynomial problem, sample the solution, figure out
  //       a way to export it to Matlab.
  /////////////////////////////////////////////////////////////////////////////

  // Find the number of waypoints
  int N = node_path.size();

  // Time in seconds, a second per waypoint
  std::vector<double> times = {};
  for (double i = 0; i < N; i++) {
    times.push_back(i);
  }

  // Find the first node coordinates
  auto starting_node = node_path[0];
  auto x1 = starting_node->Data().x();
  auto y1 = starting_node->Data().y();
  
  // Initialize the waypoint vector
  std::vector<p4::NodeEqualityBound> node_equality_bounds = {
								   
    // The first node must constrain position, velocity, and acceleration
    p4::NodeEqualityBound(0,0,0,x1),
    p4::NodeEqualityBound(1,0,0,y1),
    p4::NodeEqualityBound(0,0,1,x1),
    p4::NodeEqualityBound(1,0,1,y1),
    p4::NodeEqualityBound(0,0,2,x1),
    p4::NodeEqualityBound(1,0,2,y1),
								   
  };

  // Add the other nodes
  for (double i = 1; i < N; i++) {

    // Find node coordinates
    auto current_node = node_path[i];
    auto x = current_node->Data().x();
    auto y = current_node->Data().y();
    
    // Create waypoint node coordinates constrining position
    node_equality_bounds.push_back(p4::NodeEqualityBound(0,i,0,x));
    node_equality_bounds.push_back(p4::NodeEqualityBound(1,i,0,y));
  }

  // Options to configure the polynomial solver with
  p4::PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 2;     // 2D
  solver_options.polynomial_order = 8;   // Fit an 8th-order polynomial
  solver_options.continuity_order = 4;   // Require continuity to the 4th order
  solver_options.derivative_order = 2;   ////////// CHANGE  Minimize the 2nd order

  osqp_set_default_settings(&solver_options.osqp_settings);
  solver_options.osqp_settings.polish = true;       
  solver_options.osqp_settings.verbose = true;     

  // Use p4::PolynomialSolver object to solve for polynomial trajectories
  p4::PolynomialSolver solver(solver_options);
  const p4::PolynomialSolver::Solution path
    = solver.Run(
        times, 
        node_equality_bounds, 
        {}, 
        {});

  // Sampling the desired derivative
  p4::PolynomialSampler::Options sampler_options;
  sampler_options.frequency = 200;             // Number of samples per second
  sampler_options.derivative_order = 0;        // Derivative to sample (0 = pos)
  
  // Use this object to sample a trajectory
  p4::PolynomialSampler sampler(sampler_options);
  Eigen::MatrixXd samples = sampler.Run(times, path);
  
  // Plotting tool requires vectors
  std::vector<std::string> t_hist, x_hist, y_hist;
  for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
    t_hist.push_back(std::__cxx11::to_string(samples(0,time_idx)));
    x_hist.push_back(std::__cxx11::to_string(samples(1,time_idx)));
    y_hist.push_back(std::__cxx11::to_string(samples(2,time_idx)));
  }

  // Download the time history of the desired derivative
  std::ofstream MyFile("./data.txt");

  // Create easiest format file
  for (int i = 0; i < t_hist.size(); i++) {
    MyFile<<y_hist[i]<<", "<<x_hist[i]<<", 0"<<std::endl;
  }
  
  return EXIT_SUCCESS;
}

// HELPER FUNCTIONS
PathInfo RunAStar(
    const Graph2D& graph,
    const OccupancyGrid2D* occupancy_grid,
    const std::shared_ptr<Node2D>& start_node,
    const std::shared_ptr<Node2D>& end_node) {
  
  // Run A*
  AStar2D a_star;
  PathInfo path_info = a_star.Run(graph, start_node, end_node);
  return path_info;
}
