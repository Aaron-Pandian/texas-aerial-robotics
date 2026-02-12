#include <cstdlib>
#include <vector>
#include <cmath>

#include "polynomial_solver.h"
#include "polynomial_sampler.h"
#include "gnuplot-iostream.h"

///////////////////////////////////////////////////////////////////
// FUNCTION PROTOTYPES
// DO NOT MODIFY
///////////////////////////////////////////////////////////////////
// void Example();
// void DerivativeExperiments();
// void ArrivalTimeExperiments();
void NumWaypointExperiments();

///////////////////////////////////////////////////////////////////
// MAIN FUNCTION
// TODO: UNCOMMENT THE FUNCTIONS YOU WANT TO RUN
///////////////////////////////////////////////////////////////////
int main(int argc, char** argv) {
  // Example();
  // DerivativeExperiments();
  // ArrivalTimeExperiments();
  NumWaypointExperiments();

  return EXIT_SUCCESS;
}

// Example function that demonstrates how to use the polynomial solver. This
// example creates waypoints in a triangle: (0,0) -- (1,0) -- (1,1) -- (0,0)
void Example() {
  // Time in seconds
  const std::vector<double> times = {0,1,2,3};

  // The parameter order for p4::NodeEqualityBound is:
  // (dimension_index, node_idx, derivative_idx, value)
  const std::vector<p4::NodeEqualityBound> node_equality_bounds = {
    // The first node must constrain position, velocity, and acceleration
    p4::NodeEqualityBound(0,0,0,0),
    p4::NodeEqualityBound(1,0,0,0),
    p4::NodeEqualityBound(0,0,1,0),
    p4::NodeEqualityBound(1,0,1,0),
    p4::NodeEqualityBound(0,0,2,0),
    p4::NodeEqualityBound(1,0,2,0),

    // The second node constrains position
    p4::NodeEqualityBound(0,1,0,1),
    p4::NodeEqualityBound(1,1,0,0),

    // The third node constrains position
    p4::NodeEqualityBound(0,2,0,1),
    p4::NodeEqualityBound(1,2,0,1),

    // The fourth node constrains position
    p4::NodeEqualityBound(0,3,0,0),
    p4::NodeEqualityBound(1,3,0,0),
  };

  // Options to configure the polynomial solver with
  p4::PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 2;     // 2D
  solver_options.polynomial_order = 8;   // Fit an 8th-order polynomial
  solver_options.continuity_order = 4;   // Require continuity to the 4th order
  solver_options.derivative_order = 2;   ////////// CHANGE  Minimize the 2nd order

  osqp_set_default_settings(&solver_options.osqp_settings);
  solver_options.osqp_settings.polish = true;       // Polish the solution, getting the best answer possible
  solver_options.osqp_settings.verbose = true;     // Suppress the printout

  // Use p4::PolynomialSolver object to solve for polynomial trajectories
  p4::PolynomialSolver solver(solver_options);
  const p4::PolynomialSolver::Solution path
    = solver.Run(
        times, 
        node_equality_bounds, 
        {}, 
        {});

  // Sampling and Plotting
  { // Plot 2D position
    // Options to configure the polynomial sampler with
    p4::PolynomialSampler::Options sampler_options;
    sampler_options.frequency = 100;             // Number of samples per second
    sampler_options.derivative_order = 0;        // Derivative to sample (0 = pos)

    // Use this object to sample a trajectory
    p4::PolynomialSampler sampler(sampler_options);
    Eigen::MatrixXd samples = sampler.Run(times, path);

    // Plotting tool requires vectors
    std::vector<double> t_hist, x_hist, y_hist;
    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
      t_hist.push_back(samples(0,time_idx));
      x_hist.push_back(samples(1,time_idx));
      y_hist.push_back(samples(2,time_idx));
    }

    // gnu-iostream plotting library
    // Utilizes gnuplot commands with a nice stream interface
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Trajectory'" << std::endl;
      gp.send1d(boost::make_tuple(x_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'X'" << std::endl;
      gp << "set ylabel 'Y'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'X-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, x_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'X-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Y-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'Y-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
  }
}

void DerivativeExperiments() {
  // Time in seconds
  const std::vector<double> times = {0,1,2,3,4};

  // The parameter order for p4::NodeEqualityBound is:
  // (dimension_index, node_idx, derivative_idx, value)
  const std::vector<p4::NodeEqualityBound> node_equality_bounds = {
    ///////////////////////////////////////////////////////////////////
    // TODO: CREATE A SQUARE TRAJECTORY
    ///////////////////////////////////////////////////////////////////

    // The first node must constrain position, velocity, and acceleration
    p4::NodeEqualityBound(0,0,0,0),
    p4::NodeEqualityBound(1,0,0,0),
    p4::NodeEqualityBound(0,0,1,0),
    p4::NodeEqualityBound(1,0,1,0),
    p4::NodeEqualityBound(0,0,2,0),
    p4::NodeEqualityBound(1,0,2,0),

    // The second node constrains position
    p4::NodeEqualityBound(0,1,0,1),
    p4::NodeEqualityBound(1,1,0,0),

    // The third node constrains position
    p4::NodeEqualityBound(0,2,0,1),
    p4::NodeEqualityBound(1,2,0,1),

    // The third node constrains position
    p4::NodeEqualityBound(0,3,0,0),
    p4::NodeEqualityBound(1,3,0,1),

    // The fifth node constrains position
    p4::NodeEqualityBound(0,4,0,0),
    p4::NodeEqualityBound(1,4,0,0),
  };

  // Options to configure the polynomial solver with
  p4::PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 2;     // 2D
  solver_options.polynomial_order = 8;   // Fit an 8th-order polynomial
  solver_options.continuity_order = 4;   // Require continuity to the 4th order
  solver_options.derivative_order = 4;   // TODO: VARY THE DERIVATIVE ORDER

  osqp_set_default_settings(&solver_options.osqp_settings);
  solver_options.osqp_settings.polish = true;       // Polish the solution, getting the best answer possible
  solver_options.osqp_settings.verbose = false;     // Suppress the printout

  // Use p4::PolynomialSolver object to solve for polynomial trajectories
  p4::PolynomialSolver solver(solver_options);
  const p4::PolynomialSolver::Solution path
    = solver.Run(
        times, 
        node_equality_bounds, 
        {}, 
        {});

  // Sampling and Plotting
  { // Plot 2D position
    // Options to configure the polynomial sampler with
    p4::PolynomialSampler::Options sampler_options;
    sampler_options.frequency = 100;             // Number of samples per second
    sampler_options.derivative_order = 0;        // IMPORTANT: Derivative to sample

    // Use this object to sample a trajectory
    p4::PolynomialSampler sampler(sampler_options);
    Eigen::MatrixXd samples = sampler.Run(times, path);

    // Plotting tool requires vectors
    std::vector<double> t_hist, x_hist, y_hist;
    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
      t_hist.push_back(samples(0,time_idx));
      x_hist.push_back(samples(1,time_idx));
      y_hist.push_back(samples(2,time_idx));
    }

    // gnu-iostream plotting library
    // Utilizes gnuplot commands with a nice stream interface
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Trajectory'" << std::endl;
      gp.send1d(boost::make_tuple(x_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'X'" << std::endl;
      gp << "set ylabel 'Y'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'X-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, x_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'X-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Y-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'Y-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
  }
}

void ArrivalTimeExperiments() {
  // Time in seconds
  const std::vector<double> times = {0,1,2,3,4};
  // const std::vector<double> times = {0,0.02,0.04,0.06,0.08};
  // const std::vector<double> times = {0,10,20,30,40};
  
  // The parameter order for p4::NodeEqualityBound is:
  // (dimension_index, node_idx, derivative_idx, value)
  const std::vector<p4::NodeEqualityBound> node_equality_bounds = {
    ///////////////////////////////////////////////////////////////////
    // TODO: CREATE A SQUARE TRAJECTORY
    ///////////////////////////////////////////////////////////////////
								   
    // The first node must constrain position, velocity, and acceleration
    p4::NodeEqualityBound(0,0,0,0),
    p4::NodeEqualityBound(1,0,0,0),
    p4::NodeEqualityBound(0,0,1,0),
    p4::NodeEqualityBound(1,0,1,0),
    p4::NodeEqualityBound(0,0,2,0),
    p4::NodeEqualityBound(1,0,2,0),

    // The second node constrains position
    p4::NodeEqualityBound(0,1,0,1),
    p4::NodeEqualityBound(1,1,0,0),

    // The third node constrains position
    p4::NodeEqualityBound(0,2,0,1),
    p4::NodeEqualityBound(1,2,0,1),

    // The third node constrains position
    p4::NodeEqualityBound(0,3,0,0),
    p4::NodeEqualityBound(1,3,0,1),

    // The fifth node constrains position
    p4::NodeEqualityBound(0,4,0,0),
    p4::NodeEqualityBound(1,4,0,0),
  };

  // Options to configure the polynomial solver with
  p4::PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 2;     // 2D
  solver_options.polynomial_order = 8;   // Fit an 8th-order polynomial
  solver_options.continuity_order = 4;   // Require continuity to the 4th order
  solver_options.derivative_order = 2;   // IMPORTANT

  osqp_set_default_settings(&solver_options.osqp_settings);
  solver_options.osqp_settings.polish = true;       // Polish the solution, getting the best answer possible
  solver_options.osqp_settings.verbose = false;     // Suppress the printout

  // Use p4::PolynomialSolver object to solve for polynomial trajectories
  p4::PolynomialSolver solver(solver_options);
  const p4::PolynomialSolver::Solution path
    = solver.Run(
        times, 
        node_equality_bounds, 
        {}, 
        {});

  // Sampling and Plotting
  { // Plot 2D position
    // Options to configure the polynomial sampler with
    p4::PolynomialSampler::Options sampler_options;
    sampler_options.frequency = 100;             // Number of samples per second
    sampler_options.derivative_order = 0;        // Derivative to sample (0 = pos)

    // Use this object to sample a trajectory
    p4::PolynomialSampler sampler(sampler_options);
    Eigen::MatrixXd samples = sampler.Run(times, path);

    // Plotting tool requires vectors
    std::vector<double> t_hist, x_hist, y_hist;
    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
      t_hist.push_back(samples(0,time_idx));
      x_hist.push_back(samples(1,time_idx));
      y_hist.push_back(samples(2,time_idx));
    }

    // gnu-iostream plotting library
    // Utilizes gnuplot commands with a nice stream interface
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Trajectory'" << std::endl;
      gp.send1d(boost::make_tuple(x_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'X'" << std::endl;
      gp << "set ylabel 'Y'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'X-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, x_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'X-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Y-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'Y-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
  }
}

void NumWaypointExperiments() {
  // Independant variables
  double N = 100;
  int r = 1;
  
  // Time in seconds
  std::vector<double> times = {};
  for (double i = 0; i <= N; i++) {
    times.push_back(i);
  }
  // std::vector<double> times = {0,1,2,3,4};
  // std::vector<double> times = {0,1,2,3,4,5,6,7,8};
  // std::vector<double> times = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
  
  // Initialize the vector
  std::vector<p4::NodeEqualityBound> node_equality_bounds = {
								   
    // The first node must constrain position, velocity, and acceleration at (1,0)
    p4::NodeEqualityBound(0,0,0,r),
    p4::NodeEqualityBound(1,0,0,0),
    p4::NodeEqualityBound(0,0,1,r),
    p4::NodeEqualityBound(1,0,1,0),
    p4::NodeEqualityBound(0,0,2,r),
    p4::NodeEqualityBound(1,0,2,0),
								   
  };

  // Add the other nodes
  
  // Set PI
  const double PI = M_PI;

  // Set the angle variation
  double angle_shift = (2*PI)/N;
  double curr_angle = angle_shift;
  
  // Create a for loop that vaires with N waypoints
  for (double i = 1; i <= N; i++) {
    
    // Find the next point using the angle variation
    double x = r*cos(curr_angle);
    double y = r*sin(curr_angle);

    // Create waypoint node coordinates constrining position
    node_equality_bounds.push_back(p4::NodeEqualityBound(0,i,0,x));
    node_equality_bounds.push_back(p4::NodeEqualityBound(1,i,0,y));
      
    // Update curr_angle
    curr_angle += angle_shift;
  }

    
  // Options to configure the polynomial solver with
  p4::PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 2;     // 2D
  solver_options.polynomial_order = 8;   // Fit an 8th-order polynomial
  solver_options.continuity_order = 4;   // Require continuity to the 4th order
  solver_options.derivative_order = 4;   // Minimize snap

  osqp_set_default_settings(&solver_options.osqp_settings);
  solver_options.osqp_settings.polish = true;       // Polish the solution, getting the best answer possible
  solver_options.osqp_settings.verbose = false;     // Suppress the printout

  // Use p4::PolynomialSolver object to solve for polynomial trajectories
  p4::PolynomialSolver solver(solver_options);
  const p4::PolynomialSolver::Solution path
    = solver.Run(
        times, 
        node_equality_bounds, 
        {}, 
        {});

  // Sampling and Plotting
  { // Plot 2D position
    // Options to configure the polynomial sampler with
    p4::PolynomialSampler::Options sampler_options;
    sampler_options.frequency = 100;             // Number of samples per second
    sampler_options.derivative_order = 0;        // Derivative to sample (0 = pos)

    // Use this object to sample a trajectory
    p4::PolynomialSampler sampler(sampler_options);
    Eigen::MatrixXd samples = sampler.Run(times, path);

    // Plotting tool requires vectors
    std::vector<double> t_hist, x_hist, y_hist;
    for(size_t time_idx = 0; time_idx < samples.cols(); ++time_idx) {
      t_hist.push_back(samples(0,time_idx));
      x_hist.push_back(samples(1,time_idx));
      y_hist.push_back(samples(2,time_idx));
    }

    // gnu-iostream plotting library
    // Utilizes gnuplot commands with a nice stream interface
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Trajectory'" << std::endl;
      gp.send1d(boost::make_tuple(x_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'X'" << std::endl;
      gp << "set ylabel 'Y'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'X-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, x_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'X-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
    {
      Gnuplot gp;
      gp << "plot '-' using 1:2 with lines title 'Y-Profile'" << std::endl;
      gp.send1d(boost::make_tuple(t_hist, y_hist));
      gp << "set grid" << std::endl;
      gp << "set xlabel 'Time (s)'" << std::endl;
      gp << "set ylabel 'Y-Profile'" << std::endl;
      gp << "replot" << std::endl;
    }
  }
}
