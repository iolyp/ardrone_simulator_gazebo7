
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>

#include <std_msgs/Float64.h>
#include <gtddp_drone_msgs/state_data.h>

// Constants
const double gravity = 9.81;

// Initialization flags
bool sim_init = false;

// Track the force
double force;

// System parameters
double bob_mass;
double cart_mass;
double max_force;
double pole_length;

// Track state
Eigen::Vector4d state;


// System state Publisher
ros::Publisher state_pub;


void controlCallback(const std_msgs::Float64::ConstPtr& ctrl)
{
    // Update the current force
    force = ctrl->data;

    // Make sure the force stays within the boundaries
    if(force < -max_force)
    {
        force = -max_force;
    }
    else if(force > max_force)
    {
        force = max_force;
    }
}


void statePubCallback(const ros::TimerEvent event)
{
    // Make the state data message
    gtddp_drone_msgs::state_data state_msg;

    // Set the state data message to the right size
    state_msg.state.resize(4);

    // Populate the state
    for(int i = 0; i < 4; ++i)
    {
        state_msg.state[i] = state(i);
    }

    // Publish
    state_pub.publish(state_msg);
}


void simLoop(const ros::TimerEvent& event)
{
    // Make the timer run once before calculating dt
    if(!sim_init)
    {
        sim_init = true;
        return;
    }

    // Find timestep
    double dt = (event.current_real - event.last_real).toSec();

    // Name states to make it easier
    double x_dot     = state(1);
    double theta     = state(2);
    double theta_dot = state(3);
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);

    // Apply dynamics to cart position and pole rotation
    state(0) += x_dot * dt;
    state(2) += theta_dot * dt;

    // Constrain the pole rotation measurement to [0, 2*pi]
    // state(2) = fmod(state(2), 2*M_PI);

    // Apply control to the cart velocity
    double cart_accel = (-bob_mass*pole_length*pow(theta_dot,2)*sin_theta + bob_mass*gravity*sin_theta*cos_theta + force)/(cart_mass+bob_mass*pow(sin_theta,2));
    state(1) += cart_accel * dt;

    // Apply control to the pole's angular velocity
    double pole_velocity = (-bob_mass*pole_length*pow(theta_dot,2)*sin_theta*cos_theta + (cart_mass+bob_mass)*gravity*sin_theta + force*cos_theta) / (cart_mass*pole_length + bob_mass*pole_length*pow(sin_theta,2));
    state(3) += pole_velocity * dt;
}


int main(int argc, char **argv)
{
    // Initialize
    ros::init(argc, argv, "cart_pole_sim_node");

    // Define the node
    ros::NodeHandle cpsim;

    // Initialize the state
    std::vector<double> init_state;

    if(!cpsim.getParam("/init_state", init_state))
    {
        ROS_WARN("Warning: no initial state provided");

        // Start with the default start state (pendulum up, cart sitting still)
        state << 0, 0, 0, 0;
    }
    else
    {
        // Copy initial conditions from the temporary vector
        for(int i=0; i < 4; ++i)
        {
            state(i) = init_state[i];
        }
    }

    // Initialize the system paramters
    bob_mass = cpsim.param<double>("/mass", 0.50);
    cart_mass = cpsim.param<double>("/cart_mass", 10.0);
    max_force = cpsim.param<double>("/max_force", 100.0);
    pole_length = cpsim.param<double>("/length", 1.0);

    // Initialize force
    force = 0.0;

    // Subscribe to the control signal
    ros::Subscriber ctrl_sub = cpsim.subscribe(cpsim.resolveName("/control"), 1, &controlCallback);

    // Publisher for the system state
    state_pub = cpsim.advertise<gtddp_drone_msgs::state_data>(cpsim.resolveName("/cart/state"), 1);

    // Simulator main thread and state estimator are run on timers
    ros::Timer sim_timer = cpsim.createTimer(ros::Duration(0.001), &simLoop);   // 1000 Hz
    ros::Timer state_pub_timer = cpsim.createTimer(ros::Duration(1), &statePubCallback); // 100 Hz

    // Keep thread alive / pump callbacks
    ros::spin();

    return 0;
}
