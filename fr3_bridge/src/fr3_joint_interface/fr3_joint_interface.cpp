#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include "franka_trajectory_utils.hpp"
#include "lcm/lcm-cpp.hpp"
#include "ipc_trigger_t.hpp"
#include <signal.h>
#include <chrono>
#include "lcm_msgs/fr3_states/fr3_state.hpp"
#include "lcm_msgs/fr3_commands/fr3_cmd.hpp"
#include <sys/select.h>


namespace
{
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array)
{
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

bool running = true;

void signal_callback_handler(int signum)
{
  //  std::cout << "Caught signal " << signum << std::endl;
  running = false;
  exit(signum);
}

// LCM communication channel subsystem
class LCMCommandHandler
{
public:
  ~LCMCommandHandler()
  {
  }
  uint64_t timestamp;
  double cmd[7];
  void handleMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const fr3_commands::fr3_cmd* msg)
  {
    for (int i = 0; i < 7; i++)
      cmd[i] = msg->cmd[i];
    timestamp = msg->timestamp;
    // std::cout << timestamp << std::endl;
  }
};

void lcmThreadFunc(lcm::LCM* lcm)
{
  while (running)
  {
    lcm->handle();
  }
}

int main(int argc, char** argv)
{
  // Check whether the required arguments were passed.
  if (argc < 4 || argc > 5)
  {
    std::cout << argc;
    std::cerr << "Usage: " << argv[0] << " <robot hostname>"
              << " <robot name>"
              << " <interface type>" << "optional camera mass" << std::endl;
    return -1;
  }
  signal(SIGINT, signal_callback_handler);

  // Load the DGM parameters and start the graph in a separate thread
  std::string robot_name = std::string(argv[2]);

  lcm::LCM lcm("udpm://239.255.76.67:7667?ttl=1");
  if (!lcm.good())
  {
    std::cout << "Can not initialize the LCM connection! " << std::endl;
    return -1;
  }

  fr3_states::fr3_state lcm_state_msg;
  
  LCMCommandHandler cmdShm;
  lcm.subscribe(robot_name + "_command", &LCMCommandHandler::handleMessage, &cmdShm);
  std::thread lcmThread(lcmThreadFunc, &lcm);

  // Start communicating with the robot hardware initiate the controller loop
  try
  {
    // Connect to robot.
    franka::Robot robot(argv[1]);
    franka::Model model(robot.loadModel());

    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = { { 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4 } };
    MotionGenerator motion_generator(0.5, q_goal);
    system("clear");
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
    if (strcmp(argv[3], "torque") == 0)
      std::cout << "Control interface is: Torque" << std::endl;
    else if (strcmp(argv[3], "velocity") == 0)
      std::cout << "Control interface is: velocity" << std::endl;
    else
    {
      std::cout << "Control interface not implemented!" << std::endl;
      return -1;
    }

    std::cout << "Publishing robot states on: " << robot_name + "_state" << std::endl;
    std::cout << "Subscribing to robot commands on: " << robot_name + "_command" << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        { { 20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0 } }, { { 20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0 } },
        { { 20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0 } }, { { 20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0 } },
        { { 20.0, 20.0, 20.0, 25.0, 25.0, 25.0 } }, { { 20.0, 20.0, 20.0, 25.0, 25.0, 25.0 } },
        { { 20.0, 20.0, 20.0, 25.0, 25.0, 25.0 } }, { { 20.0, 20.0, 20.0, 25.0, 25.0, 25.0 } });
    // if an extra camera weight compensation argument is provided, set the camera weight (realsense d435)
    if(argc==5)
    {
      std::cout << "Setting load for the Realsense D435i" << std::endl;
      std::array<double, 3> offset;
      std::array<double, 9> inertia;
      double mass = 0.097;
      offset[0] = 0.035;
      offset[1] = 0.035;
      offset[2] = 0.1;
      for(int i=0; i<9; i++)
      {
        inertia[i] = 1e-7;
      }
      robot.setLoad(mass, offset, inertia);
    }
    // Define callback for control loop.
    if (strcmp(argv[3], "torque") == 0)
    {
      std::function<franka::Torques(const franka::RobotState&, franka::Duration)> control_callback =
          [&cmdShm, &robot_name, &lcm, &lcm_state_msg, &model](const franka::RobotState& state,
                                                       franka::Duration /*period*/) -> franka::Torques {
        std::array<double, 7> coriolis = model.coriolis(state);
        std::array<double, 7> gravity = model.gravity(state);
        std::array<double, 49> mass = model.mass(state);
        // Get the current CPU time
        auto stamp_now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = stamp_now.time_since_epoch();

        // load the LCM state message to be transmitted
        lcm_state_msg.timestamp =
            std::chrono::duration_cast<std::chrono::microseconds>(stamp_now.time_since_epoch()).count();
        for (size_t i = 0; i < 7; i++)
        {
          lcm_state_msg.q[i] = state.q[i];
          lcm_state_msg.dq[i] = state.dq[i];
          lcm_state_msg.T[i] = state.tau_J[i];
          lcm_state_msg.G[i] = gravity[i];
          lcm_state_msg.C[i] = coriolis[i];
        }
        
        for (size_t i = 0; i < 49; i++)
          lcm_state_msg.M[i] = mass[i];
        lcm.publish(robot_name + "_state", &lcm_state_msg);
        // How recent is the computed control command?
        uint64_t current_time =
            std::chrono::duration_cast<std::chrono::microseconds>(stamp_now.time_since_epoch()).count();

        double control_lag = current_time - cmdShm.timestamp;

        std::array<double, 7> tau_d_calculated;

        // Apply the control command to the robot only if the command is recent enough
        if (control_lag < 0.2 * 1e7)
        {
          for (size_t i = 0; i < 7; i++)
          {
            tau_d_calculated[i] = cmdShm.cmd[i];
          }
        }
        else
        {
          for (size_t i = 0; i < 7; i++)
          {
            tau_d_calculated[i] = 0;
          }
        }

        // The following line is only necessary for printing the rate limited torque. As we activated
        // rate limiting for the control loop (activated by default), the torque would anyway be
        // adjusted!
        std::array<double, 7> tau_d_rate_limited =
            franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);

        // Send torque command.
        return tau_d_rate_limited;
      };
      // Start real-time control loop.
      robot.control(control_callback);
    }
    else if (strcmp(argv[3], "velocity") == 0)
    {
      std::function<franka::JointVelocities(const franka::RobotState&, franka::Duration)> control_callback =
          [&cmdShm, &robot_name, &lcm, &lcm_state_msg](const franka::RobotState& state,
                                                       franka::Duration /*period*/) -> franka::JointVelocities {
        // Get the current CPU time
        auto stamp_now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = stamp_now.time_since_epoch();

        // load the LCM state message to be transmitted
        lcm_state_msg.timestamp =
            std::chrono::duration_cast<std::chrono::microseconds>(stamp_now.time_since_epoch()).count();
        for (size_t i = 0; i < 7; i++)
        {
          lcm_state_msg.q[i] = state.q[i];
          lcm_state_msg.dq[i] = state.dq[i];
          lcm_state_msg.T[i] = state.tau_J[i];
        }
        lcm.publish(robot_name + "_state", &lcm_state_msg);
        // How recent is the computed control command?
        uint64_t current_time =
            std::chrono::duration_cast<std::chrono::microseconds>(stamp_now.time_since_epoch()).count();

        double control_lag = current_time - cmdShm.timestamp;

        std::array<double, 7> vel_d_calculated;

        // Apply the control command to the robot only if the command is recent enough
        if (control_lag < 0.2 * 1e7)
        {
          for (size_t i = 0; i < 7; i++)
          {
            vel_d_calculated[i] = cmdShm.cmd[i];
          }
        }
        else
        {
          for (size_t i = 0; i < 7; i++)
          {
            vel_d_calculated[i] = 0;
          }
        }

        // The following line is only necessary for printing the rate limited torque. As we activated
        // rate limiting for the control loop (activated by default), the torque would anyway be
        // adjusted!
        // std::array<double, 7> vel_d_rate_limited =
            // franka::limitRate(franka::kMaxElbowVelocity, vel_d_calculated, state.dq_d);

        // Send torque command.
        return vel_d_calculated;
      };
      // Start real-time control loop.
      robot.control(control_callback);
    }
  }
  catch (const franka::Exception& ex)
  {
    std::cerr << ex.what() << std::endl;
  }

  return 0;
}
