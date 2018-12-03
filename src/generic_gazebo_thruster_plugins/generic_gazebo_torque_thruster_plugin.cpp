#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  // Torque "thrusters" don't exist for real, but reaction wheels and control
  // moment gyroscopes do. This plugin simulates a torque actuator.
  class GenericGazeboTorqueThrusterPlugin : public ModelPlugin
  {
  protected:
    std::unique_ptr<ros::NodeHandle> ros_node_ptr_;
    ros::Subscriber ros_throttle_command_sub_;
    ros::Publisher ros_throttle_status_pub_;
    ros::Publisher ros_thrust_status_pub_;
    ros::CallbackQueue ros_callback_queue_;
    std::thread ros_callback_thread_;
    physics::ModelPtr model_ptr_;
    event::ConnectionPtr simulation_step_connection_;
    ignition::math::Vector3d torque_axis_;
    std::string thruster_name_;
    physics::LinkPtr thruster_body_;
    double rated_thrust_;
    double min_throttle_;
    double max_throttle_;
    std::atomic<double> throttle_;

    void CommandMessageCallback(const std_msgs::Float64ConstPtr& msg)
    {
      const double commanded_throttle = msg->data;
      if (std::abs(commanded_throttle) < std::numeric_limits<double>::epsilon())
      {
        ROS_DEBUG(
            "[Gazebo torque thruster plugin: %s] Commanded throttle below "
            "epsilon, throttling to zero", thruster_name_.c_str());
        throttle_ = 0.0;
      }
      else if (std::abs(commanded_throttle) < min_throttle_)
      {
        ROS_WARN(
            "[Gazebo torque thruster plugin: %s] Commanded throttle %f below "
            "min throttle, throttling to min throttle %f",
            thruster_name_.c_str(), std::abs(commanded_throttle),
            min_throttle_);
        throttle_ = min_throttle_;
      }
      else if (std::abs(commanded_throttle) > max_throttle_)
      {
        ROS_WARN(
            "[Gazebo torque thruster plugin: %s] Commanded throttle %f above "
            "max throttle, throttling to max throttle %f",
            thruster_name_.c_str(), std::abs(commanded_throttle),
            max_throttle_);
        throttle_ = max_throttle_;
      }
      else
      {
        ROS_DEBUG(
            "[Gazebo torque thruster plugin: %s] Commanded throttle to %f",
            thruster_name_.c_str(), std::abs(commanded_throttle));
        throttle_ = std::abs(commanded_throttle);
      }
    }

    void ROSCallbackThread()
    {
      const double timeout = 0.01;
      while (ros_node_ptr_->ok())
      {
        ros_callback_queue_.callAvailable(ros::WallDuration(timeout));
      }
    }

  public:

    GenericGazeboTorqueThrusterPlugin()
        : torque_axis_(ignition::math::Vector3d::UnitZ),
          rated_thrust_(0.0), min_throttle_(0.0), max_throttle_(0.0),
          throttle_(0.0) {}

    virtual ~GenericGazeboTorqueThrusterPlugin()
    {
      if (ros_node_ptr_)
      {
        ros_node_ptr_->shutdown();
        ros_node_ptr_.reset();
      }
    }

    /* Setup the plugin on load */
    virtual void Load(physics::ModelPtr model_ptr, sdf::ElementPtr sdf_ptr)
    {
      // Store the model pointer for convenience.
      model_ptr_ = model_ptr;
      // Load parameters
      if (sdf_ptr->HasElement("rated_thrust"))
      {
        rated_thrust_ = std::abs(sdf_ptr->Get<double>("rated_thrust"));
      }
      else
      {
        throw std::invalid_argument("No rated_thrust parameter provided");
      }
      if (sdf_ptr->HasElement("min_throttle"))
      {
        min_throttle_ = std::abs(sdf_ptr->Get<double>("min_throttle"));
      }
      else
      {
        throw std::invalid_argument("No min_throttle parameter provided");
      }
      if (sdf_ptr->HasElement("max_throttle"))
      {
        const double max_throttle =
            std::abs(sdf_ptr->Get<double>("max_throttle"));
        if (max_throttle >= min_throttle_)
        {
          max_throttle_ = max_throttle;
        }
        else
        {
          throw std::invalid_argument(
              "Provided max_throttle is lower than min_throttle");
        }
      }
      else
      {
        throw std::invalid_argument("No max_throttle parameter provided");
      }
      if (sdf_ptr->HasElement("thruster_name"))
      {
        const std::string thruster_name =
            sdf_ptr->Get<std::string>("thruster_name");
        thruster_name_ = thruster_name;
      }
      if (sdf_ptr->HasElement("thruster_body"))
      {
        const std::string thruster_body_name =
            sdf_ptr->Get<std::string>("thruster_body");
        const physics::LinkPtr thruster_link =
            model_ptr_->GetLink(thruster_body_name);
        if (thruster_link)
        {
          if (thruster_name_ == "")
          {
            std::cerr << "No/invalid thruster name provided,"
                      << "defaulted to using thruster_body_name" << std::endl;
            thruster_name_ = thruster_body_name;
          }
          thruster_body_ = thruster_link;
        }
        else
        {
          const std::string err_msg =
              "Invalid thruster_body parameter provided: " + thruster_body_name
              + " - cannot find link in model";
          throw std::invalid_argument(err_msg);
        }
      }
      else
      {
        throw std::invalid_argument("No thruster_body parameter provided");
      }
      if (sdf_ptr->HasElement("body_torque_axis"))
      {
        ignition::math::Vector3d torque_axis =
            sdf_ptr->Get<ignition::math::Vector3d>("body_torque_axis");
        const double mag = torque_axis.Length();
        if (mag > 0.0 && !std::isnan(mag))
        {
          torque_axis_ = torque_axis.Normalize();
        }
        else
        {
          throw std::invalid_argument(
              "Invalid body_torque_axis parameter provided");
        }
      }
      else
      {
        throw std::invalid_argument(
            "No body_torque_axis parameter provided");
      }
      // Listen to the update event.
      // This event is broadcast every simulation iteration.
      simulation_step_connection_ =
          event::Events::ConnectWorldUpdateBegin(
              boost::bind(&GenericGazeboTorqueThrusterPlugin::SimulationStep,
                          this, _1));
      // Initialize ros, if it has not already been initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(
            argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
      }
      ros_node_ptr_.reset(new ros::NodeHandle());
      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions subscribe_options =
          ros::SubscribeOptions::create<std_msgs::Float64>(
              model_ptr_->GetName() + "/" + thruster_name_
              + "/throttle/command", 1,
              boost::bind(
                  &GenericGazeboTorqueThrusterPlugin::CommandMessageCallback,
                  this, _1),
              ros::VoidPtr(), &ros_callback_queue_);
      ros_throttle_command_sub_ = ros_node_ptr_->subscribe(subscribe_options);
      ros_throttle_status_pub_ =
          ros_node_ptr_->advertise<std_msgs::Float64>(
              model_ptr_->GetName() + "/" + thruster_name_ + "/throttle/status",
              1, true);
      ros_thrust_status_pub_ =
          ros_node_ptr_->advertise<geometry_msgs::WrenchStamped>(
              model_ptr_->GetName() + "/" + thruster_name_ + "/thrust/status",
              1, true);
      // Spin up the queue helper thread.
      ros_callback_thread_ =
          std::thread(std::bind(
              &GenericGazeboTorqueThrusterPlugin::ROSCallbackThread, this));
    }

    // Called at each simulation step, this is where we apply the torque
    void SimulationStep(const common::UpdateInfo&)
    {
      // Compute the current thrust vector
      const double thrust_torque = rated_thrust_ * throttle_;
      const ignition::math::Vector3d body_frame_torque_vector =
          torque_axis_ * thrust_torque;
      // Apply the thrust torque
      const ignition::math::Pose3d thruster_body_pose =
          thruster_body_->WorldCoGPose();
      const ignition::math::Vector3d world_frame_torque_vector =
          thruster_body_pose.Rot().RotateVector(body_frame_torque_vector);
      thruster_body_->AddTorque(world_frame_torque_vector);
      // Update the status
      std_msgs::Float64 throttle_msg;
      throttle_msg.data = throttle_;
      ros_throttle_status_pub_.publish(throttle_msg);
      geometry_msgs::WrenchStamped thrust_msg;
      thrust_msg.header.stamp = ros::Time::now();
      thrust_msg.header.frame_id = thruster_body_->GetName();
      thrust_msg.wrench.force.x = 0.0;
      thrust_msg.wrench.force.y = 0.0;
      thrust_msg.wrench.force.z = 0.0;
      thrust_msg.wrench.torque.x = body_frame_torque_vector.X();
      thrust_msg.wrench.torque.y = body_frame_torque_vector.Y();
      thrust_msg.wrench.torque.z = body_frame_torque_vector.Z();
      ros_thrust_status_pub_.publish(thrust_msg);
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(GenericGazeboTorqueThrusterPlugin)
}

