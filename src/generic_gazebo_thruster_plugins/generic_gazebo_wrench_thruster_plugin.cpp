#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
  class GenericGazeboWrenchThrusterPlugin : public ModelPlugin
  {
  protected:
    std::unique_ptr<ros::NodeHandle> ros_node_ptr_;
    ros::Subscriber ros_thrust_command_sub_;
    ros::Publisher ros_thrust_status_pub_;
    ros::CallbackQueue ros_callback_queue_;
    std::thread ros_callback_thread_;
    physics::ModelPtr model_ptr_;
    event::ConnectionPtr simulation_step_connection_;
    ignition::math::Vector3d body_frame_force_vector_;
    ignition::math::Vector3d body_frame_torque_vector_;
    std::mutex command_mutex_;
    std::string thruster_name_;
    physics::LinkPtr thruster_body_;

    void CommandMessageCallback(const geometry_msgs::WrenchStampedConstPtr& msg)
    {
      if (msg->header.frame_id == thruster_body_->GetName())
      {
        ROS_DEBUG(
            "[Gazebo wrench thruster plugin: %s] Command [%f, %f, %f], [%f, %f,"
            " %f]", thruster_name_.c_str(), msg->wrench.force.x,
            msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x,
            msg->wrench.torque.y, msg->wrench.torque.z);
        std::lock_guard<std::mutex> lock(command_mutex_);
        body_frame_force_vector_ =
            ignition::math::Vector3d(msg->wrench.force.x,
                                     msg->wrench.force.y,
                                     msg->wrench.force.z);
        body_frame_torque_vector_ =
            ignition::math::Vector3d(msg->wrench.torque.x,
                                     msg->wrench.torque.y,
                                     msg->wrench.torque.z);
      }
      else
      {
        ROS_WARN(
            "[Gazebo wrench thruster plugin: %s] Command with frame_id [%s] "
            " does not match thruster body frame [%s], set to zero wrench",
            thruster_name_.c_str(), msg->header.frame_id.c_str(),
            thruster_body_->GetName().c_str());
        body_frame_force_vector_ = ignition::math::Vector3d(0.0, 0.0, 0.0);
        body_frame_torque_vector_ = ignition::math::Vector3d(0.0, 0.0, 0.0);
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

    GenericGazeboWrenchThrusterPlugin()
        : body_frame_force_vector_(0.0, 0.0, 0.0),
          body_frame_torque_vector_(0.0, 0.0, 0.0) {}

    virtual ~GenericGazeboWrenchThrusterPlugin()
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
      // Listen to the update event.
      // This event is broadcast every simulation iteration.
      simulation_step_connection_ =
          event::Events::ConnectWorldUpdateBegin(
              boost::bind(&GenericGazeboWrenchThrusterPlugin::SimulationStep,
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
          ros::SubscribeOptions::create<geometry_msgs::WrenchStamped>(
              model_ptr_->GetName() + "/" + thruster_name_
              + "/command", 1,
              boost::bind(
                  &GenericGazeboWrenchThrusterPlugin::CommandMessageCallback,
                  this, _1),
              ros::VoidPtr(), &ros_callback_queue_);
      ros_thrust_command_sub_ = ros_node_ptr_->subscribe(subscribe_options);
      ros_thrust_status_pub_ =
          ros_node_ptr_->advertise<geometry_msgs::WrenchStamped>(
              model_ptr_->GetName() + "/" + thruster_name_ + "/status",
              1, true);
      // Spin up the queue helper thread.
      ros_callback_thread_ =
          std::thread(std::bind(
              &GenericGazeboWrenchThrusterPlugin::ROSCallbackThread, this));
    }

    // Called at each simulation step, this is where we apply the force
    void SimulationStep(const common::UpdateInfo&)
    {
      command_mutex_.lock();
      const ignition::math::Vector3d body_frame_force_vector =
          body_frame_force_vector_;
      const ignition::math::Vector3d body_frame_torque_vector =
          body_frame_torque_vector_;
      command_mutex_.unlock();
      // Get current body pose
      const ignition::math::Pose3d thruster_body_pose =
          thruster_body_->WorldCoGPose();
      // Compute the current thrust force vector
      const ignition::math::Vector3d world_frame_force_vector =
          thruster_body_pose.Rot().RotateVector(body_frame_force_vector);
      // Compute the current thrust torque vector
      const ignition::math::Vector3d world_frame_torque_vector =
          thruster_body_pose.Rot().RotateVector(body_frame_torque_vector);
      // Apply the thrust force
      thruster_body_->AddForce(world_frame_force_vector);
      // Apply the thrust torque
      thruster_body_->AddTorque(world_frame_torque_vector);
      // Update the status
      geometry_msgs::WrenchStamped thrust_msg;
      thrust_msg.header.stamp = ros::Time::now();
      thrust_msg.header.frame_id = thruster_body_->GetName();
      thrust_msg.wrench.force.x = body_frame_force_vector.X();
      thrust_msg.wrench.force.y = body_frame_force_vector.Y();
      thrust_msg.wrench.force.z = body_frame_force_vector.Z();
      thrust_msg.wrench.torque.x = body_frame_torque_vector.X();
      thrust_msg.wrench.torque.y = body_frame_torque_vector.Y();
      thrust_msg.wrench.torque.z = body_frame_torque_vector.Z();
      ros_thrust_status_pub_.publish(thrust_msg);
    }
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(GenericGazeboWrenchThrusterPlugin)
}

