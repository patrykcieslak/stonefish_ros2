//
//  SceneLaunchers.h
//  stonefish_ros2
//
//  Objects the vehicle lets go of, on a std_srvs/Trigger: torpedoes and markers.
//

#ifndef __Stonefish_SceneLaunchers__
#define __Stonefish_SceneLaunchers__

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <Stonefish/StonefishCommon.h>

class btRigidBody;

namespace sf
{
    class SimulationManager;

    //! <ros_launcher service="" robot="" objects="" count="" speed="" base_frame="" frame=""/>: a call on the
    //! service puts the next of the <dynamic> objects <objects>_0 .. <objects>_<count-1> at the TF frame, with
    //! speed along its X. base_frame is the robot's link in TF. The frame comes from the drone's description,
    //! like on the real drone. When the objects run out it starts over with the oldest, so it never runs out.
    class SceneLaunchers
    {
    public:
        SceneLaunchers(SimulationManager* sm, const std::shared_ptr<rclcpp::Node>& nh);

        void Add(const std::string& service, const std::string& robot, const std::string& objects,
                 int count, double speed, const std::string& baseFrame, const std::string& frame);
        void Clear();

        //! From the simulation thread, after each step.
        void Step();

    private:
        struct Body
        {
            btRigidBody* rigid = nullptr;
            Transform originToCg;
            Transform parked;
            bool inWater = false;
        };

        struct Launcher
        {
            std::string service;
            std::string robot;
            std::string objects;
            int count = 0;
            double speed = 0.0;
            std::string baseFrame;
            std::string frame;
            std::vector<Body> bodies;
            size_t next = 0;
            std::mutex mutex;
            std::vector<Transform> requested; //!< base_frame to frame, one per shot
            rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv;
        };

        bool Resolve(Launcher& l);

        SimulationManager* sm_;
        std::shared_ptr<rclcpp::Node> nh_;
        std::vector<std::unique_ptr<Launcher>> launchers_;
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;
    };
}

#endif
