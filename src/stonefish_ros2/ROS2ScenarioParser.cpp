/*    
    This file is a part of stonefish_ros2.

    stonefish_ros is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    stonefish_ros is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  ROS2ScenarioParser.cpp
//  stonefish_ros2
//
//  Created by Patryk Cieslak on 02/10/23.
//  Copyright (c) 2025-2026 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros2/ROS2ScenarioParser.h"
#include "stonefish_ros2/ROS2SimulationManager.h"
#include "stonefish_ros2/ROS2Interface.h"
#include "stonefish_ros2/ROS2SensorPluginInterface.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "stonefish_ros2/msg/int32_stamped.hpp"
#include "stonefish_ros2/msg/beacon_info.hpp"
#include "stonefish_ros2/msg/dvl.hpp"
#include "stonefish_ros2/msg/ins.hpp"
#include "stonefish_ros2/msg/thruster_state.hpp"
#include "stonefish_ros2/msg/debug_physics.hpp"
#include "stonefish_ros2/srv/sonar_settings.hpp"
#include "stonefish_ros2/srv/sonar_settings2.hpp"
#include "image_transport/image_transport.hpp"
#include "stonefish_ros2/msg/event.hpp"
#include "stonefish_ros2/msg/event_array.hpp"

#include <Stonefish/core/Robot.h>
#include <Stonefish/entities/AnimatedEntity.h>
#include <Stonefish/entities/animation/ManualTrajectory.h>
#include <Stonefish/entities/forcefields/Uniform.h>
#include <Stonefish/entities/forcefields/Jet.h>
#include <Stonefish/actuators/Actuator.h>
#include <Stonefish/actuators/Light.h>
#include <Stonefish/actuators/Motor.h>
#include <Stonefish/actuators/Servo.h>
#include <Stonefish/actuators/SimpleThruster.h>
#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/actuators/Propeller.h>
#include <Stonefish/actuators/Push.h>
#include <Stonefish/actuators/VariableBuoyancy.h>
#include <Stonefish/actuators/SuctionCup.h>
#include <Stonefish/sensors/ScalarSensor.h>
#include <Stonefish/sensors/vision/ColorCamera.h>
#include <Stonefish/sensors/vision/DepthCamera.h>
#include <Stonefish/sensors/vision/ThermalCamera.h>
#include <Stonefish/sensors/vision/OpticalFlowCamera.h>
#include <Stonefish/sensors/vision/SegmentationCamera.h>
#include <Stonefish/sensors/vision/EventBasedCamera.h>
#include <Stonefish/sensors/vision/Lidar.h>
#include <Stonefish/sensors/vision/FLS.h>
#include <Stonefish/sensors/vision/SSS.h>
#include <Stonefish/sensors/vision/MSIS.h>
#include <Stonefish/comms/Comm.h>
#include <Stonefish/joints/FixedJoint.h>
#include <Stonefish/utils/SystemUtil.hpp>

using namespace std::placeholders;

namespace sf
{

ROS2ScenarioParser::ROS2ScenarioParser(ROS2SimulationManager* sm, const std::shared_ptr<rclcpp::Node>& nh) : ScenarioParser(sm), nh_(nh)
{   
}

std::string ROS2ScenarioParser::SubstituteROSVars(const std::string& value)
{
    std::string replacedValue;

    size_t currentPos = 0;
    size_t startPos, endPos;
    while ((startPos = value.find("$(", currentPos)) != std::string::npos && (endPos = value.find(")", startPos+2)) != std::string::npos)
    {
        replacedValue += value.substr(currentPos, startPos - currentPos);

        std::string arguments = value.substr(startPos+2, endPos-startPos-2);
        std::istringstream iss(arguments);
        std::vector<std::string> results(std::istream_iterator<std::string>{iss},
                                         std::istream_iterator<std::string>());
        if (results.size() != 2)
        {
            log.Print(MessageType::ERROR, "[ROS] Argument substitution requires 2 values to be provided, got: '%s'!", arguments.c_str());
            RCLCPP_ERROR(nh_->get_logger(), "Scenario parser: Argument substitution failed '%s'!", arguments.c_str());
            continue;
        }

        if (results[0] == "find")
        {
            std::string packagePath = ament_index_cpp::get_package_share_directory(results[1]);
            if (packagePath.empty())
            {
                log.Print(MessageType::ERROR, "[ROS] Could not find package '%s'!", results[1].c_str());
                RCLCPP_ERROR(nh_->get_logger(), "Scenario parser: Could not find package '%s'!", results[1].c_str());
                return value;
            }
            replacedValue += packagePath;
        }
        else if (results[0] == "param")
        {
            if(!nh_->has_parameter(results[1]))
                nh_->declare_parameter(results[1], rclcpp::PARAMETER_STRING);
            try
            {
                auto param = nh_->get_parameter(results[1]);
                replacedValue += param.as_string();
            }
            catch(const rclcpp::exceptions::ParameterUninitializedException& e)
            {
                log.Print(MessageType::ERROR, "[ROS] Could not find parameter '%s'!", results[1].c_str());
                RCLCPP_ERROR(nh_->get_logger(), "Scenario parser: Could not find parameter '%s'!", results[1].c_str());
                return value;
            }
        }
        else //Command unsupported
        {
            return value;
        }

        currentPos = endPos + 1;
    }

    replacedValue += value.substr(currentPos, value.size() - currentPos);

    return replacedValue;
}

bool ROS2ScenarioParser::ReplaceROSVars(XMLNode* node)
{
    XMLElement* element = node->ToElement();
    if (element != nullptr)
    {
        for (const tinyxml2::XMLAttribute* attr = element->FirstAttribute(); attr != nullptr; attr = attr->Next())
        {
            std::string value = std::string(attr->Value());
            std::string substitutedValue = SubstituteROSVars(value);
            if (substitutedValue != value)
            {
                log.Print(MessageType::INFO, "[ROS] Replacing '%s' with '%s'.", value.c_str(), substitutedValue.c_str());
                element->SetAttribute(attr->Name(), substitutedValue.c_str());
            }
        }
    }

    for (tinyxml2::XMLNode* child = node->FirstChild(); child != nullptr; child = child->NextSibling())
    {
        if(!ReplaceROSVars(child))
            return false;
    }

    return true;
}

bool ROS2ScenarioParser::PreProcess(XMLNode* root, const std::map<std::string, std::string>& args)
{
    //First replace ROS variables to support them inside include arguments
    if(!ReplaceROSVars(root))
        return false;
    //Then process include arguments
    return ScenarioParser::PreProcess(root, args);
}

std::unique_ptr<VelocityField> ROS2ScenarioParser::ParseVelocityField(XMLElement* element)
{
    std::unique_ptr<VelocityField> vf = ScenarioParser::ParseVelocityField(element);
    if(vf != nullptr)
    {
        ROS2SimulationManager* sim = static_cast<ROS2SimulationManager*>(GetSimulationManager());
        std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr>& subs = sim->GetSubscribers();
        XMLElement* item;

        switch(vf->GetType())
        {
            case VelocityFieldType::UNIFORM:
            {
                const char* subTopic = nullptr;
                if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                   && item->QueryStringAttribute("velocity", &subTopic) == XML_SUCCESS)
                {
                    std::function<void(const geometry_msgs::msg::Vector3::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::UniformVFCallback, sim, _1, static_cast<Uniform*>(vf.get()));               
                    subs["vf"+std::to_string(rclcpp::Clock().now().nanoseconds())]  //Unique subscriber key string
                        = nh_->create_subscription<geometry_msgs::msg::Vector3>(std::string(subTopic), 10, callbackFunc);
                }
            }
                break;

            case VelocityFieldType::JET:
            {
                const char* subTopic = nullptr;
                if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                   && item->QueryStringAttribute("outlet_velocity", &subTopic) == XML_SUCCESS)
                {
                    std::function<void(const std_msgs::msg::Float64::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::JetVFCallback, sim, _1, static_cast<Jet*>(vf.get()));
                    subs["vf"+std::to_string(rclcpp::Clock().now().nanoseconds())]  //Unique subscriber key string
                        = nh_->create_subscription<std_msgs::msg::Float64>(std::string(subTopic), 10, callbackFunc);
                }
            }
                break;

            default:
                break;
        }
    }
    return vf;
}

bool ROS2ScenarioParser::ParseRobot(XMLElement* element)
{
    if(!ScenarioParser::ParseRobot(element))
        return false;

    ROS2SimulationManager* sim = static_cast<ROS2SimulationManager*>(GetSimulationManager());
    std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs = sim->GetPublishers();
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr>& subs = sim->GetSubscribers();
    
    //Robot info
    const char* name = nullptr;
    element->QueryStringAttribute("name", &name);
    std::string nameStr(name);
    Robot* robot = GetSimulationManager()->GetRobot(nameStr);

    //Check actuators
    unsigned int nThrusters = 0;
    unsigned int nPropellers = 0;
    unsigned int nServos = 0;
    unsigned int nMotors = 0;
    unsigned int nRudders = 0;

    unsigned int aID = 0;
    Actuator* act;

    while((act = robot->GetActuator(aID++)) != NULL)
    {
        if (act->GetType() == ActuatorType::LINK)
        {
            LinkActuator* linkAct = static_cast<LinkActuator*>(act);
            switch(linkAct->GetLinkActuatorType())
            {
                case LinkActuatorType::SIMPLE_THRUSTER:
                case LinkActuatorType::THRUSTER:
                    ++nThrusters;
                    break;

                case LinkActuatorType::PROPELLER:
                    ++nPropellers;
                    break;

                case LinkActuatorType::RUDDER:
                    ++nRudders;
                    break;

                default:
                    break;
            }
        }
        else if (act->GetType() == ActuatorType::JOINT)
        {
            JointActuator* jointAct = static_cast<JointActuator*>(act);
            switch(jointAct->GetJointActuatorType())
            {
                case JointActuatorType::SERVO:
                    ++nServos;
                    break;

                case JointActuatorType::MOTOR:
                    ++nMotors;
                    break;

                default:
                    break;
            }
        }
    }

    std::unique_ptr<ROS2Robot> ros2Robot = std::make_unique<ROS2Robot>(robot, nThrusters, nPropellers, nRudders);

    //Check if we should publish world_ned -> base_link transform
    XMLElement* item;
    if((item = element->FirstChildElement("ros_base_link_transform")) != nullptr)
        item->QueryBoolAttribute("publish", &ros2Robot->publishBaseLinkTransform_);

    //Check if we should publish debug information
    if((item = element->FirstChildElement("ros_debug")) != nullptr)
    {
        const char* topicPhysics = nullptr;
        if(item->QueryStringAttribute("physics", &topicPhysics) == XML_SUCCESS)
            pubs[robot->GetName() + "/debug/physics"] = nh_->create_publisher<stonefish_ros2::msg::DebugPhysics>(std::string(topicPhysics), 10);
    }

    //Generate subscribers
    if((item = element->FirstChildElement("ros_subscriber")) != nullptr)
    {
        const char* topicThrust = nullptr;
        const char* topicProp = nullptr;
        const char* topicRudder = nullptr;
        const char* topicSrv = nullptr;

        if(nThrusters > 0 && item->QueryStringAttribute("thrusters", &topicThrust) == XML_SUCCESS)
        {
            std::function<void(const std_msgs::msg::Float64MultiArray::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::ThrustersCallback, sim, _1, ros2Robot.get());
            subs[robot->GetName() + "/thrusters"] = nh_->create_subscription<std_msgs::msg::Float64MultiArray>(std::string(topicThrust), 10, callbackFunc);
        }

        if(nPropellers > 0 && item->QueryStringAttribute("propellers", &topicProp) == XML_SUCCESS)
        {
            std::function<void(const std_msgs::msg::Float64MultiArray::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::PropellersCallback, sim, _1, ros2Robot.get());
            subs[robot->GetName() + "/propellers"] = nh_->create_subscription<std_msgs::msg::Float64MultiArray>(std::string(topicProp), 10, callbackFunc);
        }

        if(nRudders > 0 && item->QueryStringAttribute("rudders", &topicRudder) == XML_SUCCESS)
        {
            std::function<void(const std_msgs::msg::Float64MultiArray::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::RuddersCallback, sim, _1, ros2Robot.get());
            subs[robot->GetName() + "/rudders"] = nh_->create_subscription<std_msgs::msg::Float64MultiArray>(std::string(topicRudder), 10, callbackFunc);
        }

        if(nServos > 0 && item->QueryStringAttribute("servos", &topicSrv) == XML_SUCCESS)
        {
            std::function<void(const sensor_msgs::msg::JointState::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::ServosCallback, sim, _1, ros2Robot.get());
            subs[robot->GetName() + "/servos"] = nh_->create_subscription<sensor_msgs::msg::JointState>(std::string(topicSrv), 10, callbackFunc);
            //Generate setpoint placeholders for the whole system
            aID = 0;
            while((act = robot->GetActuator(aID++)) != nullptr && act->GetType() == ActuatorType::JOINT)
            {
                JointActuator* jointAct = static_cast<JointActuator*>(act);
                if (jointAct->GetJointActuatorType() == JointActuatorType::SERVO)
                    ros2Robot->servoSetpoints_[(static_cast<Servo*>(jointAct))->GetJointName()] = std::pair(ServoControlMode::VELOCITY, Scalar(0));
            }
        }
    }

    //Parse all defined joint groups, single joint subscribers and ros control interfaces
    if(nServos > 0 || nMotors > 0)
    {
        //Joint group subscribers
        const char* jgTopic = nullptr;
        const char* jgMode = nullptr;
        unsigned int jg = 0;
        ServoControlMode mode;

        for(item = element->FirstChildElement("ros_joint_group_subscriber"); item != nullptr; item = item->NextSiblingElement("ros_joint_group_subscriber"))
        {
            if(item->QueryStringAttribute("topic", &jgTopic) == XML_SUCCESS && item->QueryStringAttribute("control_mode", &jgMode) == XML_SUCCESS)
            {
                std::string modeStr(jgMode);
                if(modeStr == "velocity")
                    mode = ServoControlMode::VELOCITY;
                else if(modeStr == "position")
                    mode = ServoControlMode::POSITION;
                else if(modeStr == "torque")
                    mode = ServoControlMode::TORQUE;
                else
                    continue; //Skip joint group -> missing parameters
            }
            else
                continue;

            std::vector<std::string> jointNames;
            for(XMLElement* joint = item->FirstChildElement("joint"); joint != nullptr; joint = joint->NextSiblingElement("joint"))
            {
                const char* jgJointName = nullptr;
                if(joint->QueryStringAttribute("name", &jgJointName) == XML_SUCCESS)
                    jointNames.push_back(robot->GetName() + "/" + std::string(jgJointName));
            }

            if(jointNames.size() > 0) // Any joints defined?
            {
                RCLCPP_INFO_STREAM(nh_->get_logger(), "Creating joint group subscriber (" << std::string(jgMode) << ") " << std::string(jgTopic));
                for(size_t i=0; i<jointNames.size(); ++i)
                    ros2Robot->servoSetpoints_[jointNames[i]] = std::make_pair(mode, Scalar(0));
                std::function<void(const std_msgs::msg::Float64MultiArray::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::JointGroupCallback, sim, _1, ros2Robot.get(), mode, jointNames);
                subs[robot->GetName() + "/joint_group" + std::to_string(jg)] = nh_->create_subscription<std_msgs::msg::Float64MultiArray>(std::string(jgTopic), 10, callbackFunc);
                ++jg;
            }
        }

        //Single joint subscribers
        const char* jointName = nullptr;
        jg = 0;
        for(item = element->FirstChildElement("ros_joint_subscriber"); item != nullptr; item = item->NextSiblingElement("ros_joint_subscriber"))
        {
            if(item->QueryStringAttribute("joint_name", &jointName) == XML_SUCCESS
               && item->QueryStringAttribute("topic", &jgTopic) == XML_SUCCESS
               && item->QueryStringAttribute("control_mode", &jgMode) == XML_SUCCESS)
            {
                std::string modeStr(jgMode);
                if(modeStr == "velocity")
                    mode = ServoControlMode::VELOCITY;
                else if(modeStr == "position")
                    mode = ServoControlMode::POSITION;
                else if(modeStr == "torque")
                    mode = ServoControlMode::TORQUE;
                else    
                    continue; //Skip joint group -> missing parameters
                
                RCLCPP_INFO_STREAM(nh_->get_logger(), "Creating joint subscriber (" << modeStr << ") " << std::string(jgTopic));
                std::string jointNameStr = robot->GetName() + "/" + std::string(jointName);
                ros2Robot->servoSetpoints_[jointNameStr] = std::make_pair(mode, Scalar(0));
                std::function<void(const std_msgs::msg::Float64::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::JointCallback, sim, _1, ros2Robot.get(), mode, jointNameStr);
                subs[robot->GetName() + "/joint" + std::to_string(jg)] = nh_->create_subscription<std_msgs::msg::Float64>(std::string(jgTopic), 10, callbackFunc);
                ++jg;
            }
        }
    }

    //Generate publishers
    if((item = element->FirstChildElement("ros_publisher")) != nullptr)
    {
        const char* topicThr = nullptr;
        const char* topicSrv = nullptr;
        const char* topicMtr = nullptr;
        const char* topicRud = nullptr;

        if(nThrusters > 0 && item->QueryStringAttribute("thrusters", &topicThr) == XML_SUCCESS)
            pubs[robot->GetName() + "/thrusters"] = nh_->create_publisher<stonefish_ros2::msg::ThrusterState>(std::string(topicThr), 10);

        if(nServos > 0 && item->QueryStringAttribute("servos", &topicSrv) == XML_SUCCESS)
            pubs[robot->GetName() + "/servos"] = nh_->create_publisher<sensor_msgs::msg::JointState>(std::string(topicSrv), 10);

        if(nMotors > 0 && item->QueryStringAttribute("motors", &topicMtr) == XML_SUCCESS)
            pubs[robot->GetName() + "/motors"] = nh_->create_publisher<sensor_msgs::msg::JointState>(std::string(topicMtr), 10);

        if(nRudders > 0 && item->QueryStringAttribute("rudders", &topicRud) == XML_SUCCESS)
            pubs[robot->GetName() + "/rudders"] = nh_->create_publisher<sensor_msgs::msg::JointState>(std::string(topicRud), 10);
    }

    //Save robot
    sim->AddROS2Robot(std::move(ros2Robot));

    return true;
}

bool ROS2ScenarioParser::ParseAnimated(XMLElement* element)
{
    if(!ScenarioParser::ParseAnimated(element))
        return false;

    ROS2SimulationManager* sim = static_cast<ROS2SimulationManager*>(GetSimulationManager());
    std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs = sim->GetPublishers();
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr>& subs = sim->GetSubscribers();

    //Get name
    const char* name = nullptr;
    element->QueryStringAttribute("name", &name);
    std::string nameStr(name);

    //Get type of trajectory
    const char* type = "";
    XMLElement* item = element->FirstChildElement("trajectory");
    item->QueryStringAttribute("type", &type);
    std::string typeStr(type);

    if(typeStr == "manual") //Position of animated body set by a message
    {
        //Get topic group name
        const char* topic = nullptr;
        if((item = element->FirstChildElement("ros_subscriber")) == nullptr
            || item->QueryStringAttribute("topic", &topic) != XML_SUCCESS)
        {
            return true;
        }
        AnimatedEntity* anim = static_cast<AnimatedEntity*>(sim->GetEntity(nameStr));
        if(anim != nullptr)
        {
            std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> callbackFunc =
                std::bind(&ROS2SimulationManager::TrajectoryCallback, sim, _1, static_cast<ManualTrajectory*>(anim->GetTrajectory()));
            subs[nameStr + "/odometry"] = nh_->create_subscription<nav_msgs::msg::Odometry>(std::string(topic), 10, callbackFunc);
        }
    }
    else //State of the trajectory published with a message
    {
        //Get topic group name
        const char* topic = nullptr;
        if((item = element->FirstChildElement("ros_publisher")) == nullptr
            || item->QueryStringAttribute("topic", &topic) != XML_SUCCESS)
        {
            return true;
        }
        pubs[nameStr + "/odometry"] = nh_->create_publisher<nav_msgs::msg::Odometry>(std::string(topic), 10);
        pubs[nameStr + "/iteration"] = nh_->create_publisher<stonefish_ros2::msg::Int32Stamped>(std::string(topic) + "/iteration", 10);
    }

    return true;
}

std::unique_ptr<Actuator, ActuatorDeleter> ROS2ScenarioParser::ParseActuator(XMLElement* element, std::string_view namePrefix)
{
    std::unique_ptr<Actuator, ActuatorDeleter> act = ScenarioParser::ParseActuator(element, namePrefix);
    
    if(act != nullptr && act->GetType() == ActuatorType::LINK)
    {
        ROS2SimulationManager* sim = static_cast<ROS2SimulationManager*>(GetSimulationManager());
        std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs = sim->GetPublishers();
        std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr>& subs = sim->GetSubscribers();
        std::unordered_map<std::string, rclcpp::ServiceBase::SharedPtr>& srvs = sim->GetServices();
        std::string actuatorName = act->GetName();
        LinkActuator* linkAct = static_cast<LinkActuator*>(act.get());
        XMLElement* item;

        switch(linkAct->GetLinkActuatorType())
        {
            case LinkActuatorType::SIMPLE_THRUSTER:
            {
                const char* subTopic = nullptr;
                if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                    && item->QueryStringAttribute("topic", &subTopic) == XML_SUCCESS)
                {
                    std::function<void(const std_msgs::msg::Float64::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::SimpleThrusterCallback, sim, _1, static_cast<SimpleThruster*>(linkAct));
                    subs[actuatorName] = nh_->create_subscription<std_msgs::msg::Float64>(std::string(subTopic), 10, callbackFunc);
                }

                const char* pubTopic = nullptr;
                if((item = element->FirstChildElement("ros_publisher")) != nullptr)
                {
                    if(item->QueryStringAttribute("wrench_topic", &pubTopic) == XML_SUCCESS)
                    {
                        pubs[actuatorName+"/wrench"] = nh_->create_publisher<geometry_msgs::msg::WrenchStamped>(std::string(pubTopic), 10);
                    }
                    if(item->QueryStringAttribute("joint_state_topic", &pubTopic) == XML_SUCCESS)
                    {
                        pubs[actuatorName+"/joint_state"] = nh_->create_publisher<sensor_msgs::msg::JointState>(std::string(pubTopic), 10);
                    }
                }
            }
                break;
            case LinkActuatorType::THRUSTER:
            {
                const char* subTopic = nullptr;
                if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                    && item->QueryStringAttribute("topic", &subTopic) == XML_SUCCESS)
                {
                    std::function<void(const std_msgs::msg::Float64::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::ThrusterCallback, sim, _1, static_cast<Thruster*>(linkAct));     
                    subs[actuatorName] = nh_->create_subscription<std_msgs::msg::Float64>(std::string(subTopic), 10, callbackFunc);
                }

                const char* pubTopic = nullptr;
                if((item = element->FirstChildElement("ros_publisher")) != nullptr)
                {
                    if(item->QueryStringAttribute("wrench_topic", &pubTopic) == XML_SUCCESS)
                    {
                        pubs[actuatorName+"/wrench"] = nh_->create_publisher<geometry_msgs::msg::WrenchStamped>(std::string(pubTopic), 10);
                    }
                    if(item->QueryStringAttribute("joint_state_topic", &pubTopic) == XML_SUCCESS)
                    {
                        pubs[actuatorName+"/joint_state"] = nh_->create_publisher<sensor_msgs::msg::JointState>(std::string(pubTopic), 10);
                    }
                }
            }
                break;
            case LinkActuatorType::PROPELLER:
            {
                const char* subTopic = nullptr;
                if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                    && item->QueryStringAttribute("topic", &subTopic) == XML_SUCCESS)
                {
                    std::function<void(const std_msgs::msg::Float64::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::PropellerCallback, sim, _1, static_cast<Propeller*>(linkAct));     
                    subs[actuatorName] = nh_->create_subscription<std_msgs::msg::Float64>(std::string(subTopic), 10, callbackFunc);
                }

                const char* pubTopic = nullptr;
                if((item = element->FirstChildElement("ros_publisher")) != nullptr
                    && item->QueryStringAttribute("topic", &pubTopic) == XML_SUCCESS)
                {
                    pubs[actuatorName] = nh_->create_publisher<geometry_msgs::msg::WrenchStamped>(std::string(pubTopic), 10);
                }
            }
                break;
            case LinkActuatorType::PUSH:
            {
                const char* subTopic = nullptr;
                if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                    && item->QueryStringAttribute("topic", &subTopic) == XML_SUCCESS)
                {
                    std::function<void(const std_msgs::msg::Float64::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::PushCallback, sim, _1, static_cast<Push*>(linkAct));     
                    subs[actuatorName] = nh_->create_subscription<std_msgs::msg::Float64>(std::string(subTopic), 10, callbackFunc);
                }

                const char* pubTopic = nullptr;
                if((item = element->FirstChildElement("ros_publisher")) != nullptr
                    && item->QueryStringAttribute("topic", &pubTopic) == XML_SUCCESS)
                {
                    pubs[actuatorName] = nh_->create_publisher<geometry_msgs::msg::WrenchStamped>(std::string(pubTopic), 10);
                }
            }
                break;
            case LinkActuatorType::VBS:
            {
                const char* pubTopic = nullptr;
                const char* subTopic = nullptr;
                if((item = element->FirstChildElement("ros_publisher")) != nullptr
                    && item->QueryStringAttribute("topic", &pubTopic) == XML_SUCCESS)
                {
                    pubs[actuatorName] = nh_->create_publisher<std_msgs::msg::Float64>(std::string(pubTopic), 10);
                }
                if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                    && item->QueryStringAttribute("topic", &subTopic) == XML_SUCCESS)
                {
                    std::function<void(const std_msgs::msg::Float64::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::VBSCallback, sim, _1, static_cast<VariableBuoyancy*>(linkAct));     
                    subs[actuatorName] = nh_->create_subscription<std_msgs::msg::Float64>(std::string(subTopic), 10, callbackFunc);
                }
            }
                break;
            case LinkActuatorType::SUCTION_CUP:
            {
                const char* pubTopic = nullptr;
                const char* srvTopic = nullptr;
                if((item = element->FirstChildElement("ros_publisher")) != nullptr
                    && item->QueryStringAttribute("topic", &pubTopic) == XML_SUCCESS)
                {
                    pubs[actuatorName] = nh_->create_publisher<std_msgs::msg::Bool>(std::string(pubTopic), 10);
                }
                if((item = element->FirstChildElement("ros_service")) != nullptr
                    && item->QueryStringAttribute("topic", &srvTopic) == XML_SUCCESS)
                {
                    std::function<void(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res)> callbackFunc =
                        std::bind(&ROS2SimulationManager::SuctionCupService, sim, _1, _2, static_cast<SuctionCup*>(linkAct));
                    srvs[actuatorName] = nh_->create_service<std_srvs::srv::SetBool>(std::string(srvTopic), callbackFunc);
                }
            }
                break;
            default:
                return act;
        }
        
        //Handling of online origin updates
        switch(linkAct->GetLinkActuatorType())
        {
            case LinkActuatorType::PUSH:
            case LinkActuatorType::SIMPLE_THRUSTER:
            case LinkActuatorType::THRUSTER:
            case LinkActuatorType::PROPELLER:
            case LinkActuatorType::VBS:
            {
                const char* originTopic = nullptr;
                if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                    && item->QueryStringAttribute("origin", &originTopic) == XML_SUCCESS)
                {
                    std::function<void(const geometry_msgs::msg::Transform::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::ActuatorOriginCallback, sim, _1, linkAct);     
                    subs[actuatorName] = nh_->create_subscription<geometry_msgs::msg::Transform>(std::string(originTopic), 10, callbackFunc);
                }
            }
                break;

            default:
                break;
        }
    }
    
    return act;
}

std::unique_ptr<Sensor, SensorDeleter> ROS2ScenarioParser::ParseSensor(XMLElement* element, std::string_view namePrefix)
{
    std::unique_ptr<Sensor, SensorDeleter> sens = ScenarioParser::ParseSensor(element, namePrefix);
    
    if(sens != nullptr)
    {
        ROS2SimulationManager* sim = static_cast<ROS2SimulationManager*>(GetSimulationManager());
        std::shared_ptr<image_transport::ImageTransport> imgTrans = sim->GetImageTransportHandle();
        std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs = sim->GetPublishers();
        std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr>& subs = sim->GetSubscribers();
        std::unordered_map<std::string, rclcpp::ServiceBase::SharedPtr>& srvs = sim->GetServices();
        std::unordered_map<std::string, image_transport::Publisher>& imgPubs = sim->GetImagePublishers();
        std::unordered_map<std::string, std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr>>& camMsgProto = sim->GetCameraMsgPrototypes();
        std::unordered_map<std::string, std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr, sensor_msgs::msg::Image::SharedPtr>>& dualCamMsgProto = sim->GetDualImageCameraMsgPrototypes();
        std::unordered_map<std::string, std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::Image::SharedPtr>>& sonarMsgProto = sim->GetSonarMsgPrototypes();
        
        //Publishing info
        std::string sensorName = sens->GetName();
        XMLElement* item;
        std::string frameIdStr = "";

        //Standard sensor publisher
        std::unordered_map<std::string, std::string> pubTopics;
        if((item = element->FirstChildElement("ros_publisher")) != nullptr)
        {
            for (const XMLAttribute* attr = item->FirstAttribute(); attr != nullptr; attr = attr->Next()) 
            {
                std::string name(attr->Name());
                std::string value(attr->Value());
                if (value != "")
                {
                    if (name == "frame_id") 
                        frameIdStr = std::string(attr->Value());
                    else 
                        pubTopics[name] = std::string(attr->Value());
                }
            }
        }    
        else
        {
            return sens;
        }

        if (pubTopics.empty())
            return sens;

        //Sensor enable/disable service
        const char* serviceTopic = nullptr;
        XMLElement* item2; // item is used below so we need a new pointer
        if((item2 = element->FirstChildElement("ros_service")) != nullptr
            && item2->QueryStringAttribute("set_enabled", &serviceTopic) == XML_SUCCESS)
        {
            std::string serviceTopicStr(serviceTopic);
            std::function<void(const std_srvs::srv::SetBool::Request::SharedPtr req, 
                            std_srvs::srv::SetBool::Response::SharedPtr res)> callbackFunc =
                            std::bind(&ROS2SimulationManager::SensorService, sim, _1, _2, sens.get());
            srvs[sensorName+"/set_enabled"] = nh_->create_service<std_srvs::srv::SetBool>(serviceTopicStr, callbackFunc);
            sens->SetEnabled(false); // Sensor with an enable service starts as disabled
        }

        unsigned int queueSize = (unsigned int)ceil(sens->GetUpdateFrequency());

        //Generate publishers for different sensor types
        switch(sens->GetType())
        {
            case SensorType::JOINT:
            {
                switch(static_cast<ScalarSensor*>(sens.get())->GetScalarSensorType())
                {
                    case ScalarSensorType::FT:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                            pubs[sensorName] = nh_->create_publisher<geometry_msgs::msg::WrenchStamped>(it->second, queueSize);
                    }
                        break;

                    case ScalarSensorType::ENCODER:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                            pubs[sensorName] = nh_->create_publisher<sensor_msgs::msg::JointState>(it->second, queueSize);
                    }
                        break;

                    case ScalarSensorType::PLUGIN:
                    {
                        ROS2SensorPluginInterface* iface = dynamic_cast<ROS2SensorPluginInterface*>(sens.get());
                        if (iface != nullptr)
                            iface->CreatePublishers(nh_, pubs, pubTopics);
                    }
                        break;

                    default:
                        log.Print(MessageType::ERROR, "[ROS] Sensor '%s' not supported!", sensorName.c_str());
                        RCLCPP_ERROR(nh_->get_logger(), "Scenario parser: Sensor '%s' not supported!", sensorName.c_str());
                        break;
                }
            }
                break;

            case SensorType::LINK:
            {
                switch(static_cast<ScalarSensor*>(sens.get())->GetScalarSensorType())
                {
                    case ScalarSensorType::ACC:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                            pubs[sensorName] = nh_->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(it->second, queueSize);
                    }
                        break;

                    case ScalarSensorType::GYRO:
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                            pubs[sensorName] = nh_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(it->second, queueSize);
                        break;

                    case ScalarSensorType::IMU:
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                            pubs[sensorName] = nh_->create_publisher<sensor_msgs::msg::Imu>(it->second, queueSize);
                        break;

                    case ScalarSensorType::DVL:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                            pubs[sensorName] = nh_->create_publisher<stonefish_ros2::msg::DVL>(it->second, queueSize);
                        if (auto it = pubTopics.find("altitude_topic"); it != pubTopics.end())
                            pubs[sensorName + "/altitude"] = nh_->create_publisher<sensor_msgs::msg::Range>(it->second, queueSize);
                    }
                        break;

                    case ScalarSensorType::GPS:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                            pubs[sensorName] = nh_->create_publisher<sensor_msgs::msg::NavSatFix>(it->second, queueSize);
                    }
                        break;

                    case ScalarSensorType::PRESSURE:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                            pubs[sensorName] = nh_->create_publisher<sensor_msgs::msg::FluidPressure>(it->second, queueSize);
                    }
                        break;

                    case ScalarSensorType::INS:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                            pubs[sensorName] = nh_->create_publisher<stonefish_ros2::msg::INS>(it->second, queueSize);
                        if (auto it = pubTopics.find("odometry_topic"); it != pubTopics.end())
                            pubs[sensorName + "/odometry"] = nh_->create_publisher<nav_msgs::msg::Odometry>(it->second, queueSize);
                    }
                        break;

                    case ScalarSensorType::ODOM:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                            pubs[sensorName] = nh_->create_publisher<nav_msgs::msg::Odometry>(it->second, queueSize);
                    }
                        break;

                    case ScalarSensorType::MULTIBEAM:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                            pubs[sensorName] = nh_->create_publisher<sensor_msgs::msg::LaserScan>(it->second, queueSize);
                        if (auto it = pubTopics.find("pcl_topic"); it != pubTopics.end())
                            pubs[sensorName + "/pcl"] = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(it->second, queueSize);
                    }
                        break;

                    case ScalarSensorType::PROFILER:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                            pubs[sensorName] = nh_->create_publisher<sensor_msgs::msg::LaserScan>(it->second, queueSize);
                    }
                        break;

                    case ScalarSensorType::PLUGIN:
                    {
                        ROS2SensorPluginInterface* iface = dynamic_cast<ROS2SensorPluginInterface*>(sens.get());
                        if (iface != nullptr)
                            iface->CreatePublishers(nh_, pubs, pubTopics);
                    }
                        break;

                    default:
                        log.Print(MessageType::ERROR, "[ROS] Sensor '%s' not supported!", sensorName.c_str());
                        RCLCPP_ERROR(nh_->get_logger(), "Scenario parser: Sensor '%s' not supported!", sensorName.c_str());
                        break;
                }
                //Origin frame updates
                const char* originTopic = nullptr;
                if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                    && item->QueryStringAttribute("origin", &originTopic) == XML_SUCCESS)
                {
                    std::function<void(const geometry_msgs::msg::Transform::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::SensorOriginCallback, sim, _1, sens.get());
                    subs[sensorName] = nh_->create_subscription<geometry_msgs::msg::Transform>(std::string(originTopic), queueSize, callbackFunc);
                }
            }
                break;

            case SensorType::VISION:
            {
                switch(static_cast<VisionSensor*>(sens.get())->GetVisionSensorType())
                {
                    case VisionSensorType::COLOR_CAMERA:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        {
                            imgPubs[sensorName] = imgTrans->advertise(it->second + "/image_color", queueSize);
                            pubs[sensorName + "/info"] = nh_->create_publisher<sensor_msgs::msg::CameraInfo>(it->second + "/camera_info", queueSize);
                            ColorCamera* cam = static_cast<ColorCamera*>(sens.get());
                            cam->InstallNewDataHandler(std::bind(&ROS2SimulationManager::ColorCameraImageReady, sim, _1));
                            camMsgProto[sensorName] = ROS2Interface::GenerateCameraMsgPrototypes(cam, false, frameIdStr);
                        }
                    }
                        break;

                    case VisionSensorType::DEPTH_CAMERA:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        {
                            imgPubs[sensorName] = imgTrans->advertise(it->second + "/image_depth", queueSize);
                            pubs[sensorName + "/info"] = nh_->create_publisher<sensor_msgs::msg::CameraInfo>(it->second + "/camera_info", queueSize);
                            DepthCamera* cam = static_cast<DepthCamera*>(sens.get());
                            cam->InstallNewDataHandler(std::bind(&ROS2SimulationManager::DepthCameraImageReady, sim, _1));
                            camMsgProto[sensorName] = ROS2Interface::GenerateCameraMsgPrototypes(cam, true, frameIdStr);
                        }
                    }
                        break;

                    case VisionSensorType::THERMAL_CAMERA:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        {
                            imgPubs[sensorName] = imgTrans->advertise(it->second + "/image_raw", queueSize);
                            imgPubs[sensorName + "/display"] = imgTrans->advertise(it->second + "/image_color", queueSize);
                            pubs[sensorName + "/info"] = nh_->create_publisher<sensor_msgs::msg::CameraInfo>(it->second + "/camera_info", queueSize);                        
                            ThermalCamera* cam = static_cast<ThermalCamera*>(sens.get());
                            cam->InstallNewDataHandler(std::bind(&ROS2SimulationManager::ThermalCameraImageReady, sim, _1));
                            dualCamMsgProto[sensorName] = ROS2Interface::GenerateThermalCameraMsgPrototypes(cam);
                        }
                    }
                        break;

                    case VisionSensorType::OPTICAL_FLOW_CAMERA:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        {
                            imgPubs[sensorName] = imgTrans->advertise(it->second + "/image_raw", queueSize);
                            imgPubs[sensorName + "/display"] = imgTrans->advertise(it->second + "/image_color", queueSize);
                            pubs[sensorName + "/info"] = nh_->create_publisher<sensor_msgs::msg::CameraInfo>(it->second + "/camera_info", queueSize);                        
                            OpticalFlowCamera* cam = static_cast<OpticalFlowCamera*>(sens.get());
                            cam->InstallNewDataHandler(std::bind(&ROS2SimulationManager::OpticalFlowCameraImageReady, sim, _1));
                            dualCamMsgProto[sensorName] = ROS2Interface::GenerateOpticalFlowCameraMsgPrototypes(cam);
                        }
                    }
                        break;

                    case VisionSensorType::SEGMENTATION_CAMERA:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        {
                            imgPubs[sensorName] = imgTrans->advertise(it->second + "/image_raw", queueSize);
                            imgPubs[sensorName + "/display"] = imgTrans->advertise(it->second + "/image_color", queueSize);
                            pubs[sensorName + "/info"] = nh_->create_publisher<sensor_msgs::msg::CameraInfo>(it->second + "/camera_info", queueSize);                        
                            SegmentationCamera* cam = static_cast<SegmentationCamera*>(sens.get());
                            cam->InstallNewDataHandler(std::bind(&ROS2SimulationManager::SegmentationCameraImageReady, sim, _1));
                            dualCamMsgProto[sensorName] = ROS2Interface::GenerateSegmentationCameraMsgPrototypes(cam);
                        }
                    }
                        break;

                    case VisionSensorType::EVENT_BASED_CAMERA:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        {
                            pubs[sensorName] = nh_->create_publisher<stonefish_ros2::msg::EventArray>(it->second, queueSize);
                            EventBasedCamera* cam = static_cast<EventBasedCamera*>(sens.get());
                            cam->InstallNewDataHandler(std::bind(&ROS2SimulationManager::EventBasedCameraOutputReady, sim, _1));
                        }
                    }
                        break;

                    case VisionSensorType::LIDAR:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        {
                            pubs[sensorName] = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(it->second, queueSize);
                            Lidar* mb = static_cast<Lidar*>(sens.get());
                            mb->InstallNewDataHandler(std::bind(&ROS2SimulationManager::LidarScanReady, sim, _1));
                        }
                    }
                        break;

                    case VisionSensorType::FLS:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        {
                            FLS* fls = static_cast<FLS*>(sens.get());
                            fls->InstallNewDataHandler(std::bind(&ROS2SimulationManager::FLSScanReady, sim, _1));
                            sonarMsgProto[sensorName] = ROS2Interface::GenerateFLSMsgPrototypes(fls);
                            std::function<void(const stonefish_ros2::srv::SonarSettings::Request::SharedPtr req, 
                                stonefish_ros2::srv::SonarSettings::Response::SharedPtr res)> callbackFunc =
                            std::bind(&ROS2SimulationManager::FLSService, sim, _1, _2, fls);
                            srvs[sensorName] = nh_->create_service<stonefish_ros2::srv::SonarSettings>(it->second + "/settings", callbackFunc);
                            imgPubs[sensorName] = imgTrans->advertise(it->second + "/image", queueSize);
                            imgPubs[sensorName + "/display"] = imgTrans->advertise(it->second + "/display", queueSize);
                        }
                    }
                        break;

                    case VisionSensorType::SSS:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        {
                            SSS* sss = static_cast<SSS*>(sens.get());
                            sss->InstallNewDataHandler(std::bind(&ROS2SimulationManager::SSSScanReady, sim, _1));
                            sonarMsgProto[sensorName] = ROS2Interface::GenerateSSSMsgPrototypes(sss);
                            std::function<void(const stonefish_ros2::srv::SonarSettings::Request::SharedPtr req, 
                                stonefish_ros2::srv::SonarSettings::Response::SharedPtr res)> callbackFunc =
                            std::bind(&ROS2SimulationManager::SSSService, sim, _1, _2, sss);
                            srvs[sensorName] = nh_->create_service<stonefish_ros2::srv::SonarSettings>(it->second + "/settings", callbackFunc);
                            imgPubs[sensorName] = imgTrans->advertise(it->second + "/image", queueSize);
                            imgPubs[sensorName + "/display"] = imgTrans->advertise(it->second + "/display", queueSize);
                        }
                    }
                        break;

                    case VisionSensorType::MSIS:
                    {
                        if (auto it = pubTopics.find("topic"); it != pubTopics.end())
                        {
                            MSIS* msis = static_cast<MSIS*>(sens.get());
                            msis->InstallNewDataHandler(std::bind(&ROS2SimulationManager::MSISScanReady, sim, _1));
                            sonarMsgProto[sensorName] = ROS2Interface::GenerateMSISMsgPrototypes(msis);
                            std::function<void(const stonefish_ros2::srv::SonarSettings2::Request::SharedPtr req, 
                                stonefish_ros2::srv::SonarSettings2::Response::SharedPtr res)> callbackFunc =
                            std::bind(&ROS2SimulationManager::MSISService, sim, _1, _2, msis);                    
                            srvs[sensorName] = nh_->create_service<stonefish_ros2::srv::SonarSettings2>(it->second + "/settings", callbackFunc);
                            imgPubs[sensorName] = imgTrans->advertise(it->second + "/image", queueSize);
                            imgPubs[sensorName + "/display"] = imgTrans->advertise(it->second + "/display", queueSize);
                            pubs[sensorName + "/beam"] = nh_->create_publisher<sensor_msgs::msg::LaserScan>(it->second + "/last_beam", queueSize);
                        }
                    }
                        break;

                    default:
                        log.Print(MessageType::ERROR, "[ROS] Sensor '%s' not supported!", sensorName.c_str());
                        RCLCPP_ERROR(nh_->get_logger(), "Scenario parser: Sensor '%s' not supported!", sensorName.c_str());
                        break;
                }
                //Origin frame updates
                const char* originTopic = nullptr;
                if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                    && item->QueryStringAttribute("origin", &originTopic) == XML_SUCCESS)
                {
                    std::function<void(const geometry_msgs::msg::Transform::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::SensorOriginCallback, sim, _1, sens.get());
                    subs[sensorName] = nh_->create_subscription<geometry_msgs::msg::Transform>(std::string(originTopic), queueSize, callbackFunc);
                }
            }
                break;

            default:
                log.Print(MessageType::ERROR, "[ROS] Sensor '%s' not supported!", sensorName.c_str());
                RCLCPP_ERROR(nh_->get_logger(), "Scenario parser: Sensor '%s' not supported!", sensorName.c_str());
                break;
        }
    }
    return sens;
}

std::unique_ptr<Comm, CommDeleter> ROS2ScenarioParser::ParseComm(XMLElement* element, std::string_view namePrefix)
{
    std::unique_ptr<Comm, CommDeleter> comm = ScenarioParser::ParseComm(element, namePrefix);
    if(comm != nullptr)
    {
        ROS2SimulationManager* sim = static_cast<ROS2SimulationManager*>(GetSimulationManager());
        std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs = sim->GetPublishers();
        std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr>& subs = sim->GetSubscribers();
        std::string commName = comm->GetName();

        //Publishing info
        XMLElement* item;
        const char* pubTopic = nullptr;
        const char* subTopic = nullptr;
        if((item = element->FirstChildElement("ros_publisher")) != nullptr)
        {
            item->QueryStringAttribute("topic", &pubTopic);
        }

        if((item = element->FirstChildElement("ros_subscriber")) != nullptr)
        {
            item->QueryStringAttribute("topic", &subTopic);
        }

        if(pubTopic == nullptr && subTopic == nullptr)
            return comm;

        //Generate publishers for different comm types
        switch(comm->GetType())
        {
            case CommType::ACOUSTIC:
            {
                if(pubTopic != nullptr)
                {
                    std::string topicStr {pubTopic};
                    pubs[commName] = nh_->create_publisher<std_msgs::msg::String>(topicStr + "/received_data", 10);
                }
                if(subTopic != nullptr)
                {
                    std::string topicStr {subTopic};
                    std::function<void(const std_msgs::msg::String::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::CommCallback, sim, _1, comm.get());
                    subs[commName + "/data_to_send"] = nh_->create_subscription<std_msgs::msg::String>(topicStr + "/data_to_send", 1, callbackFunc);
                }
            }
                break;

            case CommType::USBL:
            {
                if(pubTopic != nullptr)
                {
                    std::string topicStr {pubTopic};
                    pubs[commName] = nh_->create_publisher<visualization_msgs::msg::MarkerArray>(topicStr, 10);
                    pubs[commName + "/beacon_info"] = nh_->create_publisher<stonefish_ros2::msg::BeaconInfo>(topicStr + "/beacon_info", 10);
                    pubs[commName + "/received_data"] = nh_->create_publisher<std_msgs::msg::String>(topicStr + "/received_data", 10);
                }
                if(subTopic != nullptr)
                {
                    std::string topicStr {subTopic};
                    std::function<void(const std_msgs::msg::String::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::CommCallback, sim, _1, comm.get());
                    subs[commName + "/data_to_send"] = nh_->create_subscription<std_msgs::msg::String>(topicStr + "/data_to_send", 1, callbackFunc);
                }
            }
                break;

            case CommType::OPTICAL:
            {
                if(pubTopic != nullptr)
                {
                    std::string topicStr {pubTopic};
                    pubs[commName] = nh_->create_publisher<std_msgs::msg::Float64>(topicStr + "/reception_quality", 10);
                    pubs[commName + "/received_data"] = nh_->create_publisher<std_msgs::msg::String>(topicStr + "/received_data", 10);
                }
                if(subTopic != nullptr)
                {
                    std::string topicStr {subTopic};
                    std::function<void(const std_msgs::msg::String::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::CommCallback, sim, _1, comm.get());
                    subs[commName + "/data_to_send"] = nh_->create_subscription<std_msgs::msg::String>(topicStr + "/data_to_send", 1, callbackFunc);
                }
            }
                break;

            default:
                break;
        }
    }
    return comm;
}

std::unique_ptr<Light> ROS2ScenarioParser::ParseLight(XMLElement* element, std::string_view namePrefix)
{
    std::unique_ptr<Light> light = ScenarioParser::ParseLight(element, namePrefix);
    
    if(light != nullptr)
    {
        ROS2SimulationManager* sim = static_cast<ROS2SimulationManager*>(GetSimulationManager());
        XMLElement* item;

        //Online update of light origin frame
        const char* originTopic = nullptr;
        if((item = element->FirstChildElement("ros_subscriber")) != nullptr
            && item->QueryStringAttribute("origin", &originTopic) == XML_SUCCESS)
        {
            std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr>& subs = sim->GetSubscribers();
            std::function<void(const geometry_msgs::msg::Transform::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::ActuatorOriginCallback, sim, _1, light.get());
            subs[light->GetName()] = nh_->create_subscription<geometry_msgs::msg::Transform>(std::string(originTopic), 10, callbackFunc);
        }

        //Service for switching the light
        const char* switchTopic = nullptr;
        if((item = element->FirstChildElement("ros_service")) != nullptr
            && item->QueryStringAttribute("topic", &switchTopic) == XML_SUCCESS)
        {
            std::unordered_map<std::string, rclcpp::ServiceBase::SharedPtr>& srvs = sim->GetServices();
            std::function<void(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res)> 
                callbackFunc = std::bind(&ROS2SimulationManager::LightService, sim, _1, _2, light.get());
            srvs[light->GetName()] = nh_->create_service<std_srvs::srv::SetBool>(std::string(switchTopic), callbackFunc);
        }
    }
    return light;
}

bool ROS2ScenarioParser::ParseContact(XMLElement* element)
{
    if(!ScenarioParser::ParseContact(element))
        return false;
   
    ROS2SimulationManager* sim = static_cast<ROS2SimulationManager*>(GetSimulationManager());
    std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs = sim->GetPublishers();
    
    //Contact info
    const char* name = nullptr;
    element->QueryStringAttribute("name", &name);
    std::string contactName = std::string(name);

    //Publishing info
    XMLElement* item;
    const char* topic = nullptr;
    if((item = element->FirstChildElement("ros_publisher")) == nullptr
        || item->QueryStringAttribute("topic", &topic) != XML_SUCCESS)
        return true;
    std::string topicStr(topic);

    pubs[contactName] = nh_->create_publisher<visualization_msgs::msg::Marker>(topicStr, 10);

    return true;
}

FixedJoint* ROS2ScenarioParser::ParseGlue(XMLElement* element)
{   
    FixedJoint* glue = ScenarioParser::ParseGlue(element);
    if(glue != nullptr)
    {
        XMLElement* item;
        const char* topic = nullptr;
        if((item = element->FirstChildElement("ros_service")) == nullptr
            || item->QueryStringAttribute("topic", &topic) != XML_SUCCESS)
        {
            return glue;
        }

        ROS2SimulationManager* sim = static_cast<ROS2SimulationManager*>(GetSimulationManager());
        std::unordered_map<std::string, rclcpp::ServiceBase::SharedPtr>& srvs = sim->GetServices();
        std::function<void(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res)> callbackFunc =
            std::bind(&ROS2SimulationManager::GlueService, sim, _1, _2, glue);
        srvs[glue->GetName()] = nh_->create_service<std_srvs::srv::SetBool>(std::string(topic), callbackFunc);
    }
    return glue;
}

}