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
//  Copyright (c) 2025 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros2/ROS2ScenarioParser.h"
#include "stonefish_ros2/ROS2SimulationManager.h"
#include "stonefish_ros2/ROS2Interface.h"

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
#include "stonefish_ros2/msg/sonar_info.hpp"
#include "stonefish_ros2/srv/sonar_settings.hpp"
#include "stonefish_ros2/srv/sonar_settings2.hpp"
#include "image_transport/image_transport.hpp"
#include "stonefish_ros2/msg/event.hpp"
#include "stonefish_ros2/msg/event_array.hpp"

#include <Stonefish/core/Robot.h>
#include <Stonefish/sensors/scalar/Odometry.h>
#include <Stonefish/entities/SolidEntity.h>
#include <Stonefish/entities/StaticEntity.h>
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
#include <Stonefish/actuators/VariableBuoyancy.h>
#include <Stonefish/actuators/SuctionCup.h>
#include <Stonefish/sensors/ScalarSensor.h>
#include <Stonefish/sensors/vision/ColorCamera.h>
#include <Stonefish/sensors/vision/DepthCamera.h>
#include <Stonefish/sensors/vision/ThermalCamera.h>
#include <Stonefish/sensors/vision/OpticalFlowCamera.h>
#include <Stonefish/sensors/vision/SegmentationCamera.h>
#include <Stonefish/sensors/vision/EventBasedCamera.h>
#include <Stonefish/sensors/vision/Multibeam2.h>
#include <Stonefish/sensors/vision/FLS.h>
#include <Stonefish/sensors/vision/SSS.h>
#include <Stonefish/sensors/vision/MSIS.h>
#include <Stonefish/comms/Comm.h>
#include <Stonefish/joints/FixedJoint.h>

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
    if(!ScenarioParser::PreProcess(root, args))
        return false;

    //Scene landmarks: <ros_landmarks topic="/orca/landmarks" frame_id="orca/odom" rate="10" profile="none"
    //                  front_camera="orca/front_camera_optical" down_camera="orca/downwards_camera_optical"/>
    XMLElement* item = root->FirstChildElement("ros_landmarks");
    if(item != nullptr)
    {
        const char* topic = nullptr;
        const char* frameId = nullptr;
        double rate = 10.0;
        if(item->QueryStringAttribute("topic", &topic) != XML_SUCCESS
           || item->QueryStringAttribute("frame_id", &frameId) != XML_SUCCESS)
        {
            log.Print(MessageType::ERROR, "<ros_landmarks> needs 'topic' and 'frame_id' attributes!");
            return false;
        }
        item->QueryDoubleAttribute("rate", &rate);
        auto attr = [&](const char* name) { const char* v = item->Attribute(name); return std::string(v != nullptr ? v : ""); };
        ((ROS2SimulationManager*)getSimulationManager())->getLandmarks().Configure(
            topic, frameId, rate, attr("profile"), attr("front_camera"), attr("down_camera"));
    }

    //Objects the vehicle lets go of, on a service. See SceneLaunchers.h
    for(XMLElement* l = root->FirstChildElement("ros_launcher"); l != nullptr; l = l->NextSiblingElement("ros_launcher"))
    {
        auto attr = [&](const char* name) { const char* v = l->Attribute(name); return std::string(v != nullptr ? v : ""); };
        int count = 0;
        double speed = 0.0;
        l->QueryIntAttribute("count", &count);
        l->QueryDoubleAttribute("speed", &speed);
        if(attr("service").empty() || attr("robot").empty() || attr("objects").empty() || attr("base_frame").empty()
           || attr("frame").empty() || count < 1)
        {
            log.Print(MessageType::ERROR, "<ros_launcher> needs service, robot, objects, count, base_frame and frame!");
            return false;
        }
        ((ROS2SimulationManager*)getSimulationManager())->getLaunchers().Add(
            attr("service"), attr("robot"), attr("objects"), count, speed, attr("base_frame"), attr("frame"));
    }

    //The waypoint the vehicle is going to, drawn in the 3D view: <ros_waypoint topic="/orca/debug/waypoint"/>
    if((item = root->FirstChildElement("ros_waypoint")) != nullptr)
    {
        const char* topic = nullptr;
        if(item->QueryStringAttribute("topic", &topic) == XML_SUCCESS)
            ((ROS2SimulationManager*)getSimulationManager())->SubscribeWaypoint(topic);
    }

    //The landmark server's frames, drawn in the 3D view: <ros_landmark_frames map_frame="orca/map" world_frame="orca/odom"/>
    if((item = root->FirstChildElement("ros_landmark_frames")) != nullptr)
    {
        const char* mapFrame = nullptr;
        const char* worldFrame = nullptr;
        if(item->QueryStringAttribute("map_frame", &mapFrame) == XML_SUCCESS
           && item->QueryStringAttribute("world_frame", &worldFrame) == XML_SUCCESS)
            ((ROS2SimulationManager*)getSimulationManager())->SubscribeLandmarkFrames(mapFrame, worldFrame);
    }
    return true;
}

bool ROS2ScenarioParser::ParseStatic(XMLElement* element)
{
    return ScenarioParser::ParseStatic(element) && ParseLandmarks(element);
}

bool ROS2ScenarioParser::ParseDynamic(XMLElement* element)
{
    return ScenarioParser::ParseDynamic(element) && ParseLandmarks(element);
}

static bool ParseLandmarkVector(const char* text, Vector3& v)
{
    double x, y, z;
    if(text == nullptr || sscanf(text, "%lf %lf %lf", &x, &y, &z) != 3)
        return false;
    v = Vector3(x, y, z);
    return true;
}

bool ROS2ScenarioParser::IncludeFiles(XMLNode* node)
{
    if(!ScenarioParser::IncludeFiles(node))
        return false;

    //Standalone landmarks, anywhere in the scenario or its includes: <landmark object="name" .../>
    for(XMLElement* item = node->FirstChildElement("landmark"); item != nullptr; item = item->NextSiblingElement("landmark"))
    {
        const char* object = nullptr;
        if(item->QueryStringAttribute("object", &object) != XML_SUCCESS)
        {
            log.Print(MessageType::ERROR, "A <landmark> outside an object needs an 'object' attribute!");
            return false;
        }
        if(!ParseLandmark(item, object))
            return false;
    }
    for(XMLElement* item = node->FirstChildElement("spawn"); item != nullptr; item = item->NextSiblingElement("spawn"))
        if(!ParseSpawnOrPrior(item, SceneLandmark::Kind::SPAWN))
            return false;
    for(XMLElement* item = node->FirstChildElement("prior"); item != nullptr; item = item->NextSiblingElement("prior"))
        if(!ParseSpawnOrPrior(item, SceneLandmark::Kind::PRIOR))
            return false;
    return true;
}

/*
    Where the drone can start, and the tasks of the landmark server's premap, attached to an object
    like landmarks so they move with it (both editable in the scene editor):
        <spawn name="slalom" object="slalom_triple__blackmat" point="x y z" yaw_deg="0"/>
        <prior name="slalom" object="slalom_triple__blackmat" point="x y z" yaw_deg="0"/>
    point and yaw_deg (the heading) are in the object's frame. spawn:=<name> starts the drone there.
*/
bool ROS2ScenarioParser::ParseSpawnOrPrior(XMLElement* item, SceneLandmark::Kind kind)
{
    const char* tag = kind == SceneLandmark::Kind::SPAWN ? "spawn" : "prior";
    const char* name = nullptr;
    const char* object = nullptr;
    Vector3 point;
    if(item->QueryStringAttribute("name", &name) != XML_SUCCESS || item->QueryStringAttribute("object", &object) != XML_SUCCESS
       || !ParseLandmarkVector(item->Attribute("point"), point))
    {
        log.Print(MessageType::ERROR, "A <%s> needs 'name', 'object' and 'point' attributes!", tag);
        return false;
    }
    double yawDeg = 0.0;
    item->QueryDoubleAttribute("yaw_deg", &yawDeg);
    SceneLandmark l;
    l.kind = kind;
    l.entityName = object;
    l.typeName = kind == SceneLandmark::Kind::SPAWN ? "SPAWN" : "PRIOR";
    l.subtypeName = name;
    l.point = point;
    l.normal = Vector3(std::cos(yawDeg * M_PI / 180.0), std::sin(yawDeg * M_PI / 180.0), 0);
    std::string error;
    return ((ROS2SimulationManager*)getSimulationManager())->getLandmarks().Add(l, error);
}

bool ROS2ScenarioParser::PlaceDrone()
{
    // After the whole scenario is built, so a spawn point can be on any object, also a robot defined
    // after the drone (the TAC pipeline)
    ROS2SimulationManager* sim = (ROS2SimulationManager*)getSimulationManager();
    Robot* robot = sim->getRobot(0u);
    if(robot == nullptr)
        return true;
    if(!nh_->has_parameter("spawn"))
        nh_->declare_parameter("spawn", std::string(""));
    std::string spawn = nh_->get_parameter("spawn").as_string();

    // The heading the stack sees is the odometry's, which depends on how the drone file mounts the
    // model (Orca faces -90 deg in the TAC config and 0 in RoboSub): measure it on the built robot
    Odometry* odom = nullptr;
    for(size_t i = 0; odom == nullptr && robot->getSensor(i) != nullptr; ++i)
        odom = dynamic_cast<Odometry*>(robot->getSensor(i));
    auto heading = [](const Matrix3& R) {
        Vector3 x = R.getColumn(0);
        return std::atan2(x.getY(), x.getX());
    };
    Scalar current = odom != nullptr ? heading(odom->getSensorFrame().getBasis()) : Scalar(0);
    if(spawn.empty())
    {
        sim->setStartPose(robotOrigin_.getOrigin(), current, "");
        return true;
    }

    std::string names;
    for(const SceneLandmark& l : sim->getLandmarks().All())
    {
        if(l.kind != SceneLandmark::Kind::SPAWN)
            continue;
        names += " " + l.subtypeName;
        Vector3 p;
        Transform T;
        if(l.subtypeName != spawn || !sim->getLandmarks().WorldPosition(l, p) || !sim->getLandmarks().EntityTransform(l.entityName, T))
            continue;
        Vector3 h = T.getBasis() * *l.normal;
        Scalar yaw = std::atan2(h.getY(), h.getX());
        // Turn the drone file's pose about the world's vertical so the odometry heading is the spawn's
        Scalar delta = std::atan2(std::sin(yaw - current), std::cos(yaw - current));
        robot->Respawn(sim, Transform(Quaternion(delta, 0, 0) * robotOrigin_.getRotation(), p));
        sim->setStartPose(p, yaw, spawn);
        log.Print(MessageType::INFO, "[ROS] Spawned at '%s': %.2f %.2f %.2f, heading %.1f deg", spawn.c_str(),
                  (double)p.getX(), (double)p.getY(), (double)p.getZ(), yaw * 180.0 / M_PI);
        RCLCPP_INFO(nh_->get_logger(), "Spawned at '%s' (%.2f, %.2f, %.2f), heading %.1f deg", spawn.c_str(),
                    (double)p.getX(), (double)p.getY(), (double)p.getZ(), yaw * 180.0 / M_PI);
        return true;
    }
    log.Print(MessageType::ERROR, "No spawn point '%s' in the scenario, the spawn points are:%s", spawn.c_str(), names.c_str());
    RCLCPP_ERROR(nh_->get_logger(), "No spawn point '%s' in the scenario, the spawn points are:%s", spawn.c_str(), names.c_str());
    sim->setStartPose(robotOrigin_.getOrigin(), current, "");
    return false;
}

/*
    Landmarks published by SceneLandmarks, either inside a <static>/<dynamic> object or at the
    top level with object="name":
        <landmark type="GATE" subtype="GATE_SEARCH_RESCUE" point="x y z" normal="x y z"/>
    point (optional) is in the object's frame, default is the centre of its bounding box.
    normal (optional, object frame) publishes its yaw, e.g. the front face of a board.
    rpy (optional, object frame) publishes the full orientation of that frame, e.g. a valve
    (Z out of its face, X along the handle). object can be a robot link: "robot/link".
    enabled="false" keeps the landmark in the scene without publishing it.
    camera="down" says the down camera sees it (bin floors, table items), for detection profiles.
*/
bool ROS2ScenarioParser::ParseLandmarks(XMLElement* element)
{
    const char* name = nullptr;
    if(element->QueryStringAttribute("name", &name) != XML_SUCCESS)
        return true;

    for(XMLElement* item = element->FirstChildElement("landmark"); item != nullptr; item = item->NextSiblingElement("landmark"))
        if(!ParseLandmark(item, name))
            return false;
    return true;
}

bool ROS2ScenarioParser::ParseLandmark(XMLElement* item, const std::string& objectName)
{
    SceneLandmark l;
    l.entityName = objectName;
    const char* type = nullptr;
    const char* subtype = nullptr;
    if(item->QueryStringAttribute("type", &type) != XML_SUCCESS)
    {
        log.Print(MessageType::ERROR, "Landmark of '%s' has no type!", objectName.c_str());
        return false;
    }
    l.typeName = type;
    if(item->QueryStringAttribute("subtype", &subtype) == XML_SUCCESS)
        l.subtypeName = subtype;

    Vector3 v;
    if(ParseLandmarkVector(item->Attribute("point"), v))
        l.point = v;
    if(ParseLandmarkVector(item->Attribute("normal"), v))
        l.normal = v;
    if(ParseLandmarkVector(item->Attribute("rpy"), v))
        l.rotation = Quaternion(v.getZ(), v.getY(), v.getX());
    item->QueryBoolAttribute("enabled", &l.enabled);
    const char* camera = item->Attribute("camera");
    l.downCamera = camera != nullptr && std::string(camera) == "down";

    std::string error;
    if(!((ROS2SimulationManager*)getSimulationManager())->getLandmarks().Add(l, error))
    {
        log.Print(MessageType::ERROR, "Landmark of '%s': %s!", objectName.c_str(), error.c_str());
        return false;
    }
    return true;
}

VelocityField* ROS2ScenarioParser::ParseVelocityField(XMLElement* element)
{
    VelocityField* vf = ScenarioParser::ParseVelocityField(element);
    if(vf != nullptr)
    {
        ROS2SimulationManager* sim = (ROS2SimulationManager*)getSimulationManager();
        std::map<std::string, rclcpp::SubscriptionBase::SharedPtr>& subs = sim->getSubscribers();
        XMLElement* item;

        switch(vf->getType())
        {
            case VelocityFieldType::UNIFORM:
            {
                const char* subTopic = nullptr;
                if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                   && item->QueryStringAttribute("velocity", &subTopic) == XML_SUCCESS)
                {
                    std::function<void(const geometry_msgs::msg::Vector3::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::UniformVFCallback, sim, _1, (Uniform*)vf);               
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
                        std::bind(&ROS2SimulationManager::JetVFCallback, sim, _1, (Jet*)vf);
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
    // The first robot is the drone (the camera follows it too)
    if(getSimulationManager()->getRobot(0u) == nullptr)
    {
        // Its pose as the drone file places it, for PlaceDrone()
        XMLElement* wt = element->FirstChildElement("world_transform");
        Vector3 xyz, rpy;
        if(wt != nullptr && ParseLandmarkVector(wt->Attribute("xyz"), xyz) && ParseLandmarkVector(wt->Attribute("rpy"), rpy))
            robotOrigin_ = Transform(Quaternion(rpy.z(), rpy.y(), rpy.x()), xyz);
    }
    if(!ScenarioParser::ParseRobot(element))
        return false;

    ROS2SimulationManager* sim = (ROS2SimulationManager*)getSimulationManager();
    std::map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs = sim->getPublishers();
    std::map<std::string, rclcpp::SubscriptionBase::SharedPtr>& subs = sim->getSubscribers();
    
    //Robot info
    const char* name = nullptr;
    element->QueryStringAttribute("name", &name);
    std::string nameStr(name);
    Robot* robot = getSimulationManager()->getRobot(nameStr);

    //Check actuators
    unsigned int nThrusters = 0;
    unsigned int nPropellers = 0;
    unsigned int nServos = 0;
    unsigned int nMotors = 0;
    unsigned int nRudders = 0;

    unsigned int aID = 0;
    Actuator* act;
    while((act = robot->getActuator(aID++)) != NULL)
    {
        switch(act->getType())
        {
            case ActuatorType::SIMPLE_THRUSTER:
            case ActuatorType::THRUSTER:
                ++nThrusters;
                break;

            case ActuatorType::PROPELLER:
                ++nPropellers;
                break;

            case ActuatorType::RUDDER:
                ++nRudders;
                break;

            case ActuatorType::MOTOR:
                ++nMotors;
                break;

            case ActuatorType::SERVO:
                ++nServos;
                break;

            default:
                break;
        }
    }

    std::shared_ptr<ROS2Robot> rosRobot(new ROS2Robot(robot, nThrusters, nPropellers, nRudders));

    //Check if we should publish world_ned -> base_link transform
    XMLElement* item;
    if((item = element->FirstChildElement("ros_base_link_transform")) != nullptr)
        item->QueryBoolAttribute("publish", &rosRobot->publishBaseLinkTransform_);

    //Check if we should publish debug information
    if((item = element->FirstChildElement("ros_debug")) != nullptr)
    {
        const char* topicPhysics = nullptr;
        if(item->QueryStringAttribute("physics", &topicPhysics) == XML_SUCCESS)
            pubs[robot->getName() + "/debug/physics"] = nh_->create_publisher<stonefish_ros2::msg::DebugPhysics>(std::string(topicPhysics), 10);
    }

    //Save robot
    sim->AddROS2Robot(rosRobot);

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
                        std::bind(&ROS2SimulationManager::ThrustersCallback, sim, _1, rosRobot);
            subs[robot->getName() + "/thrusters"] = nh_->create_subscription<std_msgs::msg::Float64MultiArray>(std::string(topicThrust), 10, callbackFunc);
        }

        if(nPropellers > 0 && item->QueryStringAttribute("propellers", &topicProp) == XML_SUCCESS)
        {
            std::function<void(const std_msgs::msg::Float64MultiArray::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::PropellersCallback, sim, _1, rosRobot);
            subs[robot->getName() + "/propellers"] = nh_->create_subscription<std_msgs::msg::Float64MultiArray>(std::string(topicProp), 10, callbackFunc);
        }

        if(nRudders > 0 && item->QueryStringAttribute("rudders", &topicRudder) == XML_SUCCESS)
        {
            std::function<void(const std_msgs::msg::Float64MultiArray::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::RuddersCallback, sim, _1, rosRobot);
            subs[robot->getName() + "/rudders"] = nh_->create_subscription<std_msgs::msg::Float64MultiArray>(std::string(topicRudder), 10, callbackFunc);
        }

        if(nServos > 0 && item->QueryStringAttribute("servos", &topicSrv) == XML_SUCCESS)
        {
            std::function<void(const sensor_msgs::msg::JointState::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::ServosCallback, sim, _1, rosRobot);
            subs[robot->getName() + "/servos"] = nh_->create_subscription<sensor_msgs::msg::JointState>(std::string(topicSrv), 10, callbackFunc);
            //Generate setpoint placeholders for the whole system
            aID = 0;
            while((act = robot->getActuator(aID++)) != nullptr)
            {
                if(act->getType() == ActuatorType::SERVO)
                    rosRobot->servoSetpoints_[((Servo*)act)->getJointName()] = std::pair(ServoControlMode::VELOCITY, Scalar(0));
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
                    jointNames.push_back(robot->getName() + "/" + std::string(jgJointName));
            }

            if(jointNames.size() > 0) // Any joints defined?
            {
                RCLCPP_INFO_STREAM(nh_->get_logger(), "Creating joint group subscriber (" << std::string(jgMode) << ") " << std::string(jgTopic));
                for(size_t i=0; i<jointNames.size(); ++i)
                    rosRobot->servoSetpoints_[jointNames[i]] = std::make_pair(mode, Scalar(0));
                std::function<void(const std_msgs::msg::Float64MultiArray::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::JointGroupCallback, sim, _1, rosRobot, mode, jointNames);
                subs[robot->getName() + "/joint_group" + std::to_string(jg)] = nh_->create_subscription<std_msgs::msg::Float64MultiArray>(std::string(jgTopic), 10, callbackFunc);
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
                std::string jointNameStr = robot->getName() + "/" + std::string(jointName);
                rosRobot->servoSetpoints_[jointNameStr] = std::make_pair(mode, Scalar(0));
                std::function<void(const std_msgs::msg::Float64::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::JointCallback, sim, _1, rosRobot, mode, jointNameStr);
                subs[robot->getName() + "/joint" + std::to_string(jg)] = nh_->create_subscription<std_msgs::msg::Float64>(std::string(jgTopic), 10, callbackFunc);
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
            pubs[robot->getName() + "/thrusters"] = nh_->create_publisher<stonefish_ros2::msg::ThrusterState>(std::string(topicThr), 10);

        if(nServos > 0 && item->QueryStringAttribute("servos", &topicSrv) == XML_SUCCESS)
            pubs[robot->getName() + "/servos"] = nh_->create_publisher<sensor_msgs::msg::JointState>(std::string(topicSrv), 10);

        if(nMotors > 0 && item->QueryStringAttribute("motors", &topicMtr) == XML_SUCCESS)
            pubs[robot->getName() + "/motors"] = nh_->create_publisher<sensor_msgs::msg::JointState>(std::string(topicMtr), 10);

        if(nRudders > 0 && item->QueryStringAttribute("rudders", &topicRud) == XML_SUCCESS)
            pubs[robot->getName() + "/rudders"] = nh_->create_publisher<sensor_msgs::msg::JointState>(std::string(topicRud), 10);
    }

    return true;
}

bool ROS2ScenarioParser::ParseAnimated(XMLElement* element)
{
    if(!ScenarioParser::ParseAnimated(element))
        return false;

    ROS2SimulationManager* sim = (ROS2SimulationManager*)getSimulationManager();
    std::map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs = sim->getPublishers();
    std::map<std::string, rclcpp::SubscriptionBase::SharedPtr>& subs = sim->getSubscribers();

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
        AnimatedEntity* anim = (AnimatedEntity*)sim->getEntity(nameStr);
        if(anim != nullptr)
        {
            std::function<void(const nav_msgs::msg::Odometry::SharedPtr msg)> callbackFunc =
                std::bind(&ROS2SimulationManager::TrajectoryCallback, sim, _1, (ManualTrajectory*)anim->getTrajectory());
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

Actuator* ROS2ScenarioParser::ParseActuator(XMLElement* element, const std::string& namePrefix)
{
    Actuator* act = ScenarioParser::ParseActuator(element, namePrefix);
    
    if(act != nullptr)
    {
        ROS2SimulationManager* sim = (ROS2SimulationManager*)getSimulationManager();
        std::map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs = sim->getPublishers();
        std::map<std::string, rclcpp::SubscriptionBase::SharedPtr>& subs = sim->getSubscribers();
        std::map<std::string, rclcpp::ServiceBase::SharedPtr>& srvs = sim->getServices();
        std::string actuatorName = act->getName();
        XMLElement* item;
        //Actuator specific handling
        switch(act->getType())
        {
            case ActuatorType::SIMPLE_THRUSTER:
            {
                const char* subTopic = nullptr;
                if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                    && item->QueryStringAttribute("topic", &subTopic) == XML_SUCCESS)
                {
                    std::function<void(const std_msgs::msg::Float64::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::SimpleThrusterCallback, sim, _1, (SimpleThruster*)act);     
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

            case ActuatorType::THRUSTER:
            {
                const char* subTopic = nullptr;
                if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                    && item->QueryStringAttribute("topic", &subTopic) == XML_SUCCESS)
                {
                    std::function<void(const std_msgs::msg::Float64::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::ThrusterCallback, sim, _1, (Thruster*)act);     
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

            case ActuatorType::PROPELLER:
            {
                const char* subTopic = nullptr;
                if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                    && item->QueryStringAttribute("topic", &subTopic) == XML_SUCCESS)
                {
                    std::function<void(const std_msgs::msg::Float64::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::PropellerCallback, sim, _1, (Propeller*)act);     
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

            case ActuatorType::PUSH:
            {
                const char* subTopic = nullptr;
                if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                    && item->QueryStringAttribute("topic", &subTopic) == XML_SUCCESS)
                {
                    std::function<void(const std_msgs::msg::Float64::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::PushCallback, sim, _1, (Push*)act);     
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

            case ActuatorType::VBS:
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
                        std::bind(&ROS2SimulationManager::VBSCallback, sim, _1, (VariableBuoyancy*)act);     
                    subs[actuatorName] = nh_->create_subscription<std_msgs::msg::Float64>(std::string(subTopic), 10, callbackFunc);
                }
            }
                break;

            case ActuatorType::SUCTION_CUP:
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
                        std::bind(&ROS2SimulationManager::SuctionCupService, sim, _1, _2, (SuctionCup*)act);
                    srvs[actuatorName] = nh_->create_service<std_srvs::srv::SetBool>(std::string(srvTopic), callbackFunc);
                }
            }
                break;

            default:
                break;
        }
        //Handling of online origin updates
        switch(act->getType())
        {
            case ActuatorType::PUSH:
            case ActuatorType::SIMPLE_THRUSTER:
            case ActuatorType::THRUSTER:
            case ActuatorType::PROPELLER:
            case ActuatorType::VBS:
            {
                const char* originTopic = nullptr;
                if((item = element->FirstChildElement("ros_subscriber")) != nullptr
                    && item->QueryStringAttribute("origin", &originTopic) == XML_SUCCESS)
                {
                    std::function<void(const geometry_msgs::msg::Transform::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::ActuatorOriginCallback, sim, _1, act);     
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

Sensor* ROS2ScenarioParser::ParseSensor(XMLElement* element, const std::string& namePrefix)
{
    Sensor* sens = ScenarioParser::ParseSensor(element, namePrefix);
    
    if(sens != nullptr)
    {
        ROS2SimulationManager* sim = (ROS2SimulationManager*)getSimulationManager();
        std::shared_ptr<image_transport::ImageTransport> it = sim->getImageTransportHandle();
        std::map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs = sim->getPublishers();
        std::map<std::string, rclcpp::SubscriptionBase::SharedPtr>& subs = sim->getSubscribers();
        std::map<std::string, rclcpp::ServiceBase::SharedPtr>& srvs = sim->getServices();
        std::map<std::string, image_transport::Publisher>& img_pubs = sim->getImagePublishers();
        std::map<std::string, std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr>>& camMsgProto = sim->getCameraMsgPrototypes();
        std::map<std::string, std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr, sensor_msgs::msg::Image::SharedPtr>>& dualCamMsgProto = sim->getDualImageCameraMsgPrototypes();
        std::map<std::string, std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::Image::SharedPtr>>& sonarMsgProto = sim->getSonarMsgPrototypes();
        
        //Publishing info
        std::string sensorName = sens->getName();
        XMLElement* item;
        const char* topic = nullptr;
        const char* frameId = nullptr;
        std::string frameIdStr = "";

        //Standard sensor publisher
        if((item = element->FirstChildElement("ros_publisher")) != nullptr)
        {
            if(item->QueryStringAttribute("topic", &topic) != XML_SUCCESS)
                return sens;

            if(item->QueryStringAttribute("frame_id", &frameId) == XML_SUCCESS)
                frameIdStr = std::string(frameId);
        }    
        else
        {
            return sens;
        }
        std::string topicStr(topic);

        //Sensor enable/disable service
        const char* serviceTopic = nullptr;
        XMLElement* item2; // item is used below so we need a new pointer
        if((item2 = element->FirstChildElement("ros_service")) != nullptr
            && item2->QueryStringAttribute("set_enabled", &serviceTopic) == XML_SUCCESS)
        {
            std::string serviceTopicStr(serviceTopic);
            std::function<void(const std_srvs::srv::SetBool::Request::SharedPtr req, 
                            std_srvs::srv::SetBool::Response::SharedPtr res)> callbackFunc =
                            std::bind(&ROS2SimulationManager::SensorService, sim, _1, _2, sens);
            srvs[sensorName+"/set_enabled"] = nh_->create_service<std_srvs::srv::SetBool>(serviceTopicStr, callbackFunc);
            sens->setEnabled(false); // Sensor with an enable service starts as disabled
        }

        unsigned int queueSize = (unsigned int)ceil(sens->getUpdateFrequency());

        //Generate publishers for different sensor types
        switch(sens->getType())
        {
            case SensorType::JOINT:
            {
                switch(((ScalarSensor*)sens)->getScalarSensorType())
                {
                    case ScalarSensorType::FT:
                        pubs[sensorName] = nh_->create_publisher<geometry_msgs::msg::WrenchStamped>(topicStr, queueSize);
                        break;

                    case ScalarSensorType::ENCODER:
                        pubs[sensorName] = nh_->create_publisher<sensor_msgs::msg::JointState>(topicStr, queueSize);
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
                switch(((ScalarSensor*)sens)->getScalarSensorType())
                {
                    case ScalarSensorType::ACC:
                        pubs[sensorName] = nh_->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(topicStr, queueSize);
                        break;

                    case ScalarSensorType::GYRO:
                        pubs[sensorName] = nh_->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(topicStr, queueSize);
                        break;

                    case ScalarSensorType::IMU:
                        pubs[sensorName] = nh_->create_publisher<sensor_msgs::msg::Imu>(topicStr, queueSize);
                        break;

                    case ScalarSensorType::DVL:
                    {
                        pubs[sensorName] = nh_->create_publisher<stonefish_ros2::msg::DVL>(topicStr, queueSize);

                        //Second topic with altitude
                        const char* altTopic = nullptr;
                        if(item->QueryStringAttribute("altitude_topic", &altTopic) == XML_SUCCESS)
                        {
                            std::string altTopicStr = std::string(altTopic);
                            pubs[sensorName + "/altitude"] = nh_->create_publisher<sensor_msgs::msg::Range>(altTopicStr, queueSize);
                        }
                    }
                        break;

                    case ScalarSensorType::GPS:
                        pubs[sensorName] = nh_->create_publisher<sensor_msgs::msg::NavSatFix>(topicStr, queueSize);
                        break;

                    case ScalarSensorType::PRESSURE:
                        pubs[sensorName] = nh_->create_publisher<sensor_msgs::msg::FluidPressure>(topicStr, queueSize);
                        break;

                    case ScalarSensorType::INS:
                    {
                        pubs[sensorName] = nh_->create_publisher<stonefish_ros2::msg::INS>(topicStr, queueSize);

                        //Second topic with odometry
                        const char* odomTopic = nullptr;
                        if(item->QueryStringAttribute("odometry_topic", &odomTopic) == XML_SUCCESS)
                        {
                            std::string odomTopicStr = std::string(odomTopic);
                            pubs[sensorName + "/odometry"] = nh_->create_publisher<nav_msgs::msg::Odometry>(odomTopicStr, queueSize);
                        }
                    }
                        break;

                    case ScalarSensorType::ODOM:
                        pubs[sensorName] = nh_->create_publisher<nav_msgs::msg::Odometry>(topicStr, queueSize);
                        break;

                    case ScalarSensorType::MULTIBEAM:
                    {
                        pubs[sensorName] = nh_->create_publisher<sensor_msgs::msg::LaserScan>(topicStr, queueSize);

                        //Second topic with point cloud
                        const char* pclTopic = nullptr;
                        if(item->QueryStringAttribute("pcl_topic", &pclTopic) == XML_SUCCESS)
                        {
                            std::string pclTopicStr = std::string(pclTopic);
                            pubs[sensorName + "/pcl"] = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(pclTopicStr, queueSize);
                        }
                    }
                        break;

                    case ScalarSensorType::PROFILER:
                        pubs[sensorName] = nh_->create_publisher<sensor_msgs::msg::LaserScan>(topicStr, queueSize);
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
                        std::bind(&ROS2SimulationManager::SensorOriginCallback, sim, _1, sens);
                    subs[sensorName] = nh_->create_subscription<geometry_msgs::msg::Transform>(std::string(originTopic), queueSize, callbackFunc);
                }
            }
                break;

            case SensorType::VISION:
            {
                switch(((VisionSensor*)sens)->getVisionSensorType())
                {
                    case VisionSensorType::COLOR_CAMERA:
                    {
                        img_pubs[sensorName] = it->advertise(topicStr + "/image_color", queueSize);
                        pubs[sensorName + "/info"] = nh_->create_publisher<sensor_msgs::msg::CameraInfo>(topicStr + "/camera_info", queueSize);
                        ColorCamera* cam = (ColorCamera*)sens;
                        cam->InstallNewDataHandler(std::bind(&ROS2SimulationManager::ColorCameraImageReady, sim, _1));
                        camMsgProto[sensorName] = ROS2Interface::GenerateCameraMsgPrototypes(cam, false, frameIdStr);
                    }
                        break;

                    case VisionSensorType::DEPTH_CAMERA:
                    {
                        img_pubs[sensorName] = it->advertise(topicStr + "/image_depth", queueSize);
                        pubs[sensorName + "/info"] = nh_->create_publisher<sensor_msgs::msg::CameraInfo>(topicStr + "/camera_info", queueSize);
                        DepthCamera* cam = (DepthCamera*)sens;
                        cam->InstallNewDataHandler(std::bind(&ROS2SimulationManager::DepthCameraImageReady, sim, _1));
                        camMsgProto[sensorName] = ROS2Interface::GenerateCameraMsgPrototypes(cam, true, frameIdStr);
                    }
                        break;

                    case VisionSensorType::THERMAL_CAMERA:
                    {
                        img_pubs[sensorName] = it->advertise(topicStr + "/image_raw", queueSize);
                        img_pubs[sensorName + "/display"] = it->advertise(topicStr + "/image_color", queueSize);
                        pubs[sensorName + "/info"] = nh_->create_publisher<sensor_msgs::msg::CameraInfo>(topicStr + "/camera_info", queueSize);                        
                        ThermalCamera* cam = (ThermalCamera*)sens;
                        cam->InstallNewDataHandler(std::bind(&ROS2SimulationManager::ThermalCameraImageReady, sim, _1));
                        dualCamMsgProto[sensorName] = ROS2Interface::GenerateThermalCameraMsgPrototypes(cam);
                    }
                        break;

                    case VisionSensorType::OPTICAL_FLOW_CAMERA:
                    {
                        img_pubs[sensorName] = it->advertise(topicStr + "/image_raw", queueSize);
                        img_pubs[sensorName + "/display"] = it->advertise(topicStr + "/image_color", queueSize);
                        pubs[sensorName + "/info"] = nh_->create_publisher<sensor_msgs::msg::CameraInfo>(topicStr + "/camera_info", queueSize);                        
                        OpticalFlowCamera* cam = (OpticalFlowCamera*)sens;
                        cam->InstallNewDataHandler(std::bind(&ROS2SimulationManager::OpticalFlowCameraImageReady, sim, _1));
                        dualCamMsgProto[sensorName] = ROS2Interface::GenerateOpticalFlowCameraMsgPrototypes(cam);
                    }
                        break;

                    case VisionSensorType::SEGMENTATION_CAMERA:
                    {
                        img_pubs[sensorName] = it->advertise(topicStr + "/image_raw", queueSize);
                        img_pubs[sensorName + "/display"] = it->advertise(topicStr + "/image_color", queueSize);
                        pubs[sensorName + "/info"] = nh_->create_publisher<sensor_msgs::msg::CameraInfo>(topicStr + "/camera_info", queueSize);                        
                        SegmentationCamera* cam = (SegmentationCamera*)sens;
                        cam->InstallNewDataHandler(std::bind(&ROS2SimulationManager::SegmentationCameraImageReady, sim, _1));
                        dualCamMsgProto[sensorName] = ROS2Interface::GenerateSegmentationCameraMsgPrototypes(cam);
                    }
                        break;

                    case VisionSensorType::EVENT_BASED_CAMERA:
                    {
                        pubs[sensorName] = nh_->create_publisher<stonefish_ros2::msg::EventArray>(topicStr, queueSize);
                        EventBasedCamera* cam = (EventBasedCamera*)sens;
                        cam->InstallNewDataHandler(std::bind(&ROS2SimulationManager::EventBasedCameraOutputReady, sim, _1));
                    }
                        break;

                    case VisionSensorType::MULTIBEAM2:
                    {
                        pubs[sensorName] = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(topicStr, queueSize);
                        Multibeam2* mb = (Multibeam2*)sens;
                        mb->InstallNewDataHandler(std::bind(&ROS2SimulationManager::Multibeam2ScanReady, sim, _1));
                    }
                        break;

                    case VisionSensorType::FLS:
                    {
                        FLS* fls = (FLS*)sens;
                        fls->InstallNewDataHandler(std::bind(&ROS2SimulationManager::FLSScanReady, sim, _1));
                        sonarMsgProto[sensorName] = ROS2Interface::GenerateFLSMsgPrototypes(fls);
                        std::function<void(const stonefish_ros2::srv::SonarSettings::Request::SharedPtr req, 
                            stonefish_ros2::srv::SonarSettings::Response::SharedPtr res)> callbackFunc =
                        std::bind(&ROS2SimulationManager::FLSService, sim, _1, _2, fls);
                        srvs[sensorName] = nh_->create_service<stonefish_ros2::srv::SonarSettings>(topicStr + "/settings", callbackFunc);
                        img_pubs[sensorName] = it->advertise(topicStr + "/image", queueSize);
                        img_pubs[sensorName + "/display"] = it->advertise(topicStr + "/display", queueSize);
                        img_pubs[sensorName + "/display_mono"] = it->advertise(topicStr + "/display_mono", queueSize);
                        pubs[sensorName + "/info"] = nh_->create_publisher<stonefish_ros2::msg::SonarInfo>(topicStr + "/stonefish_sonar_info", queueSize);
                    }
                        break;

                    case VisionSensorType::SSS:
                    {
                        SSS* sss = (SSS*)sens;
                        sss->InstallNewDataHandler(std::bind(&ROS2SimulationManager::SSSScanReady, sim, _1));
                        sonarMsgProto[sensorName] = ROS2Interface::GenerateSSSMsgPrototypes(sss);
                        std::function<void(const stonefish_ros2::srv::SonarSettings::Request::SharedPtr req, 
                            stonefish_ros2::srv::SonarSettings::Response::SharedPtr res)> callbackFunc =
                        std::bind(&ROS2SimulationManager::SSSService, sim, _1, _2, sss);
                        srvs[sensorName] = nh_->create_service<stonefish_ros2::srv::SonarSettings>(topicStr + "/settings", callbackFunc);
                        img_pubs[sensorName] = it->advertise(topicStr + "/image", queueSize);
                        img_pubs[sensorName + "/display"] = it->advertise(topicStr + "/display", queueSize);
                    }
                        break;

                    case VisionSensorType::MSIS:
                    {
                        MSIS* msis = (MSIS*)sens;
                        msis->InstallNewDataHandler(std::bind(&ROS2SimulationManager::MSISScanReady, sim, _1));
                        sonarMsgProto[sensorName] = ROS2Interface::GenerateMSISMsgPrototypes(msis);
                        std::function<void(const stonefish_ros2::srv::SonarSettings2::Request::SharedPtr req, 
                            stonefish_ros2::srv::SonarSettings2::Response::SharedPtr res)> callbackFunc =
                        std::bind(&ROS2SimulationManager::MSISService, sim, _1, _2, msis);                    
                        srvs[sensorName] = nh_->create_service<stonefish_ros2::srv::SonarSettings2>(topicStr + "/settings", callbackFunc);
                        img_pubs[sensorName] = it->advertise(topicStr + "/image", queueSize);
                        img_pubs[sensorName + "/display"] = it->advertise(topicStr + "/display", queueSize);
                        pubs[sensorName + "/beam"] = nh_->create_publisher<sensor_msgs::msg::LaserScan>(topicStr + "/last_beam", queueSize);
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
                        std::bind(&ROS2SimulationManager::SensorOriginCallback, sim, _1, sens);
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

Comm* ROS2ScenarioParser::ParseComm(XMLElement* element, const std::string& namePrefix)
{
    Comm* comm = ScenarioParser::ParseComm(element, namePrefix);
    if(comm != nullptr)
    {
        ROS2SimulationManager* sim = (ROS2SimulationManager*)getSimulationManager();
        std::map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs = sim->getPublishers();
        std::map<std::string, rclcpp::SubscriptionBase::SharedPtr>& subs = sim->getSubscribers();
        std::string commName = comm->getName();

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
        switch(comm->getType())
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
                        std::bind(&ROS2SimulationManager::CommCallback, sim, _1, comm);
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
                        std::bind(&ROS2SimulationManager::CommCallback, sim, _1, comm);
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
                        std::bind(&ROS2SimulationManager::CommCallback, sim, _1, comm);
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

Light* ROS2ScenarioParser::ParseLight(XMLElement* element, const std::string& namePrefix)
{
    Light* l = ScenarioParser::ParseLight(element, namePrefix);
    
    if(l != nullptr)
    {
        ROS2SimulationManager* sim = (ROS2SimulationManager*)getSimulationManager();
        XMLElement* item;

        //Online update of light origin frame
        const char* originTopic = nullptr;
        if((item = element->FirstChildElement("ros_subscriber")) != nullptr
            && item->QueryStringAttribute("origin", &originTopic) == XML_SUCCESS)
        {
            std::map<std::string, rclcpp::SubscriptionBase::SharedPtr>& subs = sim->getSubscribers();
            std::function<void(const geometry_msgs::msg::Transform::SharedPtr msg)> callbackFunc =
                        std::bind(&ROS2SimulationManager::ActuatorOriginCallback, sim, _1, (Actuator*)l);
            subs[l->getName()] = nh_->create_subscription<geometry_msgs::msg::Transform>(std::string(originTopic), 10, callbackFunc);
        }

        //Service for switching the light
        const char* switchTopic = nullptr;
        if((item = element->FirstChildElement("ros_service")) != nullptr
            && item->QueryStringAttribute("topic", &switchTopic) == XML_SUCCESS)
        {
            std::map<std::string, rclcpp::ServiceBase::SharedPtr>& srvs = sim->getServices();
            std::function<void(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res)> 
                callbackFunc = std::bind(&ROS2SimulationManager::LightService, sim, _1, _2, l);
            srvs[l->getName()] = nh_->create_service<std_srvs::srv::SetBool>(std::string(switchTopic), callbackFunc);
        }
    }
    return l;
}

bool ROS2ScenarioParser::ParseContact(XMLElement* element)
{
    if(!ScenarioParser::ParseContact(element))
        return false;
   
    ROS2SimulationManager* sim = (ROS2SimulationManager*)getSimulationManager();
    std::map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs = sim->getPublishers();
    
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

        ROS2SimulationManager* sim = (ROS2SimulationManager*)getSimulationManager();
        std::map<std::string, rclcpp::ServiceBase::SharedPtr>& srvs = sim->getServices();
        std::function<void(const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr res)> callbackFunc =
            std::bind(&ROS2SimulationManager::GlueService, sim, _1, _2, glue);
        srvs[glue->getName()] = nh_->create_service<std_srvs::srv::SetBool>(std::string(topic), callbackFunc);
    }
    return glue;
}


}