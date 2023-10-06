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
//  ROS2SimulationManager.cpp
//  stonefish_ros2
//
//  Created by Patryk Cieslak on 02/10/23.
//  Copyright (c) 2023 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros2/ROS2SimulationManager.h"
#include "stonefish_ros2/ROS2ScenarioParser.h"
#include "stonefish_ros2/ROS2Interface.h"
#include "stonefish_ros2/msg/thruster_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include <Stonefish/entities/animation/ManualTrajectory.h>
#include <Stonefish/entities/forcefields/Uniform.h>
#include <Stonefish/entities/forcefields/Jet.h>
#include <Stonefish/sensors/scalar/Pressure.h>
#include <Stonefish/sensors/scalar/DVL.h>
#include <Stonefish/sensors/scalar/Accelerometer.h>
#include <Stonefish/sensors/scalar/Gyroscope.h>
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/sensors/scalar/GPS.h>
#include <Stonefish/sensors/scalar/ForceTorque.h>
#include <Stonefish/sensors/scalar/RotaryEncoder.h>
#include <Stonefish/sensors/scalar/Odometry.h>
#include <Stonefish/sensors/vision/ColorCamera.h>
#include <Stonefish/sensors/vision/DepthCamera.h>
#include <Stonefish/sensors/scalar/Multibeam.h>
#include <Stonefish/sensors/vision/Multibeam2.h>
#include <Stonefish/sensors/vision/FLS.h>
#include <Stonefish/sensors/vision/SSS.h>
#include <Stonefish/sensors/vision/MSIS.h>
#include <Stonefish/sensors/Contact.h>
#include <Stonefish/comms/USBL.h>
#include <Stonefish/actuators/Push.h>
#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/actuators/Propeller.h>
#include <Stonefish/actuators/Rudder.h>
#include <Stonefish/actuators/SuctionCup.h>
#include <Stonefish/actuators/Servo.h>
#include <Stonefish/actuators/VariableBuoyancy.h>
#include <Stonefish/core/Robot.h>

using namespace std::placeholders;

namespace sf
{

ROS2SimulationManager::ROS2SimulationManager(Scalar stepsPerSecond, std::string scenarioFilePath, const std::shared_ptr<rclcpp::Node>& nh)
	: SimulationManager(stepsPerSecond, SolverType::SOLVER_SI, CollisionFilteringType::COLLISION_EXCLUSIVE), scenarioPath_(scenarioFilePath), nh_(nh)
{
    it_ = std::make_shared<image_transport::ImageTransport>(nh_);
    interface_ = std::make_shared<ROS2Interface>(nh_);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_);
}

ROS2SimulationManager::~ROS2SimulationManager()
{
}

uint64_t ROS2SimulationManager::getSimulationClock() const
{
    rclcpp::Time now = rclcpp::Clock(RCL_ROS_TIME).now();
    return now.nanoseconds()/1000;
}

void ROS2SimulationManager::SimulationClockSleep(uint64_t us)
{
    rclcpp::Duration duration(0, (uint32_t)us*1000);
    rclcpp::Clock(RCL_ROS_TIME).sleep_for(duration);
}

std::map<std::string, rclcpp::ServiceBase::SharedPtr>& ROS2SimulationManager::getServices()
{
    return srvs_;
}

std::map<std::string, rclcpp::PublisherBase::SharedPtr>& ROS2SimulationManager::getPublishers()
{
    return pubs_;
}
    
std::map<std::string, rclcpp::SubscriptionBase::SharedPtr>& ROS2SimulationManager::getSubscribers()
{
    return subs_;
}

std::map<std::string, image_transport::Publisher>& ROS2SimulationManager::getImagePublishers()
{
    return imgPubs_;
}

std::shared_ptr<image_transport::ImageTransport> ROS2SimulationManager::getImageTransportHandle()
{
    return it_;
}

std::map<std::string, std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr>>& ROS2SimulationManager::getCameraMsgPrototypes()
{
    return cameraMsgPrototypes_;
}

std::map<std::string, std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::Image::SharedPtr>>& ROS2SimulationManager::getSonarMsgPrototypes()
{
    return sonarMsgPrototypes_;
}

void ROS2SimulationManager::AddROS2Robot(const std::shared_ptr<ROS2Robot>& robot)
{
    rosRobots_.push_back(robot);
}

void ROS2SimulationManager::BuildScenario()
{
    // Run parser
    ROS2ScenarioParser parser(this, nh_);
    bool success = parser.Parse(scenarioPath_);

    // Save log
    std::string logPath = rclcpp::get_logging_directory().string() + "/stonefish_ros2_parser.log";
    bool success2 = parser.SaveLog(logPath);

    if(!success)
    {
        RCLCPP_ERROR(nh_->get_logger(), "Parsing of scenario file '%s' failed!", scenarioPath_.c_str());
        if(success2)
            RCLCPP_ERROR(nh_->get_logger(), "For more information check the parser log file '%s'.", logPath.c_str());
    }

    if(!success2)
        RCLCPP_ERROR(nh_->get_logger(), "Parser log file '%s' could not be saved!", logPath.c_str());

    // Standard services
    srvs_["enable_currents"] = nh_->create_service<std_srvs::srv::Trigger>("enable_currents", std::bind(&ROS2SimulationManager::EnableCurrentsService, this, _1, _2));
    srvs_["disable_currents"] = nh_->create_service<std_srvs::srv::Trigger>("disable_currents", std::bind(&ROS2SimulationManager::DisableCurrentsService, this, _1, _2));
}

void ROS2SimulationManager::DestroyScenario()
{
    for(auto it = imgPubs_.begin(); it != imgPubs_.end(); ++it)
        it->second.shutdown();

    pubs_.clear();
    imgPubs_.clear();
    subs_.clear();
    srvs_.clear();
    cameraMsgPrototypes_.clear();
    sonarMsgPrototypes_.clear();
    rosRobots_.clear();

    SimulationManager::DestroyScenario();
}
	
void ROS2SimulationManager::SimulationStepCompleted(Scalar timeStep)
{
    (void)timeStep; // Suppress warning

    ////////////////////////////////////////SENSORS//////////////////////////////////////////////
    unsigned int id = 0;
    Sensor* sensor;
    while((sensor = getSensor(id++)) != nullptr)
    {
        if(!sensor->isNewDataAvailable())
            continue;

        if(sensor->getType() != SensorType::VISION)
        {
            if(pubs_.find(sensor->getName()) == pubs_.end())
                continue;

            switch(((ScalarSensor*)sensor)->getScalarSensorType())
            {
                case ScalarSensorType::ACC:
                    interface_->PublishAccelerometer(pubs_.at(sensor->getName()), (Accelerometer*)sensor);
                    break;

                case ScalarSensorType::GYRO:
                    interface_->PublishGyroscope(pubs_.at(sensor->getName()), (Gyroscope*)sensor);
                    break;

                case ScalarSensorType::IMU:
                    interface_->PublishIMU(pubs_.at(sensor->getName()), (IMU*)sensor);
                    break;

                case ScalarSensorType::ODOM:
                    interface_->PublishOdometry(pubs_.at(sensor->getName()), (Odometry*)sensor);
                    break;

                case ScalarSensorType::DVL:
                {
                    interface_->PublishDVL(pubs_.at(sensor->getName()), (DVL*)sensor);
                    if(pubs_.find(sensor->getName() + "/altitude") != pubs_.end())
                        interface_->PublishDVLAltitude(pubs_.at(sensor->getName() + "/altitude"), (DVL*)sensor);
                }
                    break;

                case ScalarSensorType::INS:
                {
                    interface_->PublishINS(pubs_.at(sensor->getName()), (INS*)sensor);
                    if(pubs_.find(sensor->getName() + "/odometry") != pubs_.end())
                        interface_->PublishINSOdometry(pubs_.at(sensor->getName() + "/odometry"), (INS*)sensor);
                }
                    break;

                case ScalarSensorType::GPS:
                    interface_->PublishGPS(pubs_.at(sensor->getName()), (GPS*)sensor);
                    break;

                case ScalarSensorType::PRESSURE:
                    interface_->PublishPressure(pubs_.at(sensor->getName()), (Pressure*)sensor);
                    break;

                case ScalarSensorType::FT:
                    interface_->PublishForceTorque(pubs_.at(sensor->getName()), (ForceTorque*)sensor);
                    break;

                case ScalarSensorType::ENCODER:
                    interface_->PublishEncoder(pubs_.at(sensor->getName()), (RotaryEncoder*)sensor);
                    break;

                case ScalarSensorType::MULTIBEAM:
                {
                    interface_->PublishMultibeam(pubs_.at(sensor->getName()), (Multibeam*)sensor);
                    if(pubs_.find(sensor->getName() + "/pcl") != pubs_.end())
                        interface_->PublishMultibeamPCL(pubs_.at(sensor->getName() + "/pcl"), (Multibeam*)sensor);
                }
                    break;

                case ScalarSensorType::PROFILER:
                    interface_->PublishProfiler(pubs_.at(sensor->getName()), (Profiler*)sensor);
                    break;

                default:
                    break;
            }
        }

        sensor->MarkDataOld();
    }

    ///////////////////////////////////////COMMS///////////////////////////////////////////////////
    id = 0;
    Comm* comm;
    while((comm = getComm(id++)) != nullptr)
    {
        if(!comm->isNewDataAvailable())
            continue;

        if(pubs_.find(comm->getName()) == pubs_.end())
            continue;

        switch(comm->getType())
        {
            case CommType::USBL:
                interface_->PublishUSBL(pubs_.at(comm->getName()), pubs_.at(comm->getName() + "/beacon_info"), (USBL*)comm);
                comm->MarkDataOld();
                break;

            default:
                break;
        }
    }

    //////////////////////////////////////TRAJECTORIES/////////////////////////////////////////////
    id = 0;
    Entity* ent;
    while((ent = getEntity(id++)) != nullptr)
    {
        if(ent->getType() == EntityType::ANIMATED)
        {
            if(pubs_.find(ent->getName() + "/odometry") == pubs_.end())
                continue;

            interface_->PublishTrajectoryState(pubs_.at(ent->getName() + "/odometry"), pubs_.at(ent->getName() + "/iteration"), (AnimatedEntity*)ent);
        }
    }

    //////////////////////////////////////CONTACTS/////////////////////////////////////////////////
    id = 0;
    Contact* cnt;
    while((cnt = getContact(id++)) != nullptr)
    {
        if(!cnt->isNewDataAvailable())
            continue;

        if(pubs_.find(cnt->getName()) != pubs_.end())
        {
            interface_->PublishContact(pubs_[cnt->getName()], cnt);
            cnt->MarkDataOld();
        }
    }

    //////////////////////////////////////WORLD TRANSFORMS/////////////////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(rosRobots_[i]->publishBaseLinkTransform_)
            interface_->PublishTF(tf_, rosRobots_[i]->robot_->getTransform(), rclcpp::Clock(RCL_ROS_TIME).now(), "world_ned", rosRobots_[i]->robot_->getName() + "/base_link");
    }

    //////////////////////////////////////SERVOS(JOINTS)/////////////////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(pubs_.find(rosRobots_[i]->robot_->getName() + "/servos") != pubs_.end())
        {
            unsigned int aID = 0;
            Actuator* actuator;
            Servo* srv;
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            msg.header.frame_id = rosRobots_[i]->robot_->getName();
            
            while((actuator = rosRobots_[i]->robot_->getActuator(aID++)) != nullptr)
            {
                if(actuator->getType() == ActuatorType::SERVO)
                {
                    srv = (Servo*)actuator;
                    msg.name.push_back(srv->getJointName());
                    msg.position.push_back(srv->getPosition());
                    msg.velocity.push_back(srv->getVelocity());
                    msg.effort.push_back(srv->getEffort());
                }
            }
            if(msg.name.size() > 0)
                std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(
                    pubs_.at(rosRobots_[i]->robot_->getName() + "/servos")
                )->publish(msg);
        }

        if(rosRobots_[i]->thrusterSetpoints_.size() != 0
           && pubs_.find(rosRobots_[i]->robot_->getName() + "/thrusters") != pubs_.end())
        {
            unsigned int aID = 0;
            unsigned int thID = 0;
            Actuator* actuator;
            Thruster* th;
            stonefish_ros2::msg::ThrusterState msg;
            msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            msg.header.frame_id = rosRobots_[i]->robot_->getName();
            msg.setpoint.resize(rosRobots_[i]->thrusterSetpoints_.size());
            msg.rpm.resize(rosRobots_[i]->thrusterSetpoints_.size());
            msg.thrust.resize(rosRobots_[i]->thrusterSetpoints_.size());
            msg.torque.resize(rosRobots_[i]->thrusterSetpoints_.size());

            while((actuator = rosRobots_[i]->robot_->getActuator(aID++)) != nullptr)
            {
                if(actuator->getType() == ActuatorType::THRUSTER)
                {
                    th = (Thruster*)actuator;
                    msg.setpoint[thID] = th->getSetpoint();
                    msg.rpm[thID] = th->getOmega()/(Scalar(2)*M_PI)*Scalar(60);
                    msg.thrust[thID] = th->getThrust();
                    msg.torque[thID] = th->getTorque();
                    ++thID;

                    if(thID == rosRobots_[i]->thrusterSetpoints_.size())
                        break;
                }
            }
            std::static_pointer_cast<rclcpp::Publisher<stonefish_ros2::msg::ThrusterState>>(
                    pubs_.at(rosRobots_[i]->robot_->getName() + "/thrusters")
                )->publish(msg);
        }
    }

    //////////////////////////////////////////////ACTUATORS//////////////////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        unsigned int aID = 0;
        Actuator* actuator;
        unsigned int thID = 0;
        unsigned int propID = 0;
        unsigned int rudderID = 0;

        while((actuator = rosRobots_[i]->robot_->getActuator(aID++)) != nullptr)
        {
            switch(actuator->getType())
            {
                case ActuatorType::PUSH:
                    ((Push*)actuator)->setForce(rosRobots_[i]->thrusterSetpoints_[thID++]);
                    break;

                case ActuatorType::THRUSTER:
                    ((Thruster*)actuator)->setSetpoint(rosRobots_[i]->thrusterSetpoints_[thID++]);
                    break;

                case ActuatorType::PROPELLER:
                    ((Propeller*)actuator)->setSetpoint(rosRobots_[i]->propellerSetpoints_[propID++]);
                    break;

                case ActuatorType::RUDDER:
                    ((Rudder*)actuator)->setSetpoint(rosRobots_[i]->rudderSetpoints_[rudderID++]);
                    break;

                case ActuatorType::SERVO:
                {
                    if(rosRobots_[i]->servoSetpoints_.size() == 0)
                        continue;

                    auto it = rosRobots_[i]->servoSetpoints_.find(((Servo*)actuator)->getJointName());
                    if(it != rosRobots_[i]->servoSetpoints_.end())
                    {
                        if(it->second.first == ServoControlMode::VELOCITY)
                        {
                            ((Servo*)actuator)->setControlMode(ServoControlMode::VELOCITY);
                            ((Servo*)actuator)->setDesiredVelocity(it->second.second);
                        }
                        else if(it->second.first == ServoControlMode::POSITION)
                        {
                            ((Servo*)actuator)->setControlMode(ServoControlMode::POSITION);
                            ((Servo*)actuator)->setDesiredPosition(it->second.second);
                        }
                    }
                }
                    break;

                case ActuatorType::VBS:
                {
                    auto it = pubs_.find(actuator->getName());
                    if(it != pubs_.end())
                    {
                        std_msgs::msg::Float64 msg;
                        msg.data = ((VariableBuoyancy*)actuator)->getLiquidVolume();
                        std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float64>>(it->second)->publish(msg);
                    }
                }
                    break;

                case ActuatorType::SUCTION_CUP:
                {
                    auto it = pubs_.find(actuator->getName());
                    if(it != pubs_.end())
                    {
                        std_msgs::msg::Bool msg;
                        msg.data = ((SuctionCup*)actuator)->getPump();
                        std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::Bool>>(it->second)->publish(msg);
                    }
                }
                    break;

                default:
                    break;
            }
        }
    }

    /////////////////////////////////// DEBUG //////////////////////////////////////////
    auto hydroLambda = [](Robot* r)
    {
        std::stringstream output;
        size_t lID = 0;
        SolidEntity* link;
        sf::Vector3 Fb, Tb, Fd, Td, Fs, Ts;
    
        while((link = r->getLink(lID++)) != nullptr)
        {
            link->getHydrodynamicForces(Fb, Tb, Fd, Td, Fs, Ts);
            output << "[" << link->getName() << "]" << std::endl;
            output << "Buoyancy force[N]: " << Fb.x() << ", " << Fb.y() << ", " << Fb.z() << std::endl;
            output << "Damping force[N]: " << Fd.x() << ", " << Fd.y() << ", " << Fd.z() << std::endl;
            output << "Skin friction[N]: " << Fs.x() << ", " << Fs.y() << ", " << Fs.z() << std::endl;
        }
        return output.str();
    };

    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        Robot* r = rosRobots_[i]->robot_;
        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "===== Hydrodynamics for " << r->getName() << " =====\n" << hydroLambda(r));
    }
}

void ROS2SimulationManager::ColorCameraImageReady(ColorCamera* cam)
{
    //Fill in the image message
    sensor_msgs::msg::Image::SharedPtr img = cameraMsgPrototypes_[cam->getName()].first;
    img->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    memcpy(img->data.data(), (uint8_t*)cam->getImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::msg::CameraInfo::SharedPtr info = cameraMsgPrototypes_[cam->getName()].second;
    info->header.stamp = img->header.stamp;

    //Publish messages
    imgPubs_.at(cam->getName()).publish(img);
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(pubs_.at(cam->getName() + "/info"))->publish(*info);
}

void ROS2SimulationManager::DepthCameraImageReady(DepthCamera* cam)
{
    //Fill in the image message
    sensor_msgs::msg::Image::SharedPtr img = cameraMsgPrototypes_[cam->getName()].first;
    img->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    memcpy(img->data.data(), (float*)cam->getImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::msg::CameraInfo::SharedPtr info = cameraMsgPrototypes_[cam->getName()].second;
    info->header.stamp = img->header.stamp;

    //Publish messages
    imgPubs_.at(cam->getName()).publish(img);
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(pubs_.at(cam->getName() + "/info"))->publish(*info);
}

void ROS2SimulationManager::Multibeam2ScanReady(Multibeam2* mb)
{
    interface_->PublishMultibeam2(pubs_.at(mb->getName()), mb);
}

void ROS2SimulationManager::FLSScanReady(FLS* fls)
{
    //Fill in the data message
    sensor_msgs::msg::Image::SharedPtr img = sonarMsgPrototypes_[fls->getName()].first;
    img->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    memcpy(img->data.data(), (uint8_t*)fls->getImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr disp = sonarMsgPrototypes_[fls->getName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), (uint8_t*)fls->getDisplayDataPointer(), disp->step * disp->height);

    //Publish messages
    imgPubs_.at(fls->getName()).publish(img);
    imgPubs_.at(fls->getName() + "/display").publish(disp);

}

void ROS2SimulationManager::SSSScanReady(SSS* sss)
{
    //Fill in the data message
    sensor_msgs::msg::Image::SharedPtr img = sonarMsgPrototypes_[sss->getName()].first;
    img->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    memcpy(img->data.data(), (uint8_t*)sss->getImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr disp = sonarMsgPrototypes_[sss->getName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), (uint8_t*)sss->getDisplayDataPointer(), disp->step * disp->height);

    //Publish messages
    imgPubs_.at(sss->getName()).publish(img);
    imgPubs_.at(sss->getName() + "/display").publish(disp);
}

void ROS2SimulationManager::MSISScanReady(MSIS* msis)
{
    //Fill in the data message
    sensor_msgs::msg::Image::SharedPtr img = sonarMsgPrototypes_[msis->getName()].first;
    img->header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    memcpy(img->data.data(), (uint8_t*)msis->getImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr disp = sonarMsgPrototypes_[msis->getName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), (uint8_t*)msis->getDisplayDataPointer(), disp->step * disp->height);

    //Publish messages
    imgPubs_.at(msis->getName()).publish(img);
    imgPubs_.at(msis->getName() + "/display").publish(disp);
}

void ROS2SimulationManager::EnableCurrentsService(const std_srvs::srv::Trigger::Request::SharedPtr req, 
                                           std_srvs::srv::Trigger::Response::SharedPtr res)
{
    (void)req;
    getOcean()->EnableCurrents();
    res->message = "Ocean current simulation enabled.";
    res->success = true;
}

void ROS2SimulationManager::DisableCurrentsService(const std_srvs::srv::Trigger::Request::SharedPtr req, 
                                            std_srvs::srv::Trigger::Response::SharedPtr res)
{   
    (void)req;
    getOcean()->DisableCurrents();
    res->message = "Ocean current simulation disabled.";
    res->success = true;
}

void ROS2SimulationManager::UniformVFCallback(const geometry_msgs::msg::Vector3::SharedPtr msg, Uniform* vf)
{
    vf->setVelocity(Vector3(msg->x, msg->y, msg->z));   
}

void ROS2SimulationManager::JetVFCallback(const std_msgs::msg::Float64::SharedPtr msg, Jet* vf)
{
    vf->setOutletVelocity(msg->data);
}

void ROS2SimulationManager::ActuatorOriginCallback(const geometry_msgs::msg::Transform::SharedPtr msg, Actuator* act)
{
    sf::Transform T;
    T.setOrigin(sf::Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
    T.setRotation(sf::Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));

    switch(act->getType())
    {
        case ActuatorType::PUSH:
        case ActuatorType::THRUSTER:
        case ActuatorType::PROPELLER:
        case ActuatorType::VBS:
        case ActuatorType::LIGHT:
            ((LinkActuator*)act)->setRelativeActuatorFrame(T);
            break;

        default:
            RCLCPP_WARN_STREAM(nh_->get_logger(), "Live update of origin frame of actuator '" << act->getName() << "' not supported!");
            break;
    }
}

void ROS2SimulationManager::TrajectoryCallback(const nav_msgs::msg::Odometry::SharedPtr msg, ManualTrajectory* tr)
{
    sf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    sf::Vector3 p(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    sf::Vector3 v(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    sf::Vector3 omega(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    tr->setTransform(Transform(q, p));
    tr->setLinearVelocity(v);
    tr->setAngularVelocity(omega);
}

void ROS2SimulationManager::VBSCallback(const std_msgs::msg::Float64::SharedPtr msg, VariableBuoyancy* act)
{
    act->setFlowRate(msg->data);
}

void ROS2SimulationManager::SuctionCupService(const std_srvs::srv::SetBool::Request::SharedPtr req,
                            std_srvs::srv::SetBool::Response::SharedPtr res, SuctionCup* suction)
{
    suction->setPump(req->data);
    if(req->data)
        res->message = "Pump turned on.";
    else 
        res->message = "Pump turned off.";
    res->success = true;
}

void ROS2SimulationManager::SensorOriginCallback(const geometry_msgs::msg::Transform::SharedPtr msg, Sensor* sens)
{
    sf::Transform T;
    T.setOrigin(sf::Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
    T.setRotation(sf::Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));

    switch(sens->getType())
    {
        case SensorType::LINK:
            ((LinkSensor*)sens)->setRelativeSensorFrame(T);
            break;

        case SensorType::VISION:
            ((VisionSensor*)sens)->setRelativeSensorFrame(T);
            break;

        default:
            RCLCPP_WARN_STREAM(nh_->get_logger(), "Live update of origin frame of sensor '" << sens->getName() << "' not supported!");
            break;
    }
}

void ROS2SimulationManager::FLSService(const stonefish_ros2::srv::SonarSettings::Request::SharedPtr req,
                                    stonefish_ros2::srv::SonarSettings::Response::SharedPtr res, FLS* fls)
{
    if(req->range_min <= 0 || req->range_max <= 0 || req->gain <= 0 || req->range_min >= req->range_max)
    {
        res->success = false;
        res->message = "Wrong sonar settings!";
    }
    else
    {
        fls->setRangeMax(req->range_max);
        fls->setRangeMin(req->range_min);
        fls->setGain(req->gain);
        res->success = true;
        res->message = "New sonar settings applied.";
    }
}

void ROS2SimulationManager::SSSService(const stonefish_ros2::srv::SonarSettings::Request::SharedPtr req,
                                    stonefish_ros2::srv::SonarSettings::Response::SharedPtr res, SSS* sss)
{
    if(req->range_min <= 0 || req->range_max <= 0 || req->gain <= 0 || req->range_min >= req->range_max)
    {
        res->success = false;
        res->message = "Wrong sonar settings!";
    }
    else
    {
        sss->setRangeMax(req->range_max);
        sss->setRangeMin(req->range_min);
        sss->setGain(req->gain);
        res->success = true;
        res->message = "New sonar settings applied.";
    }
}

void ROS2SimulationManager::MSISService(const stonefish_ros2::srv::SonarSettings2::Request::SharedPtr req,
                                    stonefish_ros2::srv::SonarSettings2::Response::SharedPtr res, MSIS* msis)
{
    if(req->range_min <= 0 || req->range_max <= 0 || req->gain <= 0
       || req->range_min >= req->range_max
       || req->rotation_min < -180.0
       || req->rotation_max > 180.0
       || req->rotation_min >= req->rotation_max)
    {
        res->success = false;
        res->message = "Wrong sonar settings!";
    }
    else
    {
        msis->setRangeMax(req->range_max);
        msis->setRangeMin(req->range_min);
        msis->setGain(req->gain);
        msis->setRotationLimits(req->rotation_min, req->rotation_max);
        res->success = true;
        res->message = "New sonar settings applied.";
    }
}

void ROS2SimulationManager::ThrustersCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, std::shared_ptr<ROS2Robot> robot)
{
    if(msg->data.size() != robot->thrusterSetpoints_.size())
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Wrong number of thruster setpoints for robot: " << robot->robot_->getName());
        return;
    }
    for(size_t i=0; i<robot->thrusterSetpoints_.size(); ++i)
        robot->thrusterSetpoints_[i] = msg->data[i];
}

void ROS2SimulationManager::PropellersCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, std::shared_ptr<ROS2Robot> robot)
{
    if(msg->data.size() != robot->propellerSetpoints_.size())
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Wrong number of propeller setpoints for robot: " << robot->robot_->getName());
        return;
    }
    for(size_t i=0; i<robot->propellerSetpoints_.size(); ++i)
        robot->propellerSetpoints_[i] = msg->data[i];
}

void ROS2SimulationManager::RuddersCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, std::shared_ptr<ROS2Robot> robot)
{
    if(msg->data.size() != robot->rudderSetpoints_.size())
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Wrong number of rudder setpoints for robot: " << robot->robot_->getName());
        return;
    }
    for(size_t i=0; i<robot->rudderSetpoints_.size(); ++i)
        robot->rudderSetpoints_[i] = msg->data[i];
}

void ROS2SimulationManager::ServosCallback(const sensor_msgs::msg::JointState::SharedPtr msg, std::shared_ptr<ROS2Robot> robot)
{
    if(msg->name.size() == 0)
    {
        RCLCPP_ERROR(nh_->get_logger(), "Desired joint state message is missing joint names!");
        return;
    }

    if(msg->position.size() > 0)
    {
        for(size_t i=0; i<msg->position.size(); ++i)
        {
            try
            {
                robot->servoSetpoints_.at(msg->name[i]) = std::make_pair(ServoControlMode::POSITION, (Scalar)msg->position[i]);
            }
            catch(const std::out_of_range& e)
            {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid joint name in desired joint state message: " << msg->name[i]);
            }
        }
    }
    else if(msg->velocity.size() > 0)
    {
        for(size_t i=0; i<msg->velocity.size(); ++i)
        {
            try
            {
                robot->servoSetpoints_.at(msg->name[i]) = std::make_pair(ServoControlMode::VELOCITY, (Scalar)msg->velocity[i]);
            }
            catch(const std::out_of_range& e)
            {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid joint name in desired joint state message: " << msg->name[i]);
            }
        }
    }
    else if(msg->effort.size() > 0)
    {
        RCLCPP_ERROR(nh_->get_logger(), "No effort control mode implemented in simulation!");
    }
}

void ROS2SimulationManager::JointCallback(const std_msgs::msg::Float64::SharedPtr msg,  std::shared_ptr<ROS2Robot> robot, 
                                                            ServoControlMode mode, const std::string& jointName)
{
    try
    {
        robot->servoSetpoints_.at(jointName) = std::make_pair(mode, (Scalar)msg->data);
    }
    catch(const std::out_of_range& e)
    {
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid joint name: " << jointName);
    }
}

void ROS2SimulationManager::JointGroupCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg,  std::shared_ptr<ROS2Robot> robot, 
                                                            ServoControlMode mode, const std::vector<std::string>& jointNames)
{
    if(msg->data.size() != jointNames.size())
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Wrong size of joint group message! Required: " << jointNames.size() << " Received: " << msg->data.size());
        return;
    }
    for(size_t i=0; i<jointNames.size(); ++i)
    {
        try
        {
            robot->servoSetpoints_.at(jointNames[i]) = std::make_pair(mode, (Scalar)msg->data[i]);
        }
        catch(const std::out_of_range& e)
        {
            RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid joint name: " << jointNames[i]);
        }
    }
}

}