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
//  stonefish_simulator_nogpu.cpp
//  stonefish_ros2
//
//  Created by Patryk Cieslak on 05/10/23.
//  Copyright (c) 2023 Patryk Cieslak. All rights reserved.
//

#include "rclcpp/rclcpp.hpp"
#include <Stonefish/utils/SystemUtil.hpp>
#include "stonefish_ros2/ROS2SimulationManager.h"
#include "stonefish_ros2/ROS2ConsoleSimulationApp.h"
#include <std_msgs/msg/bool.hpp>

#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class StonefishConsoleNode : public rclcpp::Node
{
public:
    StonefishConsoleNode(const std::string& scenarioPath,
                           const std::string& dataPath,
                           sf::Scalar rate) 
                           : Node("stonefish_simulator_nogpu")
    {   
        pause_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "pause_simulation",
            std::bind(&StonefishConsoleNode::pause_simulation_callback, this, _1, _2));
        
        resume_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "resume_simulation",
            std::bind(&StonefishConsoleNode::resume_simulation_callback, this, _1, _2));

        step_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "step_simulation",
            std::bind(&StonefishConsoleNode::step_simulation_callback, this, _1, _2));

        sf::ROS2SimulationManager* manager = new sf::ROS2SimulationManager(rate, scenarioPath, std::shared_ptr<rclcpp::Node>(this));
        app_ = std::shared_ptr<sf::ROS2ConsoleSimulationApp>(new sf::ROS2ConsoleSimulationApp("Stonefish Simulator", dataPath, manager));
        app_->Startup();
    };

    void Shutdown()
    {
        app_->Shutdown();
    };

    void Step()
    {
        app_->Step();
    };

    void Pause()
    {
        app_->Pause();
    };

    void Resume()
    {
        app_->Resume();
    };

private:
    void pause_simulation_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (app_->getState() == sf::SimulationState::STOPPED)
        {
            res->success = false;
            res->message = "Simulation is not running.";
            return;
        }
        app_->Pause();
        res->success = true;
        res->message = "Simulation paused successfully.";
    }
    void resume_simulation_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (app_->getState() == sf::SimulationState::RUNNING)
        {
            res->success = false;
            res->message = "Simulation is already running.";
            return;
        }
        app_->Resume();
        res->success = true;
        res->message = "Simulation resumed successfully.";
    }

    void step_simulation_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        if (app_->getState() == sf::SimulationState::RUNNING)
        {
            res->success = false;
            res->message = "Simulation is already running, please stop before stepping.";
            return;
        }
        app_->Step();
        res->success = true;
        res->message = "Simulation stepped successfully.";
    }

    std::shared_ptr<sf::ROS2ConsoleSimulationApp> app_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr step_srv_;

};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    
    //Check number of command line arguments
	if(argc < 4)
	{
		std::cout << "Not enough command-line arguments provided!" << std::endl;
		exit(-1);
	}

    //Parse the arguments
    std::string dataPath = std::string(argv[1]) + "/";
    std::string scenarioPath(argv[2]);
    sf::Scalar rate = atof(argv[3]);

    // Start simulation
    std::shared_ptr<StonefishConsoleNode> node(new StonefishConsoleNode(scenarioPath, dataPath, rate));
    rclcpp::spin(node);
    node->Shutdown();
    rclcpp::shutdown();
    return 0;
}
