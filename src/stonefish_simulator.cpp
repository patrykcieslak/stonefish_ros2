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
//  stonefish_simulator.cpp
//  stonefish_ros2
//
//  Created by Patryk Cieslak on 02/10/23.
//  Copyright (c) 2023-2024 Patryk Cieslak. All rights reserved.
//

#include "rclcpp/rclcpp.hpp"
#include <Stonefish/utils/SystemUtil.hpp>
#include "stonefish_ros2/ROS2SimulationManager.h"
#include "stonefish_ros2/ROS2GraphicalSimulationApp.h"

#include "std_srvs/srv/trigger.hpp"
#include "stonefish_ros2/srv/set_jerlov.hpp"
#include "stonefish_ros2/srv/set_sun_params.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class StonefishNode : public rclcpp::Node
{
public:
    StonefishNode(const std::string& scenarioPath,
                           const std::string& dataPath, 
                           const sf::RenderSettings& s, 
                           const sf::HelperSettings& h,
                           sf::Scalar rate) 
                           : Node("stonefish_simulator")
    {   
        pause_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "pause_simulation",
            std::bind(&StonefishNode::pause_simulation_callback, this, _1, _2));
        
        resume_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "resume_simulation",
            std::bind(&StonefishNode::resume_simulation_callback, this, _1, _2));

        step_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "step_simulation",
            std::bind(&StonefishNode::step_simulation_callback, this, _1, _2));

        tick_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "tick_simulation",
            std::bind(&StonefishNode::tick_callback, this, _1, _2));

        set_jerlov_srv_ = this->create_service<stonefish_ros2::srv::SetJerlov>(
            "set_jerlov",
            std::bind(&StonefishNode::set_jerlov_callback, this, _1, _2));

        sf::ROS2SimulationManager* manager = new sf::ROS2SimulationManager(rate, scenarioPath, std::shared_ptr<rclcpp::Node>(this));
        app_ = std::shared_ptr<sf::ROS2GraphicalSimulationApp>(new sf::ROS2GraphicalSimulationApp("Stonefish Simulator", 
                                                                                                 dataPath, s, h, manager));
        app_->Startup();
        tickTimer_ = this->create_wall_timer(16667us, std::bind(&sf::ROS2GraphicalSimulationApp::Tick, app_));
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

    void tick_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        app_->Tick();
        res->success = true;
        res->message = "Simulation ticked successfully.";
    }

    void set_jerlov_callback(
        const std::shared_ptr<stonefish_ros2::srv::SetJerlov::Request> req,
        std::shared_ptr<stonefish_ros2::srv::SetJerlov::Response> res)
    {
        if (req->jerlov < 0.0f || req->jerlov > 1.0f)
        {
            res->success = false;
            res->message = "Invalid Jerlov water type. Must be between 0 and 1.";
            return;
        }
        app_->getSimulationManager()->getOcean()->setWaterType(req->jerlov);
        res->success = true;
        res->message = "Jerlov water type successfully set to %f." + std::to_string(req->jerlov);
    }
            
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr step_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr tick_srv_;
    rclcpp::Service<stonefish_ros2::srv::SetJerlov>::SharedPtr set_jerlov_srv_;
    rclcpp::Service<stonefish_ros2::srv::SetSunParams>::SharedPtr set_sun_params_srv_;

    std::shared_ptr<sf::ROS2GraphicalSimulationApp> app_;
    rclcpp::TimerBase::SharedPtr tickTimer_;

};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    
    //Check number of command line arguments
	if(argc < 7)
	{
		std::cout << "Not enough command-line arguments provided!" << std::endl;
		exit(-1);
	}

    //Parse the arguments
    std::string dataPath = std::string(argv[1]) + "/";
    std::string scenarioPath(argv[2]);
    sf::Scalar rate = atof(argv[3]);

	sf::RenderSettings s;
    s.windowW = atoi(argv[4]);
    s.windowH = atoi(argv[5]);

    std::string quality(argv[6]);
    if(quality == "low")
    {
        s.shadows = sf::RenderQuality::LOW;
        s.ao = sf::RenderQuality::DISABLED;
        s.atmosphere = sf::RenderQuality::LOW;
        s.ocean = sf::RenderQuality::LOW;
        s.aa = sf::RenderQuality::LOW;
        s.ssr = sf::RenderQuality::DISABLED;
    }
    else if(quality == "high")
    {
        s.shadows = sf::RenderQuality::HIGH;
        s.ao = sf::RenderQuality::HIGH;
        s.atmosphere = sf::RenderQuality::HIGH;
        s.ocean = sf::RenderQuality::HIGH;
        s.aa = sf::RenderQuality::HIGH;
        s.ssr = sf::RenderQuality::HIGH;
    }
    else // "medium"
    {
        s.shadows = sf::RenderQuality::MEDIUM;
        s.ao = sf::RenderQuality::MEDIUM;
        s.atmosphere = sf::RenderQuality::MEDIUM;
        s.ocean = sf::RenderQuality::MEDIUM;
        s.aa = sf::RenderQuality::MEDIUM;
        s.ssr = sf::RenderQuality::MEDIUM;
    }

    // Initialize GUI settings
    sf::HelperSettings h;
    h.showFluidDynamics = false;
    h.showCoordSys = false;
    h.showBulletDebugInfo = false;
    h.showSensors = false;
    h.showActuators = false;
    h.showForces = false;
	
    // Start simulation
    std::shared_ptr<StonefishNode> node(new StonefishNode(scenarioPath, dataPath, s, h, rate));
    rclcpp::spin(node);
    return 0;
}
