/*    
    This file is a part of stonefish_ros2.

    stonefish_ros2 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    stonefish_ros2 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  ROS2CommPluginInterface.h
//  stonefish_ros2
//
//  Created by Patryk Cieslak on 12/11/26.
//  Copyright (c) 2026 Patryk Cieslak. All rights reserved.

#pragma once

#include <rclcpp/rclcpp.hpp>

namespace sf
{
    //! A class representing an interface for communication device plugins in ROS2.
    class ROS2CommPluginInterface
    {
    public:
        //! A virtual destructor.
        virtual ~ROS2CommPluginInterface() = default;

        //! A pure virtual method for creating ROS2 publishers for the communication device plugin.
        /*!
         \param nh a shared pointer to the ROS2 node handle
         \param pubs a map to store the created publishers, with communication device names as keys
         \param pubTopics a map storing all parsed publisher topics
         */
        virtual void CreatePublishers(const std::shared_ptr<rclcpp::Node>& nh, 
            std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs, 
            std::string_view pubTopic) const = 0;

        //! A pure virtual method for publishing communication device data.
        /*!
         \param nh a shared pointer to the ROS2 node handle
         \param pubs a map of the available publishers, with communication device names as keys
         */
        virtual void Publish(const std::shared_ptr<rclcpp::Node>& nh, 
            std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr>& pubs) const = 0;
    };
}