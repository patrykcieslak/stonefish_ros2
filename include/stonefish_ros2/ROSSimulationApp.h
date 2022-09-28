/*    
    This file is a part of stonefish_ros.

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
//  ROSSimulationApp.h
//  stonefish_ros2
//
//  Created by Patryk Cieslak on 28/09/22.
//  Copyright (c) 2022 Patryk Cieslak. All rights reserved.
//

#ifndef __Stonefish_ROSSimulationApp__
#define __Stonefish_ROSSimulationApp__

#include <Stonefish/core/SimulationApp.h>
#include "rclcpp/rclcpp.hpp"

namespace sf
{
    class ROSSimulationApp : public SimulationApp, rclcpp::Node
    {
        


    };
}

#endif