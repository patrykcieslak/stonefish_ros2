.. _sensors:

=======
Sensors
=======

The *Stonefish* library implements a wide range of sensor simulations. Each of the sensors produces a specific output, that can be retrieved. The *stonefish_ros2* package implements automatic allocation of publishers and generation of appropriate messages, published when the new sensor data is available. The only definition that the user has to add to the standard definiton of a sensor, to use this functionality, is the name of the topic used by the publisher:

.. code-block:: xml

    <sensor name="..." type="...">
        <!-- standard sensor definitions -->
        <ros_publisher topic="/topic_name"/>
    </sensor>

For each sensor type a different message type is published. Some of the sensors need to publish on multiple topics. Specific information on all supported sensors is presented below.

Rotary encoder
==============

Message type: ``sensor_msgs::msg::JointState``

Force-torque (6-axis)
=====================

Message type: ``geometry_msgs::msg::WrenchStamped``

Odometry
========

Message type: ``nav_msgs::msg::Odometry``

IMU
===

Message type: ``sensor_msgs::msg::Imu``

GPS
===

Message type: ``sensor_msgs::msg::NavSatFix``

Doppler velocity log (DVL)
==========================

Message type: ``cola2_msgs::msg::DVL``

Second message type: ``sensor_msgs::msg::Range``

Pressure
========

Message type: ``sensor_msgs::msg::FluidPressure``

Multi-beam
==========

Message type: ``sensor_msgs::msg::LaserScan``

Color camera
============

Message type: ``sensor_msgs::msg::Image``

Second message type: ``sensor_msgs::msg::CameraInfo``

Depth camera
============

Message type: ``sensor_msgs::msg::Image``

Second message type: ``sensor_msgs::msg::CameraInfo``

Forward-looking sonar (FLS)
===========================

Message type: ``sensor_msgs::msg::Image``

Second message type: ``sensor_msgs::msg::Image``

Mechanical scanning imaging sonar (MSIS)
========================================

Message type: ``sensor_msgs::msg::Image``

Second message type: ``sensor_msgs::msg::Image``

Side-scan sonar (SSS)
=====================

Message type: ``sensor_msgs::msg::Image``

Second message type: ``sensor_msgs::msg::Image``

