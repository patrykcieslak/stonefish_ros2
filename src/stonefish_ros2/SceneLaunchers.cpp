//
//  SceneLaunchers.cpp
//  stonefish_ros2
//

#include "stonefish_ros2/SceneLaunchers.h"

#include <btBulletDynamicsCommon.h>
#include <BulletSoftBody/btSoftMultiBodyDynamicsWorld.h>
#include <Stonefish/core/Robot.h>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/entities/SolidEntity.h>

namespace sf
{

namespace
{
    btRigidBody* FindRigidBody(SimulationManager* sm, const SolidEntity* solid)
    {
        auto& objects = sm->getDynamicsWorld()->getCollisionObjectArray();
        for(int i = 0; i < objects.size(); ++i)
            if(objects[i]->getUserPointer() == solid)
                return btRigidBody::upcast(objects[i]);
        return nullptr;
    }

    void Place(btRigidBody* body, const Transform& T, const Vector3& velocity)
    {
        body->setCenterOfMassTransform(T);
        body->getMotionState()->setWorldTransform(T);
        body->setLinearVelocity(velocity);
        body->setAngularVelocity(Vector3(0, 0, 0));
        body->clearForces();
        body->activate(true);
    }
}

SceneLaunchers::SceneLaunchers(SimulationManager* sm, const std::shared_ptr<rclcpp::Node>& nh)
    : sm_(sm), nh_(nh), tfBuffer_(nh->get_clock()), tfListener_(tfBuffer_, nh, false)
{
}

void SceneLaunchers::Add(const std::string& service, const std::string& robot, const std::string& objects,
                         int count, double speed, const std::string& baseFrame, const std::string& frame)
{
    auto l = std::make_unique<Launcher>();
    l->service = service;
    l->robot = robot;
    l->objects = objects;
    l->count = count;
    l->speed = speed;
    l->baseFrame = baseFrame;
    l->frame = frame;
    Launcher* raw = l.get();
    l->srv = nh_->create_service<std_srvs::srv::Trigger>(service,
        [this, raw](const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res)
        {
            try
            {
                const auto t = tfBuffer_.lookupTransform(raw->baseFrame, raw->frame, tf2::TimePointZero).transform;
                const Transform mount(Quaternion(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w),
                                      Vector3(t.translation.x, t.translation.y, t.translation.z));
                std::lock_guard<std::mutex> lock(raw->mutex);
                raw->requested.push_back(mount);
                res->success = true;
                res->message = "fired";
            }
            catch(const tf2::TransformException& e)
            {
                res->success = false;
                res->message = e.what();
            }
        });
    launchers_.push_back(std::move(l));
}

void SceneLaunchers::Clear()
{
    launchers_.clear();
}

bool SceneLaunchers::Resolve(Launcher& l)
{
    if(!l.bodies.empty())
        return true;

    Robot* robot = sm_->getRobot(l.robot);
    if(robot == nullptr)
        return false;

    std::vector<Body> bodies;
    std::vector<SolidEntity*> solids;
    for(int i = 0; i < l.count; ++i)
    {
        auto* solid = dynamic_cast<SolidEntity*>(sm_->getEntity(l.objects + "_" + std::to_string(i)));
        btRigidBody* rigid = solid != nullptr ? FindRigidBody(sm_, solid) : nullptr;
        if(rigid == nullptr)
        {
            RCLCPP_ERROR(nh_->get_logger(), "%s: no <dynamic> object %s_%d in the scene", l.service.c_str(), l.objects.c_str(), i);
            l.count = 0;
            return false;
        }
        Body b;
        b.rigid = rigid;
        //So light and thin that the drag torque blows up the solver, and it should fly straight anyway
        rigid->setAngularFactor(Vector3(0, 0, 0));
        b.originToCg = solid->getOTransform().inverse() * solid->getCGTransform();
        //Out of the way, above the pool, until it is fired
        b.parked = Transform(Quaternion::getIdentity(), Vector3(1000.0 + 2.0 * i, double(launchers_.size()) * 2.0, -100.0));
        bodies.push_back(b);
        solids.push_back(solid);

        //It leaves from inside the vehicle
        SolidEntity* link;
        for(size_t k = 0; (link = robot->getLink(k)) != nullptr; ++k)
            sm_->DisableCollision(solid, link);
    }
    for(size_t a = 0; a < solids.size(); ++a)
        for(size_t b = a + 1; b < solids.size(); ++b)
            sm_->DisableCollision(solids[a], solids[b]);
    l.bodies = std::move(bodies);
    return true;
}

void SceneLaunchers::Step()
{
    for(auto& lp : launchers_)
    {
        Launcher& l = *lp;
        if(l.count == 0 || !Resolve(l))
            continue;

        for(Body& b : l.bodies)
            if(!b.inWater)
                Place(b.rigid, b.parked, Vector3(0, 0, 0));

        std::vector<Transform> shots;
        {
            std::lock_guard<std::mutex> lock(l.mutex);
            shots.swap(l.requested);
        }
        for(const Transform& mount : shots)
        {
            Robot* robot = sm_->getRobot(l.robot);
            SolidEntity* base = robot->getLink(size_t(0));
            const Transform T = robot->getTransform() * mount;
            Body& b = l.bodies[l.next];
            Place(b.rigid, T * b.originToCg, base->getLinearVelocity() + l.speed * T.getBasis().getColumn(0));
            b.inWater = true;
            l.next = (l.next + 1) % l.bodies.size();
            RCLCPP_INFO(nh_->get_logger(), "%s: fired from (%.2f, %.2f, %.2f)", l.service.c_str(),
                        T.getOrigin().x(), T.getOrigin().y(), T.getOrigin().z());
        }
    }
}

}
