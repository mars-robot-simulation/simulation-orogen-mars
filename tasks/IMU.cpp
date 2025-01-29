/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "IMU.hpp"
#include "Plugin.hpp"

#include <base/Logging.hpp>

#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_utils/mathUtils.h>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>

using namespace mars;

IMU::IMU(std::string const& name)
    : IMUBase(name),
    imu{boost::none}
{
}

IMU::IMU(std::string const& name, RTT::ExecutionEngine* engine)
    : IMUBase(name, engine), imu{boost::none}
{
}

IMU::~IMU()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See IMU.hpp for more detailed
// documentation about them.

bool IMU::configureHook()
{
    if (!IMUBase::configureHook())
        return false;

    // check if the imu frame is in the graph
    std::string prefix = _robot_name.value();
    std::string imuName = prefix + _name.value();

    // in mars the imu frame will be stored as DynamicObject
    // in the graph frame with prefix + name syntax
    std::string imuFrameId = imuName;
    if (!control->envireGraph->containsFrame(imuFrameId))
    {
        LOG_ERROR_S << "There is no frame '" << imuFrameId << "' in the graph";
        return false;
    }

    using DynamicObjectItem = envire::core::Item<interfaces::DynamicObjectItem>;

    if (!control->envireGraph->containsItems<DynamicObjectItem>(imuFrameId))
    {
        LOG_ERROR_S << "There is no dynamic object in the frame '" << imuFrameId << "'";
        return false;
    }

    // find the corresponding dynamic object in the frame
    using DynamicObjectItemItr = envire::core::EnvireGraph::ItemIterator<DynamicObjectItem>;
    DynamicObjectItemItr begin_itr, end_itr;
    boost::tie(begin_itr, end_itr) = control->envireGraph->getItems<DynamicObjectItem>(imuFrameId);

    bool imuFound = false;
    while (begin_itr != end_itr && imuFound == false)
    {
        auto& dynamicObject = begin_itr->getData().dynamicObject;
        if (dynamicObject->getName() == imuName)
        {
            imuFound = true;
            imu = dynamicObject;
            break;
        }
        begin_itr++;
    }

    if (!imuFound) {
        LOG_ERROR_S << "There is no dynamic object '" << imuName << "' in the frame '" << imuFrameId << "'";
        return false;
    }

    rbs.initSane();
    rbs.position.setZero();

    translation_noise = boost::normal_distribution<double>(0.0, _position_sigma.get());
    rotation_noise = boost::normal_distribution<double>(0.0, _orientation_sigma.get());
    velocity_noise = boost::normal_distribution<double>(0.0, _velocity_sigma.get());
    angular_velocity_noise = boost::normal_distribution<double>(0.0, _angular_velocity_sigma.get());

    return true;
}

bool IMU::startHook()
{
    if (! IMUBase::startHook())
        return false;
    return true;
}
void IMU::updateHook()
{
    IMUBase::updateHook();
}
void IMU::errorHook()
{
    IMUBase::errorHook();
}
void IMU::stopHook()
{
    IMUBase::stopHook();
}
void IMU::cleanupHook()
{
    imu.reset();

    IMUBase::cleanupHook();
}

void IMU::update( double time )
{
    if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning

    if (!imu.has_value())
    {
        exception();
        return;
    }
    if (imu->expired())
    {
        // This will happen, if the DynamicObject referenced by imu was removed from the envire graph. This occurs especially if the simulation was reset.
        exception();
        return;
    }

    auto validImu = imu->lock();

    //First make all invalid
    rbs.invalidate();

    rbs.time = getTime();
    rbs.sourceFrame = _imu_frame.value();
    rbs.targetFrame = _world_frame.value();
    if(_provide_orientation.get()){
        // TODO: mars1
        //rbs.orientation = control->nodes->getRotation( node_id ).normalized();
        utils::Quaternion q;
        validImu->getRotation(&q);
        rbs.orientation = q.normalized();
        rbs.cov_orientation = base::Matrix3d::Identity() * std::max(std::pow(rotation_noise.sigma(), 2), 1e-6);
        if( rotation_noise.sigma() > 0.0 )
        {
            // apply noise to orientation
            base::Orientation orientation_error = Eigen::AngleAxisd(rotation_noise(rnd_generator), Eigen::Vector3d::UnitX())*
                                                    Eigen::AngleAxisd(rotation_noise(rnd_generator), Eigen::Vector3d::UnitY()) *
                                                    Eigen::AngleAxisd(rotation_noise(rnd_generator), Eigen::Vector3d::UnitZ());
            rbs.orientation = orientation_error * rbs.orientation;
        }
        if(_provide_velocity.get())
        {
            // TODO: mars1
            //rbs.angular_velocity = rbs.orientation.conjugate() * control->nodes->getAngularVelocity( node_id);
            utils::Vector v;
            validImu->getAngularVelocity(&v);
            rbs.angular_velocity = rbs.orientation.conjugate() * v;
            rbs.cov_angular_velocity = base::Matrix3d::Identity() * std::max(std::pow(angular_velocity_noise.sigma(), 2), 1e-6);
            if( angular_velocity_noise.sigma() > 0.0 )
            {
                // apply noise to angular velocity
                rbs.angular_velocity = rbs.angular_velocity + base::Vector3d(angular_velocity_noise(rnd_generator), angular_velocity_noise(rnd_generator), angular_velocity_noise(rnd_generator));
            }
        }
    }


    if(_provide_position.get()){
        // TODO: mars1
        //rbs.position = control->nodes->getPosition( node_id );
        utils::Vector pos;
        validImu->getPosition(&pos);
        rbs.position = pos;
        rbs.cov_position = base::Matrix3d::Identity() * std::max(std::pow(translation_noise.sigma(), 2), 1e-6);
        if( translation_noise.sigma() > 0.0 )
        {
            // apply noise to position
            rbs.position = rbs.position + base::Vector3d(translation_noise(rnd_generator), translation_noise(rnd_generator), translation_noise(rnd_generator));
        }

        if(_provide_velocity.get()){
            // TODO: mars1
            //rbs.velocity = control->nodes->getLinearVelocity( node_id );
            utils::Vector v;
            validImu->getLinearVelocity(&v);
            rbs.velocity = v;
            rbs.cov_velocity = base::Matrix3d::Identity() * std::max(std::pow(velocity_noise.sigma(), 2), 1e-6);
            if( velocity_noise.sigma() > 0.0 )
            {
                // apply noise to velocity
                rbs.velocity = rbs.velocity + base::Vector3d(velocity_noise(rnd_generator), velocity_noise(rnd_generator), velocity_noise(rnd_generator));
            }
        }
    }

    _orientation.write( rbs );

    imusens.time = getTime();
    // transform acceleration and rotation to IMU frame:
    // TODO: mars1
    //base::Orientation orientation_con = control->nodes->getRotation( node_id ).normalized().conjugate();
    //imusens.acc  = orientation_con * (control->nodes->getLinearAcceleration( node_id ) - control->sim->getGravity());
    //imusens.gyro = orientation_con * control->nodes->getAngularVelocity( node_id);
    {
        utils::Quaternion q;
        validImu->getRotation(&q);
        base::Orientation orientation_con = q.normalized().conjugate();
        // TODO: add acceleration
        utils::Vector v_a, a_l;
        validImu->getLinearAcceleration(&a_l);
        imusens.acc = orientation_con * (a_l - control->sim->getGravity());
        validImu->getAngularVelocity(&v_a);
        imusens.gyro = orientation_con * v_a;
    }
    // TODO add noise?
    _calibrated_sensors.write( imusens );

}

