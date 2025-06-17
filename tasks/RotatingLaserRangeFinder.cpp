/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "RotatingLaserRangeFinder.hpp"

//#include <mars/sim/RotatingRaySensor.h>
//#include <mars/interfaces/sim/SensorManagerInterface.h>

#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_utils/mathUtils.h>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>
#include <envire_types/sensors/RotatingRaySensor.hpp>
#include <mars_core/sensors/RotatingRaySensor.hpp>

#include <base/Logging.hpp>

using namespace mars;

RotatingLaserRangeFinder::RotatingLaserRangeFinder(std::string const& name)
    : RotatingLaserRangeFinderBase(name), mSensorID(0), mSensor(NULL)
{
}

RotatingLaserRangeFinder::RotatingLaserRangeFinder(std::string const& name, RTT::ExecutionEngine* engine)
    : RotatingLaserRangeFinderBase(name, engine), mSensorID(0), mSensor(NULL)
{
}

RotatingLaserRangeFinder::~RotatingLaserRangeFinder()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See RotatingLaserRangeFinder.hpp for more detailed
// documentation about them.

bool RotatingLaserRangeFinder::configureHook()
{
    if (! RotatingLaserRangeFinderBase::configureHook())
        return false;
    return true;
}
bool RotatingLaserRangeFinder::startHook()
{
    if (! RotatingLaserRangeFinderBase::startHook())
        return false;

    std::string prefix = _robot_name.value();
    std::string name = prefix + _name.value();

    // at the moment the link name have to be the same as the laser sensor name
    // we should think about splitting it
    if (!control->envireGraph->containsFrame(name))
    {
        LOG_ERROR_S << "There is no frame '" << name << "' in the graph";
        return false;
    }

    using RotatingRaySensorItem = envire::core::Item<::envire::types::sensors::RotatingRaySensor>;

    if (!control->envireGraph->containsItems<RotatingRaySensorItem>(name))
    {
        LOG_ERROR_S << "There is no RotatingRaySensor object in the frame '" << name << "'";
        return false;
    }

    LOG_INFO_S << "Found item for RotatingRaySensor in the frame '" << name << "'";

    using BaseSensorItem = envire::core::Item<std::shared_ptr<interfaces::BaseSensor>>;
    auto& it = control->envireGraph->getItem<BaseSensorItem>(name);
    auto baseSensor = it->getData();
    mSensor = std::dynamic_pointer_cast<mars::core::RotatingRaySensor>(baseSensor);
    if( !mSensor ){
        std::cerr  << "The sensor with " <<  _name.value() <<  
                " is not of the correct type (RotatingRaySensor)" << std::endl;
        return false;
    }
    
    return true;
}
void RotatingLaserRangeFinder::updateHook()
{
    RotatingLaserRangeFinderBase::updateHook();

    // Seems Plugin is set up but not active yet, we are not sure that we 
    // are initialized correctly so retuning
    if(!isRunning()) {
        return; 
    }

    base::samples::Pointcloud pointcloud;
    //pointcloud.time = getTime();
    
    std::vector<mars::utils::Vector> data;
    long long time;
    if(mSensor->getPointcloud(data, &time)) {
        pointcloud.time = base::Time::fromMilliseconds(time);
        // TODO Min/max is actually already part of the sensor
        std::vector<mars::utils::Vector>::iterator it = data.begin();
        for(; it != data.end(); it++) {
            int len_ray = it->norm();
            if(len_ray >= _min_range.get() && len_ray <= _max_range.get()) {
                base::Vector3d vec((*it)[0], (*it)[1], (*it)[2]);
                pointcloud.points.push_back(vec);
            }
        }
        _pointcloud.write(pointcloud);
    }
}

void RotatingLaserRangeFinder::errorHook()
{
    RotatingLaserRangeFinderBase::errorHook();
}
void RotatingLaserRangeFinder::stopHook()
{
    RotatingLaserRangeFinderBase::stopHook();
}
void RotatingLaserRangeFinder::cleanupHook()
{
    RotatingLaserRangeFinderBase::cleanupHook();
}

void RotatingLaserRangeFinder::update(double delta_t) {

}
