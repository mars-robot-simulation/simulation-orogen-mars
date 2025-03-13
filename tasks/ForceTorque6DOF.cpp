/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ForceTorque6DOF.hpp"

//#include <mars/interfaces/sim/SensorManagerInterface.h>
//#include <mars/sim/Joint6DOFSensor.h>
//#include <mars/sim/NodeContactForceSensor.h>
#include <base/Float.hpp>

#include <mars_interfaces/sim/ControlCenter.h>
#include <mars_utils/mathUtils.h>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>
#include <envire_types/sensors/Joint6DOFSensor.hpp>

#include <mars_core/sensors/Joint6DOFSensor.hpp>

#include <base/Logging.hpp>


using namespace mars;

ForceTorque6DOF::ForceTorque6DOF(std::string const& name)
    : ForceTorque6DOFBase(name)
{
}

ForceTorque6DOF::ForceTorque6DOF(std::string const& name, RTT::ExecutionEngine* engine)
    : ForceTorque6DOFBase(name, engine)
{
}

ForceTorque6DOF::~ForceTorque6DOF()
{
}

void ForceTorque6DOF::init()
{
}

void ForceTorque6DOF::update(double delta_t)
{
    using BaseSensorItem = envire::core::Item<std::shared_ptr<interfaces::BaseSensor>>;

    if(!isRunning()) return;

    //normally this should not result in a resize
	  wrenches_deprecated.resize(mars_names.size());

    for( size_t i=0; i<mars_names.size(); ++i ){
        auto& it = control->envireGraph->getItem<BaseSensorItem>(mars_names[i]);
        auto baseSensor = it->getData();
        if(!baseSensor){
            throw std::runtime_error("There is no sensor by the name of " + mars_names[i]);
        }
        else {
            // try to convert to a specific sensor
            std::shared_ptr<mars::core::Joint6DOFSensor> sensor6dof = std::dynamic_pointer_cast<mars::core::Joint6DOFSensor>(baseSensor);
            
            //TODO
            //mars::core::NodeContactForceSensor* sensor1dof = std::dynamic_pointer_cast<mars::core::NodeContactForceSensor>(baseSensor);

            if (/*sensor1dof ||*/ sensor6dof) {

                base::Wrench wrench;
                wrench.force = base::Vector3d(base::unset<double>(), base::unset<double>(), base::unset<double>());
                wrench.torque = base::Vector3d(base::unset<double>(), base::unset<double>(), base::unset<double>());

                // just assign values to the valid sensor
                //if (sensor6dof) {
                    sensor6dof->getForceData(&wrench.force);
                    sensor6dof->getTorqueData(&wrench.torque);
                //} else {
                //    mars::interfaces::sReal *sens_val;
                //    sensor1dof->getSensorData(&sens_val);
                //    wrench.force.z() = *sens_val;
                //}

                wrenches.time = getTime();
                wrenches.elements[i] = wrench;
                if (external_names.empty()) {
                    wrenches.names[i] = mars_names[i];
                }
                else {
                    wrenches.names[i] = external_names[i];
                }

                // support old data type
                base::samples::Wrench wrench_deprecated;
                wrench_deprecated.force = wrench.force;
                wrench_deprecated.torque = wrench.torque;
                wrench_deprecated.time = getTime();
                wrenches_deprecated[i] = wrench_deprecated;
            }
        }
    }

    _wrenches_deprecated.write(wrenches_deprecated);
    _wrenches.write(wrenches);
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ForceTorque6DOF.hpp for more detailed
// documentation about them.

bool ForceTorque6DOF::configureHook()
{
    if (! ForceTorque6DOFBase::configureHook())
        return false;

    size_t num_sensors = _names.value().size();

    mars_ids.clear();
    mars_ids.resize(num_sensors);
    mars_names.clear();
    mars_names = _names.value();
    external_names = _name_remap.get();
    if(!external_names.empty() && external_names.size() != mars_names.size()){
        throw(std::runtime_error("Sizes of the properties 'name_remap' and 'names' differ. If 'name_remap' is given, it must have the same size as 'names'."));
    }

	wrenches.resize(num_sensors);

    return true;
}
bool ForceTorque6DOF::startHook()
{
    if (! ForceTorque6DOFBase::startHook())
        return false;
    return true;
}
void ForceTorque6DOF::updateHook()
{
    ForceTorque6DOFBase::updateHook();
}
void ForceTorque6DOF::errorHook()
{
    ForceTorque6DOFBase::errorHook();
}
void ForceTorque6DOF::stopHook()
{
    ForceTorque6DOFBase::stopHook();
}
void ForceTorque6DOF::cleanupHook()
{
    ForceTorque6DOFBase::cleanupHook();
}
