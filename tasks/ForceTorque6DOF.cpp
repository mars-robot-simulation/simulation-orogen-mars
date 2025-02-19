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
// for each of the names, get the mars motor id
    for( size_t i=0; i<mars_ids.size(); ++i )
    {
       std::string &name( mars_names[i] );

       if (!control->envireGraph->containsFrame(name))
        {
            LOG_ERROR_S << "There is no frame '" << name << "' in the graph";
            return;
        }


        using Joint6DOFItem = envire::core::Item<::envire::types::sensors::Joint6DOFSensor>;

        if (!control->envireGraph->containsItems<Joint6DOFItem>(name))
        {
            LOG_ERROR_S << "There is no Joint6DOF object in the frame '" << name << "'";
            return;
        }

        LOG_INFO_S << "Found item for Joint6DOF in the frame '" << name << "'";

        //int marsId = control->sensors->getSensorID( name );

        //if( marsId ) {
        //    mars_ids[i] = marsId;
        //}	else {
        //    throw std::runtime_error("there is no sensor by the name of " + name);
        //}
    }
}

void ForceTorque6DOF::update(double delta_t)
{
    if(!isRunning()) return;

    /*
    //normally this should not result in a resize
	  wrenches_deprecated.resize(mars_ids.size());

    for( size_t i=0; i<mars_ids.size(); ++i ){
        mars::interfaces::BaseSensor* base = control->sensors->getSimSensor(mars_ids[i]);
        if (base) {
            // try to convert to a specific sensor
            mars::sim::Joint6DOFSensor* sensor6dof = dynamic_cast<mars::sim::Joint6DOFSensor*>(base);
            mars::sim::NodeContactForceSensor* sensor1dof = dynamic_cast<mars::sim::NodeContactForceSensor*>(base);

            if (sensor1dof || sensor6dof) {

                base::Wrench wrench;
                wrench.force = base::Vector3d(base::unset<double>(), base::unset<double>(), base::unset<double>());
                wrench.torque = base::Vector3d(base::unset<double>(), base::unset<double>(), base::unset<double>());

                // just assign values to the valid sensor
                if (sensor6dof) {
                    sensor6dof->getForceData(&wrench.force);
                    sensor6dof->getTorqueData(&wrench.torque);
                } else {
                    mars::interfaces::sReal *sens_val;
                    sensor1dof->getSensorData(&sens_val);
                    wrench.force.z() = *sens_val;
                }

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
    */
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
