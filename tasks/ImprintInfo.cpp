/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ImprintInfo.hpp"

using namespace mars;

ImprintInfo::ImprintInfo(std::string const& name)
    : ImprintInfoBase(name)
{
}

ImprintInfo::ImprintInfo(std::string const& name, RTT::ExecutionEngine* engine)
    : ImprintInfoBase(name, engine)
{
}

ImprintInfo::~ImprintInfo()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See ImprintInfo.hpp for more detailed
// documentation about them.

bool ImprintInfo::configureHook()
{
    if (! ImprintInfoBase::configureHook())
        return false;
    control->dataBroker->registerSyncReceiver(this, "mars_sim", "imprint_plugin", 0);
    return true;
}
bool ImprintInfo::startHook()
{
    if (! ImprintInfoBase::startHook())
        return false;
    return true;
}
void ImprintInfo::updateHook()
{
    ImprintInfoBase::updateHook();
    unsigned long id = control->dataBroker->getDataID("mars_sim", "imprint_plugin");
    data_broker::DataPackage dbPackage = control->dataBroker->getDataPackage(id);
    double x=0, y=0;
    int type = -1;
    dbPackage.get("x", &x);
    dbPackage.get("y", &y);
    dbPackage.get("type", &type);
    //if(type > -1)
    {
        configmaps::ConfigMap map;
        map["x"] = x;
        map["y"] = y;
        map["type"] = type;
        std::string s = map.toYamlString();
        _imprintInfo.write(s);
        _x.write(x);
        _y.write(y);
        _type.write(type);
    }
}

void ImprintInfo::errorHook()
{
    ImprintInfoBase::errorHook();
}
void ImprintInfo::stopHook()
{
    ImprintInfoBase::stopHook();
    control->dataBroker->unregisterSyncReceiver(this, "*", "*");
}
void ImprintInfo::cleanupHook()
{
    ImprintInfoBase::cleanupHook();
}
