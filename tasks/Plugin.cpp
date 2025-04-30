/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Plugin.hpp"

using namespace mars;

Plugin::Plugin(std::string const& name)
    : PluginBase(name) ,PluginInterface(0), sim(0)
{
    setlocale(LC_ALL,"C"); //Make sure english Encodings are used
}

Plugin::Plugin(std::string const& name, RTT::ExecutionEngine* engine)
    : PluginBase(name, engine) ,PluginInterface(0), sim(0)
{
    setlocale(LC_ALL,"C"); //Make sure english Encodings are used
}

Plugin::~Plugin()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Plugin.hpp for more detailed
// documentation about them.
bool Plugin::configureHook()
{
    if (! RTT::TaskContext::configureHook())
    {
        RTT::log(RTT::Warning) << "Plugin: configure failed." << RTT::endlog();
        return false;
    }

    if(!connect())
    {
        RTT::log(RTT::Warning) << "Plugin: establishing connection with Mars failed. Configure hook returning false." << RTT::endlog();
        return false;
    }
    return true;
}

bool Plugin::startHook()
{
    if (! RTT::TaskContext::startHook())
        return false;

    return true;
}

void Plugin::updateHook()
{
    RTT::TaskContext::updateHook();
}

void Plugin::errorHook()
{
    RTT::TaskContext::errorHook();
}


void Plugin::stopHook()
{
    RTT::TaskContext::stopHook();
}

void Plugin::cleanupHook()
{
    RTT::TaskContext::cleanupHook();
}

void Plugin::update(double delta_t)
{
}

void Plugin::init()
{
}

base::Time Plugin::getTime()
{
    if(!sim){
        RTT::log(RTT::Error) << "Cannot request sim-time, no simulator running";
        fatal();
    }
    return base::Time::fromMilliseconds(sim->getTime());
}

bool Plugin::connect()
{
    // get simulator interface from singleton
    if( sim )
        disconnect();
    else
    {
        sim = Task::getSimulatorInterface();
        if( !sim ){
            std::cerr << "Plugin: could not get singleton instance of simulator interface." << std::endl;
            RTT::log(RTT::Error) << "Plugin: could not get singleton instance of simulator interface." << std::endl;
            return false;
        }
    }

    // register as plugin
    mars::interfaces::pluginStruct newplugin;
    newplugin.name = provides()->getName();
    newplugin.p_interface = dynamic_cast<mars::interfaces::PluginInterface*>(this);
    newplugin.p_destroy = 0;
    sim->addPlugin(newplugin);

    // get controlcenter
    control = sim->getControlCenter();
    Task::getTaskInterface()->registerPlugin(this);

    return true;
}

void Plugin::disconnect()
{
    if( sim ){
        sim->removePlugin( this );
        Task::getTaskInterface()->unregisterPlugin(this);
    }
}

void Plugin::reset() {};

void Plugin::receiveData(
        const mars::data_broker::DataInfo& info,
        const mars::data_broker::DataPackage& package,
        int id)
{
}

void Plugin::handleMarsShudown(){
    fprintf(stderr,"Shutting down %s, because mars instance is shutting down\n",getName().c_str());
    exception(LOST_MARS_CONNECTION);
}
