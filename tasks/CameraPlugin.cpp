/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "CameraPlugin.hpp"

#include <base/Logging.hpp>

#include <envire_core/graph/EnvireGraph.hpp>
#include <envire_core/items/Item.hpp>

using namespace mars;

CameraPlugin::CameraPlugin(std::string const& name)
    : CameraPluginBase(name),
        camera(nullptr),
        width(0),
        height(0),
        lastUpdateTime(0),
        lastGrabTime(0),
        frameDelay(0),
        isPeriodic(false)
{
}

CameraPlugin::CameraPlugin(std::string const& name, RTT::ExecutionEngine* engine)
    : CameraPluginBase(name, engine),
        camera(nullptr),
        width(0),
        height(0),
        lastUpdateTime(0),
        lastGrabTime(0),
        frameDelay(0),
        isPeriodic(false)
{
}

CameraPlugin::~CameraPlugin()
{
}

bool CameraPlugin::configureHook()
{
    if (!CameraPluginBase::configureHook())
        return false;

    // check if the imu frame is in the graph
    std::string prefix = _robot_name.value();
    std::string sensorName = prefix + _name.value();

    // in mars the imu frame will be stored as DynamicObject
    // in the graph frame with prefix + name syntax
    const VertexDesc subWorldVertex = control->envireGraph->vertex("World::" + prefix);
    camera = nullptr;
    if (findSensors(subWorldVertex, sensorName))
        LOG_ERROR_S << "Camera '" << sensorName << "' is found";
    else
        LOG_ERROR_S << "No camera '" << sensorName << "' is found";

    camera->activateRendering();
    width = camera->getConfig().width;
    height = camera->getConfig().height;

    control->graphics->addGraphicsUpdateInterface(this);
    // Used in postGraphicsUpdate() to trigger the updateHook every 'frameDelay' ms.
    frameDelay = 1000 / camera->getConfig().updateRate;
    RTT::base::ActivityInterface* activity = this->getActivity();
    isPeriodic = activity->isPeriodic();

    return true;
}

bool CameraPlugin::startHook()
{
    if (!CameraPluginBase::startHook())
        return false;
    return true;
}

void CameraPlugin::updateHook()
{
    CameraPluginBase::updateHook();

    getData();
}

void CameraPlugin::errorHook()
{
    CameraPluginBase::errorHook();
}

void CameraPlugin::stopHook()
{
    camera->deactivateRendering();
    CameraPluginBase::stopHook();
}

void CameraPlugin::cleanupHook()
{
    camera.reset();
    CameraPluginBase::cleanupHook();
}

void CameraPlugin::update(double time)
{
    lastUpdateTime += time;
}

void CameraPlugin::postGraphicsUpdate()
{
    // Frame rate is defined by the update rate of the module.
    if(isPeriodic)
        return;

    if((lastUpdateTime - lastGrabTime) < frameDelay)
	    return;

    // Triggers the updateHook/image request every 'frameDelay' ms.
	trigger();

    lastGrabTime = lastUpdateTime;
}

void CameraPlugin::getData(){
    //This method has to be implemented by the sublasses and is not allowed to be called
    assert(false);
}

bool CameraPlugin::findSensors(const VertexDesc &vertex, const std::string &sensorName)
{
    // parse the sub graph to find a sensor by its name
    if(control->graphTreeView->tree.find(vertex) != control->graphTreeView->tree.end())
    {
        const std::unordered_set<VertexDesc>& children = control->graphTreeView->tree[vertex].children;
        // TODO: there is some issue if children is empty
        if (!children.empty()) {
            for(const VertexDesc child : children)
            {
                using BaseSensorItem = envire::core::Item<std::shared_ptr<interfaces::BaseSensor>>;
                using BaseSensorItemItr = envire::core::EnvireGraph::ItemIterator<BaseSensorItem>;
                BaseSensorItemItr begin_itr, end_itr;
                boost::tie(begin_itr, end_itr) = control->envireGraph->getItems<BaseSensorItem>(child);
                for (; begin_itr != end_itr; begin_itr++)
                {
                    std::shared_ptr<interfaces::BaseSensor> baseSensor = begin_itr->getData();
                    if (baseSensor->getName() == sensorName)
                    {
                        camera = std::dynamic_pointer_cast<mars::core::CameraSensor>(baseSensor);
                        if( !camera ) {
                            LOG_INFO_S << "The found sensor '" << sensorName << "' is not a camera";
                        } else {
                            return true;
                        }
                    }
                }
                bool found = findSensors(child, sensorName);
                if (found)
                    return true;
            }
        }
    } else {
        LOG_ERROR_S << "The given vertex does not exist in the graph tree view";
    }
    return false;
}
