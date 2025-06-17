/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Joints.hpp"
#include <boost/foreach.hpp>
#include <mars_interfaces/sim/MotorManagerInterface.h>
#include <mars_interfaces/sim/ControlCenter.h>
#include <base/Logging.hpp>
#include <base/samples/RigidBodyState.hpp>


using namespace mars;

// TODO: add passive joint, without motors
// TODO: check effort, it seems to be always zero

Joints::Joints(std::string const& name)
    : JointsBase(name)
{
}

Joints::Joints(std::string const& name, RTT::ExecutionEngine* engine)
    : JointsBase(name, engine)
{
    //controlMode = mars::IGNORE;
}

Joints::~Joints()
{
}

bool Joints::configureHook()
{
    if (!JointsBase::configureHook())
        return false;

    // set robot name as prefix
    if (_robot_name.get() != "")
        prefix = _robot_name.get();


    // add prefix to the joint names from config to be able to find the joints inside the mars
    // since all mars elements contains prefix in their names
    jointNames.clear();
    jointNames = _names.value();
    std::for_each(jointNames.begin(), jointNames.end(),
        [&](std::string &jointName)
        { jointName = prefix + jointName; });

    // TODO: for now we get SubWorld frame by its frame name, it can be changed later
    // find all joints that required by config
    const VertexDesc subWorldVertex = control->envireGraph->vertex(SIM_CENTER_FRAME_NAME);//"World::" + prefix);
    findJoints(subWorldVertex);

    // initialise jointStatus vector with the name of required joints
    size_t num_joints = jointNames.size();
    jointStatus.resize(num_joints);
    jointStatus.names = jointNames;

    return true;
}
bool Joints::startHook()
{
    if (!JointsBase::startHook())
        return false;
    return true;
}
void Joints::updateHook()
{
    JointsBase::updateHook();
}
void Joints::errorHook()
{
    JointsBase::errorHook();
}
void Joints::stopHook()
{
    JointsBase::stopHook();
}
void Joints::cleanupHook()
{
    motorJoints.clear();
    passiveJoints.clear();
    prefix = "";
    jointNames.clear();

    JointsBase::cleanupHook();
}

void Joints::update(double time)
{
    if(!isRunning()) return; //Seems Plugin is set up but not active yet, we are not sure that we are initialized correctly so retuning

    while (_command.read(jointCommand) == RTT::NewData)
    {
        for (auto &jointName : jointCommand.names)
        {
            std::string jointNameFull = prefix + jointName;
            if (motorJoints.count(jointNameFull)) {
                base::JointState state = jointCommand.getElementByName(jointName);

                if( state.hasPosition() )
                {
                    motorJoints[jointNameFull].first->setValue(state.position);
                }
                else if( state.hasSpeed() ) {
                    motorJoints[jointNameFull].first->setVelocity(state.speed);
                }
                if( state.hasEffort() )
                {
                    motorJoints[jointNameFull].first->setFeedForwardTorque(state.effort);
                    //LOG_WARN_S << "Effort command ignored for the joint '" << jointName << "' (full name: '" << jointNameFull << ")'";
                }
                if( state.hasRaw() )
                {
                    LOG_WARN_S << "Raw command ignored for the joint '" << jointName << "' (full name: '" << jointNameFull << ")'";
                }
            } else {
                LOG_ERROR_S << "There is no joint with the name '" << jointName << "' (full name: '" << jointNameFull << ")' or this joint is passive";
            }

        }
    }

    // send the current state of the joints
    for (auto &jointName : jointStatus.names)
    {
        base::JointState state;
        // check if the joint has motor or passive
        // get the data from mars
        if (motorJoints.count(jointName)) {
            MJPair &mj = motorJoints[jointName];
            state.position = mj.first->getActualPosition();
            state.speed = mj.second->getVelocity();
            // TODO: should we use getMotorTorque?
            //state.effort = mj.first->getTorque();
            state.effort = mj.second->getMotorTorque();

            jointStatus[jointName] = state;
        }
        else if (passiveJoints.count(jointName)) {
            // TODO add passive joint
            auto &joint = passiveJoints[jointName];
            state.position = joint->getPosition();
            state.speed = joint->getVelocity();
            jointStatus[jointName] = state;
        }
    }
    jointStatus.time = getTime();
    _status.write(jointStatus);
}

void Joints::findJoints(const VertexDesc &vertex)
{
    // parse the sub graph to find all required joints and their corresponding motors
    if(control->graphTreeView->tree.find(vertex) != control->graphTreeView->tree.end())
    {
        const std::unordered_set<VertexDesc>& children = control->graphTreeView->tree[vertex].children;
        // TODO: there is some issue if children is empty
        if (!children.empty()) {
            for(const VertexDesc child : children)
            {
                using JointItem = envire::core::Item<interfaces::JointInterfaceItem>;
                using JointItemItr = envire::core::EnvireGraph::ItemIterator<JointItem>;
                JointItemItr begin_joint, end_joint, cur_joint;
                boost::tie(begin_joint, end_joint) = control->envireGraph->getItems<JointItem>(child);

                using SimMotorItem = envire::core::Item<std::shared_ptr<core::SimMotor>>;
                using SimMotorItemItr = envire::core::EnvireGraph::ItemIterator<SimMotorItem>;
                SimMotorItemItr begin_motor, end_motor, cur_motor;
                boost::tie(begin_motor, end_motor) = control->envireGraph->getItems<SimMotorItem>(child);

                // parse the joints of the frame to find the joints required by config
                for(cur_joint = begin_joint; cur_joint!=end_joint; cur_joint++)
                {
                    // all joint names saved with the prefix (robot name) inside the mars
                    std::shared_ptr<interfaces::JointInterface> jointInterface = cur_joint->getData().jointInterface;
                    std::string jointName;
                    jointInterface->getName(&jointName);

                    // check if there is a motor for joint, than store the motor joint relation
                    // if not, store joint as passive
                    bool hasMotor = false;
                    cur_motor = begin_motor;
                    std::shared_ptr<core::SimMotor> simMotor;
                    while (cur_motor != end_motor) {
                        simMotor = cur_motor->getData();
                        if (std::string(simMotor->getJointName()) == jointName)
                        {
                            hasMotor = true;
                            break;
                        }
                        cur_motor++;
                    }
                    if(hasMotor)
                    {
                        // check if the motor is required by config
                        if (std::find(jointNames.begin(), jointNames.end(), simMotor->getName()) != std::end(jointNames))
                        {
                            motorJoints[simMotor->getName()] = std::make_pair(simMotor, jointInterface);
                        }
                    }
                    else
                    {
                        // check if the joint is required by config
                        if (std::find(jointNames.begin(), jointNames.end(), jointName) != std::end(jointNames))
                        {
                            passiveJoints[jointName] = jointInterface;
                        }
                    }
                }
                findJoints(child);
            }
        }
    } else {
        LOG_ERROR_S << "Frame " << SIM_CENTER_FRAME_NAME << " can not be found";
        //LOG_ERROR_S << "SubWorld " << "SubWorld::" << prefix << " can not be found";
    }
}
