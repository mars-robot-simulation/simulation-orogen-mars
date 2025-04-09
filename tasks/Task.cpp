#include "Task.hpp"
#include <mars_app/MARS.hpp>
#include <mars_core/Simulator.hpp>
#include <mars_utils/Thread.h>
#include <mars_utils/mathUtils.h>
#include <mars_utils/misc.h>
#include <mars_interfaces/sim/SimulatorInterface.h>

#include <mars/tasks/MarsControl.hpp>
#include <mars_gui/MarsGui.h>

#include <main_gui/MainGUI.h>
#include <main_gui/GuiInterface.h>

#include <cfg_manager/CFGManagerInterface.h>
#include <mars_interfaces/graphics/GraphicsManagerInterface.h>
//#include <mars/graphics/GraphicsManager.h>
#include <mars_app/GraphicsTimer.hpp>
#include <mars_interfaces/sim/NodeManagerInterface.h>

#include <mars_core/SimMotor.hpp>
#include <mars_interfaces/sim/MotorManagerInterface.h>

//#include <mars/multisim-plugin/MultiSimPlugin.h>

#include <lib_manager/LibManager.hpp>
#include <QApplication>
#if QT_VERSION >= 0x050000
#include <QStyleFactory>
#else
#include <QPlastiqueStyle>
#endif
#include <QMutex>
#include <QWaitCondition>

#include <boost/filesystem.hpp>

#include <base-logging/Logging.hpp>

using namespace mars;
using namespace std;


mars::interfaces::SimulatorInterface *Task::simulatorInterface = 0;
mars::Task *Task::taskInterface = 0;
//mars::app::GraphicsTimer *Task::graphicsTimer = 0;
lib_manager::LibManager* Task::libManager = 0;

extern int optind;

struct MethodInQtThreadFailed : runtime_error {
    using runtime_error::runtime_error;
};

struct MethodExecutionEvent : public QEvent {
    QMutex& mLock;
    QWaitCondition& mSignal;
    bool& mResult;
    string& mMessage;

    MethodExecutionEvent(QMutex& lock, QWaitCondition& signal,
                         bool& result, string& message)
        : QEvent(QEvent::User)
        , mLock(lock), mSignal(signal), mResult(result), mMessage(message) { }

    function<void()> f;
};

class MethodExecutionObject : public QObject {
    bool event(QEvent* event) {
        auto ev = dynamic_cast<MethodExecutionEvent*>(event);
        if (ev) {
            try {
                ev->f();

                QMutexLocker sync(&(ev->mLock));
                ev->mResult = true;
                ev->mSignal.wakeAll();
            }
            catch (exception const& e) {
                QMutexLocker sync(&(ev->mLock));
                ev->mResult = false;
                ev->mMessage = e.what();
                ev->mSignal.wakeAll();
            }
            catch (...) {
                QMutexLocker sync(&(ev->mLock));
                ev->mResult = false;
                ev->mMessage =
                    "exception thrown that is not a subclass of exception";
                ev->mSignal.wakeAll();
            }
            return true;
        }
        return false;
    }
};


Task::Task(std::string const& name)
    : TaskBase(name)
    , multisimPlugin(0)
{
    Task::taskInterface = this;
    setlocale(LC_ALL,"C"); //Make sure english Encodings are used
    setenv("LANG","C",true);
    _gravity.set(Eigen::Vector3d(0,0,-9.81));
    mExecutorLock = new QMutex();
    mExecutorSignal = new QWaitCondition();
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
    , multisimPlugin(0)
{
    Task::taskInterface = this;
    setlocale(LC_ALL,"C"); //Make sure english Encodings are used
    setenv("LANG","C",true);
    mExecutorLock = new QMutex();
    mExecutorSignal = new QWaitCondition();
}

Task::~Task()
{
  delete mExecutorLock;
  delete mExecutorSignal;
  //delete simulation;
}

void Task::processInQtThread(function<void()> f)
{
    QMutexLocker sync(mExecutorLock);

    bool result = true;
    string message;
    auto* event = new MethodExecutionEvent(
        *mExecutorLock, *mExecutorSignal, result, message
    );
    event->f = f;
    QApplication::instance()->postEvent(mExecutor, event);

    mExecutorSignal->wait(mExecutorLock);
    if (!result)
    {
        throw MethodInQtThreadFailed(message);
    }
}

void Task::loadScene(::std::string const & path)
{
    if(simulatorInterface)
    {
        simulatorInterface->loadScene(path, path,true,true);
    }else
    {
        LOG_ERROR_S << "Simulator not yet started cout not load scenefile";
    }
}

mars::interfaces::SimulatorInterface* Task::getSimulatorInterface()
{
    return simulatorInterface;
}

mars::Task* Task::getTaskInterface()
{
    return taskInterface;
}

void* Task::startTaskFunc(void* argument)
{
    setlocale(LC_ALL,"C"); //Make sure english Encodings are used
    setenv("LANG","C",true);
    TaskArguments* marsArguments = static_cast<TaskArguments*>(argument);

    Task* mars = marsArguments->mars;

    // Using the 'command-line' interface to pass
    // arguments to mars interface
    // set the option to "" if it does not require further args
    std::vector<Option> rawOptions = marsArguments->raw_options;
    if(marsArguments->controller_port > 0)
    {
        char buffer[10];
        sprintf(buffer, "%d", marsArguments->controller_port);
        Option controllerPortOption("-c", std::string(buffer));
        rawOptions.push_back(controllerPortOption);
    }
    if(!marsArguments->config_dir.empty())
    {
        Option confDirOption("-C", marsArguments->config_dir);
        rawOptions.push_back(confDirOption);
    }
    if(!marsArguments->enable_gui)
    {
      Option noGUIOption("--no-gui", "");
      rawOptions.push_back(noGUIOption);
    }
    char** argv = mars->setOptions(rawOptions);
    int argc = mars->getOptionCount(rawOptions);
    // incrememt arcument counter since setOptions adds mars_core to arguments
    argc += 1;
    // Plus one for executable name
    for(int i = 0; i < argc; i++)
    {
        LOG_INFO_S << "Simulator: argument #" << i << " " << argv[i];
    }
    LOG_ERROR_S << "Create MARS instance";
    simulation = new mars::app::MARS();
    optind = 1;
    simulation->readArguments(argc, argv);
    // Prepare Qt Application Thread which is required
    // for core mars and gui
#if QT_VERSION >= 0x050000
    QStyle* style = QStyleFactory::create("plastique");
    if(style)
    {
        qApp->setStyle(style);
    } else
    {
        LOG_WARN_S << "QStyle 'plastique' is not available";
    }
#else
    qApp->setStyle(new QPlastiqueStyle);
#endif

    setlocale(LC_ALL,"C");
    setenv("LANG","C",true);
    struct lconv* locale = localeconv();
    LOG_INFO_S << "Active locale (LC_ALL): ";
    if( *(locale->decimal_point) != '.')
    {
        LOG_ERROR_S << "Current locale conflicts with mars";
        marsArguments->failed_to_init = true;
        return 0;
    }
    std::string cmd;
    for(int i = 0; i < argc;++i)
    {
        cmd += std::string(argv[i]);
        cmd += " ";
    }
    LOG_ERROR_S << "Starting mars with: " << cmd;
    simulation->start(argc, argv);

    // Prepare the LibManager and required configuration files
    libManager = simulation->getLibManager();

    // load the additionally specified plugins
    for( std::vector<std::string>::iterator it = marsArguments->mars_plugins.begin();
         it != marsArguments->mars_plugins.end(); ++it )
    {
        libManager->loadLibrary( *it );
    }

    mars->simulatorInterface = libManager->getLibraryAs<core::Simulator>("mars_core");
    if(!mars->simulatorInterface)
    {
        LOG_ERROR_S << "CRITICAL (cause abort) Simulation could not be retrieved via lib_manager";
        marsArguments->failed_to_init = true;
        return 0;
    }

    // set mars properties, if specified
    lib_manager::LibInterface* lib = libManager->getLibrary(std::string("cfg_manager"));
    if(lib)
    {
        cfg_manager::CFGManagerInterface* cfg = dynamic_cast<cfg_manager::CFGManagerInterface*>(lib);
        if(cfg)
        {
            std::vector<SimulationProperty> props = marsArguments->mars_property_list;
            for(std::vector<SimulationProperty>::iterator prop_it = props.begin(); prop_it != props.end(); ++prop_it)
            {
                // get or create property
                cfg_manager::cfgPropertyStruct cfg_prop_struct;
                cfg_prop_struct = cfg->getOrCreateProperty(prop_it->lib_name, prop_it->property_name, prop_it->value);

                // overriding any defaults
                cfg_prop_struct.sValue = prop_it->value;
                cfg->setProperty(cfg_prop_struct);
                LOG_DEBUG("setting property %s\n", cfg_prop_struct.sValue.c_str());
            }
        }
    }

    // TODO: CODE FROM MARS1
    //if(marsArguments->add_floor){
    //    mars->simulatorInterface->getControlCenter()->nodes->createPrimitiveNode("Boden",mars::interfaces::NODE_TYPE_PLANE,false,mars::utils::Vector(0,0,0.0),mars::utils::Vector(600,600,0));
    //}
    int result = mars->simulatorInterface->getControlCenter()->dataBroker->registerTriggeredReceiver(mars, "mars_sim", "simTime", "mars_sim/postPhysicsUpdate",1);
    (void)result;
    assert(result);

    // is realtime calc requested?
    if(marsArguments->realtime_calc){
        mars->simulatorInterface->getControlCenter()->cfg->setPropertyValue("Simulator", "realtime calc", "value", marsArguments->realtime_calc);
    }

    mars->marsGraphics = libManager->getLibraryAs<mars::interfaces::GraphicsManagerInterface>("mars_graphics");
    marsGraphics = mars->marsGraphics;
    // Synchronize with configureHook
    marsArguments->initialized = true;

    return 0;
}

int Task::getOptionCount(const std::vector<Option>& options)
{
    std::vector<Option>::const_iterator it;

    // First just counting the number of arguments
    int count = 0;
    for(it = options.begin(); it != options.end(); it++)
    {
        Option option = *it;
        // Differentiate between option with args and without
        if(option.parameter != "")
            count += 2;
        else
            count += 1;
    }

    return count;
}

bool Task::setShow_coordinate_system(bool value)
{
    if(!marsGraphics){
        return true;
    }

    //Call the base function, DO-NOT Remove
    //if(value)
    //    marsGraphics->hideCoords();
    //else
    //    marsGraphics->showCoords();

    return(mars::TaskBase::setShow_coordinate_system(value));
}

bool Task::setReaction_to_physics_error(::std::string const & value)
{
    //TODO Add your code here
    if(isConfigured()){
        if(!simulatorInterface){
            LOG_ERROR("Task is not running could not set reaction to physics error");
            return false;
        }
        if(value == "abort" || value == "reset" || value == "warn" || value == "shutdown"){
            simulatorInterface->getControlCenter()->cfg->setPropertyValue("Simulator", "onPhysicsError","value", value);
        }else{
            LOG_ERROR("Could not ser rection to physics: Possible Values: abort (killing sim), reset (ressing scene and mars), warn (keep mars running an print warnings), shutdown (stop physics but keep mars-running and set this tas to the error state)");
            return false;
        }
    }

    //Call the base function, DO-NOT Remove
    return(mars::TaskBase::setReaction_to_physics_error(value));
}

char** Task::setOptions(const std::vector<Option>& options)
{
    int count = getOptionCount(options)+ 1;
    char** argv = (char**) calloc(count, sizeof(char**));

    // Set executable name to mars_core
    count = 0;
    argv[count++] = "mars_core";

    std::vector<Option>::const_iterator it;
    for(it = options.begin(); it != options.end(); it++)
    {
        Option opt = *it;

        if(opt.name == "")
            continue;

        argv[count++] = strdup(opt.name.c_str());
        if(opt.parameter != "")
        {
            argv[count++] = strdup(opt.parameter.c_str());
        }
    }

    return argv;
}

bool Task::configureHook()
{
    configureError = false;
    LOG_ERROR_S << "Configure hook...";

    if(qApp) {

        if (!mExecutor) {
            mExecutor = new MethodExecutionObject();
            mExecutor->moveToThread(QApplication::instance()->thread());
        }

        processInQtThread(bind(&Task::configureUI, this));
    }
    else {
        configureUI();
    }
    return configureError;
}

void Task::configureUI()
{
    LOG_ERROR_S << "Configure hook UI...";
    if(_config_dir.get().empty())
    {
        LOG_ERROR_S << "Config directory is not set! Cannot start mars.";
        throw std::runtime_error("Config directory is not set! Can not start mars");
    }

    //check if the environemnt was sourced more than once and the path has more than one entry
    int pos = _config_dir.get().rfind(":/");
    if(pos != _config_dir.get().size()-1)
        _config_dir.set(_config_dir.get().substr(pos+1));

    LOG_ERROR_S << "Calling configure: with " << _config_dir.get();

    //mars is not setting the config path properly
    //therefore we have to go into to the config folder
    //if(0 != chdir(_config_dir.get().c_str()))
    //{
    //    LOG_ERROR_S << "Config directory " << _config_dir.get() << " does not exist. Cannot start mars.";
    //    throw std::runtime_error(std::string("Config directory ") +_config_dir.get() +" does not exist. Can not start mars.");
    //}
    // Startup of mars
    TaskArguments argument;
    argument.mars = this;
    if(qApp) {
        argument.enable_gui = true;
    }
    else {
        argument.enable_gui = false;
    }
    argument.controller_port = _controller_port.get();
    argument.raw_options = _raw_options.get();
    argument.config_dir = _config_dir.get();
    argument.mars_property_list = _simulation_property_list.get();
    argument.initialized = false;
    argument.add_floor = _add_floor.get();
    argument.failed_to_init=false;
    argument.realtime_calc = _realtime_calc.get();
    argument.mars_plugins = _plugins.get();

    // go through list of plugins, and see if they have an absolute path
    for( std::vector<std::string>::iterator it = argument.mars_plugins.begin();
          it != argument.mars_plugins.end(); ++it )
    {
        *it = boost::filesystem::absolute(
              boost::filesystem::path( *it ),
              boost::filesystem::path( _plugin_dir.get() ) ).string();
    }

    startTaskFunc(&argument);

    if(argument.failed_to_init){
            LOG_ERROR_S << "Task failed to start, see Error above";
            configureError = true;
            return;
    }

    LOG_INFO_S << "Task running";

    // Simulation is now up and running and plugins can be added
    // Configure basic functionality of mars
    // Check if distributed mars should be activated

    // todo: should be loaded via lib_manager
    /*
    if(_distributed_mars.get())
    {
        LOG_INFO_S << "Loading MultiSimPlugin";
        multisimPlugin = new MultiSimPlugin(libManager);
        LOG_INFO_S << "MultiSimPlugin loaded";
    }
    */
    bool startStop = false;
    LOG_INFO_S << "before isSimRunning";
    if(simulatorInterface->isSimRunning()) {
      simulatorInterface->StopSimulation();
      startStop = true;
      LOG_INFO_S << "startStop";
    }
    LOG_INFO_S << "after isSimRunning";
    // Load scenes before robot to avoid complex robots blocking correct loading of the scene
    std::vector<std::string> sceneNames = _initial_scenes.get();
    if(!sceneNames.empty()){
        for (std::vector< std::string >::iterator scene = sceneNames.begin(); scene != sceneNames.end();scene++){
            LOG_INFO_S << "load initial scenes ... " << *scene;
            simulatorInterface->loadScene(*scene, *scene,false,false);
        }
    }

    if(!_initial_scene.get().empty()){
        LOG_INFO_S << "load initial scene " << _initial_scene.get();
        simulatorInterface->loadScene(_initial_scene.get(), std::string("initial"),false,false);
    }
    LOG_INFO_S << "after load";
    if(startStop) {
      LOG_INFO_S << "startSim";
      simulatorInterface->StartSimulation();
    }
    std::vector<Positions> positions = _positions.get();
    if(!positions.empty()){
        for (std::vector< Positions >::iterator offset = positions.begin(); offset != positions.end();offset++){
            LOG_INFO_S << "move node ...";
            move_node(*offset);
        }
    }

    // TODO: CODE FROM MARS1
    // we dont set the motor position for now
    // need to be rewritten later to adapt to the new interfaces
    /*mars::Pose initial_pose = _initial_pose.get();
    if(!initial_pose.empty()){
        mars::interfaces::ControlCenter* control = simulatorInterface->getControlCenter();
        if (control){
            for (mars::Pose::iterator pos = initial_pose.begin(); pos != initial_pose.end();pos++){
                int marsMotorId = control->motors->getID( pos->name );
                mars::sim::SimMotor *motor = control->motors->getSimMotor( marsMotorId );
                if (motor){
                    motor->setValue( pos->pos );
                }else{
                    LOG_ERROR("no motor %s",pos->name.c_str());
                }
            }
        }else{
            LOG_ERROR("no contol center");
        }
    }*/

    {//Setting the Step-with for the mars
    cfg_manager::cfgPropertyStruct c = simulatorInterface->getControlCenter()->cfg->getOrCreateProperty("Simulator", "calc_ms", _sim_step_size.get()*1000.0);
    c.dValue = _sim_step_size.get()*1000.0;
    simulatorInterface->getControlCenter()->cfg->setProperty(c);
    }

    {
    std::string value = _reaction_to_physics_error.get();
    if(value == "abort" || value == "reset" || value == "warn" || value == "shutdown"){
        simulatorInterface->getControlCenter()->cfg->setPropertyValue("Simulator", "onPhysicsError","value", value);
    }else{
        LOG_ERROR("Wront selection for physic error setting\n");
        configureError = true;
        return;
    }
    }
    simulatorInterface->getControlCenter()->cfg->setPropertyValue("Simulator","getTime:useNow","value",_use_now_instead_of_sim_time.get());
    setGravity_internal(_gravity.get());
    configureError = updateDynamicProperties();
    LOG_INFO_S << "finished configureHookUI";
    return;
}


bool Task::startHook()
{
    if(qApp) {
        if (!mExecutor) {
            mExecutor = new MethodExecutionObject();
            mExecutor->moveToThread(QApplication::instance()->thread());
        }

        processInQtThread(bind(&Task::startUI, this));
    }
    else {
        startUI();
    }
    return true;
}

void Task::startUI()
{
    // Simulation should be either started manually,
    // or by using the control_action input_port
    //
    if (_start_sim.get()){
        simulatorInterface->StartSimulation();
    }
    return;
}

void Task::updateHook()
{
    mars::Control controlAction;
    if(_control_action.read(controlAction) == RTT::NewData)
    {
        switch(controlAction)
        {
            case START:
                LOG_INFO_S << "ControlAction: Start received";
                if(!simulatorInterface->isSimRunning())
                    simulatorInterface->startStopTrigger();
                break;
            case PAUSE:
                LOG_INFO_S << "ControlAction: Pause received";
                if(simulatorInterface->isSimRunning())
                    simulatorInterface->startStopTrigger();
                break;
            case RESET:
                LOG_INFO_S << "ControlAction: Reset received";
                simulatorInterface->resetSim();
                break;
            case STEP:
                LOG_INFO_S << "ControlAction: Step received";
                simulatorInterface->singleStep();
                break;
            default:
                LOG_WARN_S << "Simulation: Unknown control action " << controlAction << " received";

        }
    }

    if(simulatorInterface->hasSimFault()){
        LOG_INFO_S << "Simulation detected a Physics error, stopping all plugins and go to Exception state";
        for(unsigned int i=0;i<plugins.size();i++){
            plugins[i]->handleMarsShudown();
        }
        exception(PHYSICS_ERROR);
 //       QCoreApplication::quit(); //Quitting QApplication too
    }
}

void Task::errorHook()
{
}

void Task::stopHook()
{
    
    // std::cout << "STOP HOOK" << std::endl;
    // for(unsigned int i=0;i<plugins.size();i++){
    //     plugins[i]->handleTaskShudown();
    // }
    // simulatorInterface->exitTask();

    // std::cout << "STOP HOOK quitting qapp" << std::endl;
    // QCoreApplication::quit(); //Quitting QApplication too
    // std::cout << "STOP HOOK quitting qapp finish" << std::endl;
    
}

void Task::registerPlugin(Plugin* plugin){
    plugins.push_back(plugin);
}

void Task::unregisterPlugin(Plugin* plugin){
    plugins.push_back(plugin);
}

void Task::cleanupHook()
{
    if(qApp) {
        if (!mExecutor) {
            mExecutor = new MethodExecutionObject();
            mExecutor->moveToThread(QApplication::instance()->thread());
        }

        processInQtThread(bind(&Task::cleanupUI, this));
    }
    else {
        cleanupUI();
    }
}

void Task::cleanupUI()
{
    // for(unsigned int i=0;i<plugins.size();i++){
    //     plugins[i]->handleMarsShudown();
    // }
    // plugins.clear();

    simulatorInterface->exitMars();
    while( simulatorInterface->isSimRunning()) mars::utils::msleep(1);


    LOG_DEBUG_S << "CLEANUP HOOK quitting qapp finish";

   // delete libManager;

    libManager->releaseLibrary("mars_core");
    libManager->releaseLibrary("cfg_manager");
    libManager->releaseLibrary("mars_graphics");
    //delete simulation;
//    libManager->releaseLibrary("mars_gui");
//    libManager->releaseLibrary("mars_graphics");
//    libManager->releaseLibrary("gui_core");


 //   if(multisimPlugin) delete multisimPlugin;
}
/*
bool Task::recover(){
    std::cout << "RECOVER HOOK" << std::endl;
    return TaskBase::recover();
}
void Task::fatal(){
    std::cout << "FATAL HOOK" << std::endl;
    TaskBase::fatal();
}
void Task::exception(){
    std::cout << "EXCEPTION HOOK" << std::endl;
    TaskBase::exception();
}
*/

void Task::receiveData(
        const data_broker::DataInfo& info,
        const data_broker::DataPackage& package,
        int id)
{
    _simulated_time.write(base::Time::fromMilliseconds(simulatorInterface->getTime()));
}

bool Task::setGravity_internal(::base::Vector3d const & value){
    simulatorInterface->setGravity(value);
    return true;
}

bool Task::setGravity(::base::Vector3d const & value)
{
 if(!isConfigured()){
     //The configuration will be done within the configure hook later
     return(mars::TaskBase::setGravity(value));
 }

 setGravity_internal(value);
 return(mars::TaskBase::setGravity(value));
}

void Task::setPosition(::mars::Positions const & positions)
{
    if(isRunning() || isConfigured()){
        LOG_DEBUG("moving '%s' to %g/%g/%g\n", positions.nodename.c_str(), positions.posx, positions.posy, positions.posz);
        move_node(positions);
    }else{
        LOG_ERROR("setPosition called, but mars::Task is whether configured nor running ");
    }
    return;
}

bool Task::setSim_step_size(double value)
{
    //convert to ms
    value *= 1000.0;
    if(!isConfigured()){
        //The configuration will be done within the configure hook later
        return(mars::TaskBase::setSim_step_size(value));
    }
    cfg_manager::cfgPropertyStruct c = simulatorInterface->getControlCenter()->cfg->getOrCreateProperty("Simulator", "calc_ms", value);
    c.dValue = value;
    simulatorInterface->getControlCenter()->cfg->setProperty(c);
    return(mars::TaskBase::setSim_step_size(value));
}

void Task::move_node(::mars::Positions const & arg)
{
    mars::interfaces::NodeManagerInterface* nodes = simulatorInterface->getControlCenter()->nodes;
    mars::interfaces::NodeId id = nodes->getID(arg.nodename);
    if (id){
        mars::interfaces::NodeData nodedata = nodes->getFullNode(id);
        utils::Vector pos = nodes->getPosition(id);

        pos.x() = arg.posx;
        pos.y() = arg.posy;
        pos.z() = arg.posz;

        mars::utils::Vector rotoff;

        rotoff.x() =  arg.rotx;
        rotoff.y() =  arg.roty;
        rotoff.z() =  arg.rotz;

        mars::utils::Quaternion newrot = mars::utils::eulerToQuaternion(rotoff);

        nodedata.pos = pos;
        nodedata.rot = newrot;

        if (arg.edit_all){
            nodes->editNode(&nodedata, mars::interfaces::EDIT_NODE_POS | mars::interfaces::EDIT_NODE_MOVE_ALL);
            nodes->editNode(&nodedata, mars::interfaces::EDIT_NODE_ROT | mars::interfaces::EDIT_NODE_MOVE_ALL);
        }else{
            printf("edit node only %s\n",arg.nodename.c_str());
            nodes->editNode(&nodedata, mars::interfaces::EDIT_NODE_POS);
            nodes->editNode(&nodedata, mars::interfaces::EDIT_NODE_ROT);
        }
    }else{
        LOG_ERROR("node '%s' unknown\n", arg.nodename.c_str());
    }
}
