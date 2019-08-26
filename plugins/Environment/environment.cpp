#include "environment.h"
#include <random>
#include "grSim_Packet.pb.h"
#include "vision_detection.pb.h"
#include "KunDebugEngine.h"
namespace {
bool outside(const CGeoPoint& pos){
    return pos.x()>6||pos.x()<-6||pos.y()>4.5||pos.y()<-4.5;
}
double remap(const unsigned int origin_min,const unsigned int origin_max,const double target_min,const double target_max,const double input){
    const double a = double(origin_min);
    const double b = double(origin_max);
    const double c = target_min;
    const double d = target_max;
    return input*(d-c)/(b-a)+(b*c-d*a)/(b-a);
}
template<typename T>
T limitRange(T value, T minValue, T maxValue) {
    return value > maxValue ? maxValue : (value < minValue) ? minValue : value;
}
}
double Normalize(double angle)
{
    if (fabs(angle) > 10 ){
        std::cout<<angle<<" Normalize Error!!!!!!!!!!!!!!!!!!!!"<<std::endl;
        return 0;
    }
    const double M_2PI = M_PI * 2;
    // 快速粗调整
    angle -= (int)(angle / M_2PI) * M_2PI;

    // 细调整 (-PI,PI]
    while( angle > M_PI ) {
        angle -= M_2PI;
    }

    while( angle <= -M_PI ) {
        angle += M_2PI;
    }

    return angle;
}
Environment::Environment()
    :sim(SSLWorld::instance())
    ,vm(new VisionPlugin())
//    ,dm_blue_left(new DecisionPlugin(false,false))
    ,c2s_blue(new Cmd2Sim(false))
    ,vision_sender(nullptr){
    feedback.state.resize(OBS_SPACE);
    declare_publish("sim_signal");
    declare_publish("sim_packet");
    declare_publish("zss_cmds");
    declare_receive("zss_vision");

    sim.link(vm,"ssl_vision");
//    vm->link(dm_blue_left,"zss_vision");
    vm->link(this,"zss_vision");
//    dm_blue_left->link(c2s_blue,"zss_cmds");
    c2s_blue->link(&sim,"sim_packet");
    this->link(&sim,"sim_signal");
    this->link(&sim,"sim_packet");
//    dm_blue_left->link(this,"zss_cmds");
}
Environment::Environment(unsigned int port):Environment(){
    vision_sender = new UDPSender("ssl_vision","224.5.23.2",port);
    handle_receiver = new UDPReceiver("sim_packet",20011);
    sim.link(vision_sender,"ssl_vision");
    handle_receiver->link(&sim,"sim_packet");
}
Environment::~Environment(){
    delete vm;
//    delete dm_blue_left;
    delete c2s_blue;
    delete vision_sender;
    delete handle_receiver;
}
void Environment::start_all(){
    sim.start();
    vm->start();
//    dm_blue_left->start();
    c2s_blue->start();
    if(vision_sender)
        vision_sender->start();
    if(handle_receiver)
        handle_receiver->start();
}
FeedBack Environment::reset(){
    static std::random_device rd;
    static std::vector<double> static_action{0.0,0.0,0.0};

    target.setX(remap(rd.min(),rd.max(),-1.0,1.0,double(rd())));
    target.setY(remap(rd.min(),rd.max(),-1.0,1.0,double(rd())));
    velocity.setVector(remap(rd.min(),rd.max(),-3.0,3.0,double(rd()))
                       ,remap(rd.min(),rd.max(),-3.0,3.0,double(rd())));
//    target.fill(0,0);
//    DebugEngine::instance()->gui_debug_msg(CGeoPoint(100,100),QString("target : %1 %2").arg(target.x()).arg(-target.y()).toLatin1(),COLOR_RED);
//    DebugEngine::instance()->gui_debug_msg(CGeoPoint(target.x()*100,-target.y()*100),"TARGET");
//    DebugEngine::instance()->gui_debug_x(CGeoPoint(target.x()*100,-target.y()*100),COLOR_GRAY);
//    DebugEngine::instance()->send(true);
    setBallAndRobot(target,velocity,0,false,0,0);
    cycle = 0;
    std::this_thread::sleep_for(std::chrono::microseconds(10000));
    return this->step(static_action);
}
void Environment::render(){}
FeedBack Environment::step(const std::vector<double>& arr){
    cycle++;
//    DebugEngine::instance()->gui_debug_msg(CGeoPoint(100,100),QString("target : %1 %2").arg(target.x()).arg(-target.y()).toLatin1(),COLOR_RED);
//    DebugEngine::instance()->gui_debug_msg(CGeoPoint(target.x()*100,-target.y()*100),"TARGET");
//    DebugEngine::instance()->gui_debug_x(CGeoPoint(target.x()*100,-target.y()*100),COLOR_GRAY);
//    DebugEngine::instance()->send(true);
    if(arr.size() != 3){
        std::cout << "in step function : action dimention not correct : " << arr.size() << std::endl;
        return {{0.0,0.0},0.0,true};
    }
    sendAction({arr[0],arr[1],arr[2]});
    publish("sim_signal");
    getState(feedback);
    return feedback;
}
void Environment::run(){}
void Environment::setBallAndRobot(const CGeoPoint& p,const CVector& v,int id,bool team,double x,double y,double dir,bool turnon){
    static ZSData data;
    static grSim_Packet packet;
    auto replacement = packet.mutable_replacement();
    auto robot = replacement->add_robots();
    robot->set_x(x);
    robot->set_y(y);
    robot->set_id(id);
    robot->set_dir(dir);
    robot->set_yellowteam(team);
    robot->set_turnon(turnon);

    auto ball = replacement->mutable_ball();
    ball->set_x(p.x());
    ball->set_y(p.y());
    ball->set_vx(v.x());
    ball->set_vy(v.y());

    data.resize(packet.ByteSize());
    packet.SerializeToArray(data.ptr(),packet.ByteSize());
    publish("sim_packet",data);
    packet.Clear();
}
void Environment::getState(FeedBack& feedback){
    static ZSData data;
    static Vision_DetectionFrame frame;
    static CGeoPoint pos;
    static double dir;
    receive("zss_vision",data);
    frame.ParseFromArray(data.data(),data.size());
    int size = frame.robots_blue_size();
    if(size != 1){
        std::cout << "in getState function : robot size not correct : " << size << std::endl;
        return;
    }
    auto robot = frame.robots_blue(0);
    pos.fill(robot.x()/1000.0,robot.y()/1000.0);
    target.fill(frame.ball().x()/1000.0,frame.ball().y()/1000.0);
    velocity.setVector(frame.ball().vel_x()/1000.0,frame.ball().vel_y()/1000.0);
    dir = robot.orientation();
    auto target_dir = (target-pos).dir();
    auto target_dist = (target-pos).mod();
    auto target_dist_max = (target-CGeoPoint(0,0)).mod();
    auto velocity_dir = velocity.dir();
    auto velocity_mod = velocity.mod();
    feedback.reward = (-target_dist)/target_dist_max*2 + (target_dist < limitRange(target_dist_max*0.2,0.1,0.3) ? 500 : 0);
    feedback.done = (cycle > 500 || outside(pos) || target_dist < limitRange(target_dist_max*0.2,0.1,0.3));
    feedback.state[0] = target_dist/7.0;
    feedback.state[1] = (target_dir-dir)/M_PI;
    feedback.state[2] = (velocity_dir-dir)/M_PI;
    feedback.state[3] = velocity_mod/4.0;
}
void Environment::sendAction(const Action& action){
    static ZSData data;
    static grSim_Packet packet;
    auto* cmds = packet.mutable_commands();
    cmds->set_isteamyellow(false);
    cmds->set_timestamp(0.0);
    auto* cmd = cmds->add_robot_commands();
    cmd->set_id(0);
    cmd->set_kickspeedx(0);
    cmd->set_kickspeedz(0);
    cmd->set_veltangent(2*action[0]);
    cmd->set_velnormal(2*action[1]);
    cmd->set_velangular(action[2]);
    cmd->set_spinner(false);
    cmd->set_wheelsspeed(false);
    data.resize(packet.ByteSize());
    packet.SerializeToArray(data.ptr(),packet.ByteSize());
    publish("sim_packet",data);
    packet.Clear();
}
