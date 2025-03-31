#define RIX_UTIL_LOG_LEVEL 0

#include <functional>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "rix/util/cl_parser.hpp"
#include "rix/util/log.hpp"
#include "rix/util/timing.hpp"

#include "rix/core/common.hpp"
#include "rix/core/node.hpp"
#include "rix/core/publisher.hpp"
#include "rix/core/subscriber.hpp"

#include "rix/msg/geometry/TF.hpp"
#include "rix/msg/sensor/JS.hpp"

#include "rix/rdf/tree.hpp"

using Log = rix::util::Log;
using CLParser = rix::util::CLParser;

/**
 * TODO: Declare any classes or helper functions you need here.
 */

 #include "nlohmann/json.hpp"
 using namespace nlohmann;
 using namespace rix;
 using namespace rdf;
 
 class RSSub {
     public:
      
      RSSub(Tree &tree, double &rate);
     
      void jsp_callback(const rix::msg::sensor::JS &js);
      rix::msg::geometry::TF get_tf();
      bool init(const std::string &name, const std::string &hub_ip, uint16_t hub_port);
      void spin();

 
     private:
      Tree &tree;
      std::mutex rsp_callback_mtx;
      double rate;
      rix::util::Timer stopwatch = rix::util::Timer();
      int64_t last_ms = 0;
      rix::core::Node n;
      std::function<void(rix::msg::sensor::JS)> method;
      rix::core::Subscriber sub;
      rix::core::Publisher pub;
 
 };
 
 RSSub::RSSub(Tree &tree, double &rate): tree(tree), rate(rate){}
 
 void RSSub::jsp_callback(const rix::msg::sensor::JS &js){
     this->rsp_callback_mtx.lock();
    //  std::cout << "Saw message, before publish" << std::endl;
     int64_t cur_time = stopwatch.elapsed_ms();
     if(cur_time - last_ms > (int64_t) (1000/rate)){
        this->tree.set_state(js);
        // std::cout << "About to try to publish" << std::endl;
        rix::msg::geometry::TF msg = this->get_tf();
        this->pub.publish(msg);
        last_ms = cur_time;
     }
     

     this->rsp_callback_mtx.unlock();
 }
 
 rix::msg::geometry::TF RSSub::get_tf(){
     this->rsp_callback_mtx.lock();
     rix::msg::geometry::TF out_tf = this->tree.TF();
     this->rsp_callback_mtx.unlock();
     return out_tf;
 }

 bool RSSub::init(const std::string &name, const std::string &hub_ip, uint16_t hub_port){
    stopwatch.start();
    bool status = n.init(name, hub_ip, hub_port);

    method = [&](rix::msg::sensor::JS js) {this->jsp_callback(js);};
    sub = n.subscribe<rix::msg::sensor::JS>("joint_states", method);
    pub = n.advertise<rix::msg::geometry::TF, rix::core::PubImplTCP>("tf");

    return status;
 }

 void RSSub::spin(){
    n.spin(false);
    rix::util::Rate ratings(1);
    while(n.ok()){
        // std::cout << "Spinning round and round" << std::endl;
        ratings.sleep();
    }

 }
 





int main(int argc, char **argv)
{
    CLParser parser("tf_publisher", "Publishes header messages");
    parser.add_arg(CLParser::Arg("jrdf", "Path to JRDF file", '1'));
    parser.add_opt(CLParser::Opt("ip", "i", "RIX Hub IP Address", "127.0.0.1", "", '1'));
    parser.add_opt(CLParser::Opt("rate", "r", "Max publish rate in rate", "20", "", '1'));
    parser.parse(argc, argv);

    std::string jrdf_file = parser.get_arg("jrdf").front();
    std::string hub_ip = parser.get_opt("ip").front();
    double rate = std::stod(parser.get_opt("rate").front());
    
    /**
     * TODO: Implement the Robot State Publisher
     */

    // rix::core::Node n;
    // bool status = n.init("RSP", hub_ip, RIX_HUB_PORT);
    
    std::ifstream path(jrdf_file);
    json data = json::parse(path);
    Tree tree(data);
    RSSub rs_sub(tree, rate);

    bool status = rs_sub.init("RSP", hub_ip, RIX_HUB_PORT);
    
    if(!status){
        perror("Failed node init\n");
        return -1;
    }
    rs_sub.spin();



    



    return 0;
}