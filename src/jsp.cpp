#define RIX_UTIL_LOG_LEVEL 1

#include <functional>
#include <iostream>
#include <mutex>
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

#include "rix/msg/sensor/JS.hpp"
#include "rix/msg/sensor/JointState.hpp"

#include "rix/rdf/tree.hpp"

using CLParser = rix::util::CLParser;
using Log = rix::util::Log;



/**
 * TODO: Declare any classes or helper functions you need here.
 */

#include "nlohmann/json.hpp"
using namespace nlohmann;
using namespace rix;
using namespace rdf;

class JSSub {
    public:
     
     JSSub(Tree &tree);
    
     void jsp_callback(const rix::msg::sensor::JS &js);
     rix::msg::sensor::JS get_js();

    private:
     Tree &tree;
     std::mutex jsp_callback_mtx;

};

JSSub::JSSub(Tree &tree): tree(tree){}

void JSSub::jsp_callback(const rix::msg::sensor::JS &js){
    this->jsp_callback_mtx.lock();
    this->tree.set_state(js);
    this->jsp_callback_mtx.unlock();
}

rix::msg::sensor::JS JSSub::get_js(){
    this->jsp_callback_mtx.lock();
    rix::msg::sensor::JS out_js = this->tree.JS();
    this->jsp_callback_mtx.unlock();
    return out_js;
}


int main(int argc, char **argv) {
    CLParser parser("joint_state_publisher", "Publishes header messages");
    parser.add_arg(CLParser::Arg("jrdf",   "Path to JRDF file",       '1'));
    parser.add_arg(CLParser::Arg("topics", "Topics to subscribe to",  '+'));
    parser.add_opt(CLParser::Opt("ip",   "i", "RIX Hub IP Address", "127.0.0.1", "", '1'));
    parser.add_opt(CLParser::Opt("rate", "r", "Publish rate in Hz", "50",        "", '1'));
    parser.parse(argc, argv);

    std::string jrdf_file = parser.get_arg("jrdf").front();
    std::vector<std::string> topics = parser.get_arg("topics");
    std::string hub_ip = parser.get_opt("ip").front();
    int rate = std::stoi(parser.get_opt("rate").front());

    /**
     * TODO: Implement the Joint State Publisher
     */
    rix::core::Node n;
    bool status = n.init("JSP", hub_ip, RIX_HUB_PORT);
    if(!status){
        perror("Failed node init\n");
        return -1;
    }

    std::ifstream path(jrdf_file);
    json data = json::parse(path);
    Tree tree(data);
    JSSub js_sub(tree);

    std::vector<rix::core::Subscriber> twitch;
    for(std::string topic : topics){
        auto meth = [&](rix::msg::sensor::JS js) {js_sub.jsp_callback(js);};
        rix::core::Subscriber sub = n.subscribe<rix::msg::sensor::JS>(topic, meth);
        twitch.push_back(sub);
    }

    rix::core::Publisher chat = n.advertise<rix::msg::sensor::JS, rix::core::PubImplTCP>("joint_states");
    
    n.spin(false);

    rix::util::Rate ratings(rate);

    while(n.ok()){
        rix::msg::sensor::JS hey = js_sub.get_js();
        hey.stamp.nsec = (int32_t) rix::util::nanos();
        hey.stamp.sec = (int32_t)( (float_t) rix::util::millis() / 1000.0);
        chat.publish(hey);
        ratings.sleep();
    }


    return 0;
}