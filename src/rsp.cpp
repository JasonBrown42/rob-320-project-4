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

int main(int argc, char **argv)
{
    CLParser parser("tf_publisher", "Publishes header messages");
    parser.add_arg(CLParser::Arg("jrdf", "Path to JRDF file", '1'));
    parser.add_opt(CLParser::Opt("ip", "i", "RIX Hub IP Address", "127.0.0.1", "", '1'));
    parser.parse(argc, argv);

    std::string jrdf_file = parser.get_arg("jrdf").front();
    std::string hub_ip = parser.get_opt("ip").front();

    /**
     * TODO: Implement the Robot State Publisher
     */

    return 0;
}