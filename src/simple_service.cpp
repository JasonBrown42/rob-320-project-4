#include "rix/util/log.hpp"
#include "rix/core/common.hpp"
#include "rix/core/node.hpp"
#include "rix/core/service.hpp"

#include "rix/msg/standard/String.hpp"
#include "rix/msg/standard/UInt32.hpp"

using namespace rix::core;
using namespace rix::util;


using String = rix::msg::standard::String;
using UInt32 = rix::msg::standard::UInt32;

void service_callback(const String &request, UInt32 &response) {
    // TODO: Take the XOR checksum of the input string and store it in response.data

}   

int main() {
    // TODO: Initialize the node with name "xor_checksum", IP address "127.0.0.1", and port RIX_HUB_PORT

    // TODO: Advertise the service "xor_checksum" with the service_callback function

    // TOOD: Check if the service was advertised successfully

    // TODO: Spin the node in the foreground

    return 0;
}