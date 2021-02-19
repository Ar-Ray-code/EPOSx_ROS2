# ifndef EPOS_BASE_HPP
# define EPOS_BASE_HPP

#include "Definitions.h"

// ROS include ------------------------------

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
//---------------------------------------

#define MMC_SUCCESS 0
#define MMC_FAILED 1

typedef void* HANDLE;

typedef struct {
    HANDLE g_pKeyHandle;
    unsigned short g_usNodeId;
    std::string g_deviceName;
    std::string g_protocolStackName;
    std::string g_interfaceName;
    std::string g_portName;
    int g_baudrate;
} epos_device;

class epos_base: public rclcpp::Node
{

private:
    typedef int BOOL;
    std::string txt_path;
    epos_device epos;
    unsigned int ulErrorCode;

public:

    epos_base(const std::string &name_space);
    ~epos_base();

    void init_velocity();
    void VelocityMode(const std_msgs::msg::Int32::SharedPtr spd);
    // void VelocityMode();
    void SeparatorLine();
    void LogError(std::string functionName, int p_lResult, unsigned int p_ulErrorCode);
    void LogInfo(std::string message);
    void PrintHeader();
    void PrintSettings();
    void OpenDevice(unsigned int* p_pErrorCode);
    void CloseDevice(unsigned int* p_pErrorCode);
    void PrepareDemo(unsigned int* p_pErrorCode);
    void PrintAvailableInterfaces();
    void PrintAvailablePorts(char* p_pInterfaceNameSel);
    void PrintAvailableProtocols();

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_speed;

};

#endif