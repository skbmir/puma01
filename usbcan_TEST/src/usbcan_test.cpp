#include <ros/ros.h>
#include <puma01_usbcan/usbcan.h>

void main(int argc, char** argv)
{
    std::string n_name = "usbcan_test";

    ros::init(argc, argv, n_name);
    ros::NodeHandle nh;

    puma01_usbcan::VSCAN_serial_handler usbcan_handler; 

    ROS_INFO_NAMED(n_name, "Connecting to USB-CAN adapter...");
    if(!usbcan_handler.open())
    {
        ROS_ERROR_STREAM_NAMED(n_name, "Failed to connect to USB-CAN adapter! Status: " << usbcan_handler.getStatusString() << std::endl);
    }else{
        ROS_INFO_STREAM_NAMED(n_name, "Successfuly connected to USB-CAN adapter! Status: " << usbcan_handler.getStatusString() << std::endl);
    }

    ros::Rate rate(1);

    while (ros::ok())
    {


// read
        if(usbcan_handler.readRequest())
        {
            ROS_INFO_STREAM_NAMED(n_name, "Got CAN-frame with ID: " << usbcan_handler.read_buffer.Data << std::endl);
        }else{
            ROS_ERROR_STREAM_NAMED(n_name, "Failed to read data from USB-CAN adapter. Status: " << usbcan_handler.getStatusString() << std::endl);
        }


        rate.sleep();
    }

}
