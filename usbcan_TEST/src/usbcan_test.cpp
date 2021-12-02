#include <ros/ros.h>
#include <puma01_usbcan/usbcan.h>

int main(int argc, char** argv)
{
    std::string n_name = "usbcan_test";

    char tty[] = "/dev/ttyUSB0";

    ros::init(argc, argv, n_name);
    ros::NodeHandle nh;

// USB-CAN init handle
    puma01_usbcan::VSCAN_serial_handler usbcan_handler; 

// open CAN port
    ROS_INFO_STREAM_NAMED(n_name, "Connecting to USB-CAN adapter and opening port...");
    if(!usbcan_handler.open(VSCAN_FIRST_FOUND,VSCAN_MODE_NORMAL,VSCAN_SPEED_1M))
    {
        ROS_ERROR_STREAM_NAMED(n_name, "Failed to connect to USB-CAN adapter and open port! Status: " << usbcan_handler.getStatusString() << std::endl);
    }else{
        ROS_INFO_STREAM_NAMED(n_name, "Successfuly connected to USB-CAN adapter and opened port! Status: " << usbcan_handler.getStatusString() << std::endl);
    }

// define CAN frame to send
    usbcan_handler.write_buffer.Id = 0x100;
    usbcan_handler.write_buffer.Size = 4;
    usbcan_handler.write_buffer.Flags = VSCAN_FLAGS_STANDARD;
    usbcan_handler.write_buffer.Data[0] = 0x00;
    usbcan_handler.write_buffer.Data[1] = 0x01;
    usbcan_handler.write_buffer.Data[2] = 0x02;
    usbcan_handler.write_buffer.Data[3] = 0x03;

    ros::Rate rate(0.5);

    while (ros::ok())
    {
// write


//         if(usbcan_handler.writeRequest())
//         {
//             if(usbcan_handler.Flush())
//             {
//                 ROS_INFO_STREAM_NAMED(n_name, "Wrote CAN-frames!"<< std::endl);
//             }
//         }else{
//             ROS_ERROR_STREAM_NAMED(n_name, "Failed to WRITE data to USB-CAN adapter. Status: " << usbcan_handler.getStatusString() << std::endl);
//         }

// // read
//         if(usbcan_handler.readRequest())
//         {
//             ROS_INFO_STREAM_NAMED(n_name, "Got CAN-frame with ID: " << usbcan_handler.read_buffer.Data << std::endl);
//         }else{
//             ROS_ERROR_STREAM_NAMED(n_name, "Failed to READ data from USB-CAN adapter. Status: " << usbcan_handler.getStatusString() << std::endl);
//         }

        rate.sleep();
    }


    return 0;
}
