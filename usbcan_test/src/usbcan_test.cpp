#include <ros/ros.h>
#include <puma01_usbcan/usbcan.h>

int main(int argc, char** argv)
{
    std::string n_name = "usbcan_test";

    char tty[] = "/dev/ttyUSB0";

    ros::init(argc, argv, n_name);
    ros::NodeHandle nh;

    int read_buff_size = 20,
        write_buff_size = 20;

// USB-CAN init handle
    puma01_usbcan::VSCAN_serial_handler usbcan_handler(write_buff_size,read_buff_size); 

// open CAN port
    ROS_INFO_STREAM_NAMED(n_name, "Connecting to USB-CAN adapter and opening port...");

    // you can use VSCAN_FIRST_FOUND instead tty
    if(!usbcan_handler.open(tty,VSCAN_MODE_NORMAL,VSCAN_SPEED_1M))
    {
        ROS_ERROR_STREAM_NAMED(n_name, "Failed to connect to USB-CAN adapter and open port! Status: " << usbcan_handler.getStatusString() << std::endl);
    }else{
        ROS_INFO_STREAM_NAMED(n_name, "Successfuly connected to USB-CAN adapter and opened port! Status: " << usbcan_handler.getStatusString() << std::endl);
    }


    int iter;

// define CAN frame to send
    for(VSCAN_MSG msg : usbcan_handler.getWriteBuffer())
    {
        msg.Id = (char)iter++;
        msg.Size = 4;
        msg.Flags = VSCAN_FLAGS_STANDARD;
        msg.Data[0] = 0x00;
        msg.Data[1] = 0x01;
        msg.Data[2] = 0x02;
        msg.Data[3] = 0x03;
    }

    ros::Rate rate(0.5);

    while (ros::ok())
    {
        if(usbcan_handler.noError())
        {
        
    // write request
            if(usbcan_handler.writeRequest())
            {
                // if write request SUCCESS --> it means, that write frames, stored in write buffer, were successfully wrote to CAN
                if(usbcan_handler.Flush())
                {
                    ROS_INFO_STREAM_NAMED(n_name, "Wrote "<< usbcan_handler.getActualWriteNum() <<" CAN-frames!"<< std::endl);
                }
            }else{
                ROS_ERROR_STREAM_NAMED(n_name, "Failed to WRITE data to USB-CAN adapter. Status: " << usbcan_handler.getStatusString() << std::endl);
            }

            sleep(0.1);

    // read request
            if(usbcan_handler.readRequest())
            {
                // if read request SUCCESS --> frames, read from CAN, store in read buffer
                ROS_INFO_STREAM_NAMED(n_name, "Read " << usbcan_handler.getActualReadNum() << " CAN-frames." << std::endl);
                if(usbcan_handler.getActualReadNum()>0)
                {
                    for(VSCAN_MSG read_msg : usbcan_handler.getReadBuffer())
                    {
                        ROS_INFO_STREAM_NAMED(n_name, "Got CAN-frame with ID: " << read_msg.Id << std::endl);
                    }
                }
            }else{
                ROS_ERROR_STREAM_NAMED(n_name, "Failed to READ data from USB-CAN adapter. Status: " << usbcan_handler.getStatusString() << std::endl);
            }


        }else{
            ROS_ERROR_STREAM_NAMED(n_name, "Error detected: " << usbcan_handler.getStatusString() << std::endl);
        }

        rate.sleep();
    }


    return 0;
}
