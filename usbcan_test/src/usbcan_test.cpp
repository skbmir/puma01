#include <ros/ros.h>
#include <vscan_usbcan_api/usbcan.h>

int main(int argc, char **argv)
{
    std::string n_name = "usbcan_test";
    std::string devname = "/dev/ttyUSB0";

    // char tty[] = "/dev/ttyUSB0";
    char * tty;
    if(argc>1)
    {
        tty = argv[1];
    }else{
        tty = new char[devname.length()+1];
        strcpy(tty,devname.c_str());
    }

    ros::init(argc, argv, n_name);
    ros::NodeHandle nh;

    int read_buff_size = 3,
        write_buff_size = 3;

// USB-CAN init handle
    vscan_api::usbcan_handle usbcan_handle; 

// open CAN port
    ROS_INFO_STREAM_NAMED(n_name, "Connecting to USB-CAN adapter and opening port...");

    // you can use VSCAN_FIRST_FOUND instead tty
    if(!usbcan_handle.open(tty,VSCAN_MODE_NORMAL,VSCAN_SPEED_1M))
    {
        ROS_ERROR_STREAM_NAMED(n_name, "Failed to connect to USB-CAN adapter and open port! Status: " << usbcan_handle.getStatusString());
    }else{
        ROS_INFO_STREAM_NAMED(n_name, "Successfuly connected to USB-CAN adapter and opened port! Status: " << usbcan_handle.getStatusString());
    }

// define read buffer
    std::vector<VSCAN_MSG> test_read_buffer;
    test_read_buffer.resize(read_buff_size);

// define CAN frame to send
    std::vector<VSCAN_MSG> test_write_buffer;
    // test_write_buffer.resize(write_buff_size);

    // test_write_buffer[0].Id = 0x123;
    // test_write_buffer[0].Size = 4;
    // test_write_buffer[0].Flags = VSCAN_FLAGS_STANDARD;
    // test_write_buffer[0].Data[0] = 0x00;
    // test_write_buffer[0].Data[1] = 0x01;
    // test_write_buffer[0].Data[2] = 0x02;
    // test_write_buffer[0].Data[3] = 0x03;

    // test_write_buffer[1].Id = 0x321;
    // test_write_buffer[1].Size = 2;
    // test_write_buffer[1].Flags = VSCAN_FLAGS_STANDARD;
    // test_write_buffer[1].Data[0] = 0x01;
    // test_write_buffer[1].Data[1] = 0x02;

    VSCAN_MSG test_frame;
    test_frame.Id = 0x123;
    test_frame.Size = 4;
    test_frame.Flags = VSCAN_FLAGS_STANDARD;
    test_frame.Data[0] = 0x00;
    test_frame.Data[1] = 0x01;
    test_frame.Data[2] = 0x02;
    test_frame.Data[3] = 0x03;

    test_write_buffer.push_back(test_frame);

    VSCAN_MSG system_err;
    system_err.Id = 0xfff;
    system_err.Size = 0;
    system_err.Flags = VSCAN_FLAGS_REMOTE;

    ros::Rate rate(0.3);

    while (ros::ok())
    {
        if(usbcan_handle.noError())
        {
        
    // write request
            if(usbcan_handle.writeRequest(test_write_buffer.data(),test_write_buffer.size()))
            {
                // if write request SUCCESS --> it means, that write frames, stored in write buffer, were successfully wrote to CAN
                if(usbcan_handle.Flush())
                {
                    ROS_INFO_STREAM_NAMED(n_name, "Wrote "<< usbcan_handle.getActualWriteNum() <<" CAN-frames!");
                }
            }else{
                ROS_ERROR_STREAM_NAMED(n_name, "Failed to WRITE data to USB-CAN adapter. Status: " << usbcan_handle.getStatusString());
            }

            // sleep(0.05);

    // read request
            if(usbcan_handle.readRequest(test_read_buffer.data(),test_read_buffer.size()))
            {
                // if read request SUCCESS --> frames, read from CAN, store in read buffer
                ROS_INFO_STREAM_NAMED(n_name, "Read " << usbcan_handle.getActualReadNum() << " CAN-frames.");
                if(usbcan_handle.getActualReadNum()>0)
                {
                    for(VSCAN_MSG read_msg : test_read_buffer)
                    {
                        ROS_INFO_STREAM_NAMED(n_name, "Got CAN-frame with ID: " << read_msg.Id);
                    }
                }
            }else{
                ROS_ERROR_STREAM_NAMED(n_name, "Failed to READ data from USB-CAN adapter. Status: " << usbcan_handle.getStatusString());
            }


        }else{
            ROS_ERROR_STREAM_NAMED(n_name, "Error detected: " << usbcan_handle.getStatusString());
            
            ROS_WARN_STREAM_NAMED(n_name, "Reconnecting to USB-CAN adapter and opening port...");

            usbcan_handle.open(tty,VSCAN_MODE_NORMAL,VSCAN_SPEED_1M);

        }

        rate.sleep();
    }


    return 0;
}


