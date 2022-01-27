#include <ros/ros.h>
#include <vscan_usbcan_api/usbcan.h>

int main(int argc, char **argv)
{
    std::string n_name = "usbcan_test";
    std::string devname = "/dev/ttyUSB0";
    DWORD mode = 0;

    // char tty[] = "/dev/ttyUSB0";
    char * tty;
    if(argc>1)
    {
        tty = argv[1];
        if(argc>2)
        {
            if(argv[2]=="normal")
            {
                mode = 0;
            }else if(argv[2]=="self-recept"){
                mode = 2;
            }
        }
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
    ROS_INFO_STREAM("Connecting to USB-CAN adapter and opening port...");

    // you can use VSCAN_FIRST_FOUND instead tty
    if(!usbcan_handle.open(tty,VSCAN_MODE_SELF_RECEPTION,VSCAN_SPEED_1M))
    {
        ROS_ERROR_STREAM("Failed to connect to USB-CAN adapter and open port! Status: " << usbcan_handle.getStatusString());
    }else{
        ROS_INFO_STREAM("Successfuly connected to USB-CAN adapter and opened port! Status: " << usbcan_handle.getStatusString());
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
    system_err.Id = 0x2ff;
    system_err.Size = 0;
    system_err.Flags = VSCAN_FLAGS_REMOTE;

    test_write_buffer.push_back(system_err);

    ros::Rate rate(0.5);

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
                    ROS_INFO_STREAM("Wrote "<< usbcan_handle.getActualWriteNum() <<" CAN-frames!");
                }
            }else{
                ROS_ERROR_STREAM("Failed to WRITE data to USB-CAN adapter. Status: " << usbcan_handle.getStatusString());
            }

            sleep(0.1);

    // read request
            if(usbcan_handle.readRequest(test_read_buffer.data(),test_read_buffer.size()))
            {
                // if read request SUCCESS --> frames, read from CAN, store in read buffer
                ROS_INFO_STREAM("Read " << usbcan_handle.getActualReadNum() << " CAN-frames.");
                if(usbcan_handle.getActualReadNum()>0)
                {
                    for(VSCAN_MSG read_msg : test_read_buffer)
                    {
                        if(read_msg.Id==0x123)
                        {
                            ROS_INFO("Got CAN-frame with ID: %03x, Data: %02x %02x %02x %02x", read_msg.Id, read_msg.Data[0], read_msg.Data[1], read_msg.Data[2], read_msg.Data[3]);
                        }

                    }
                }
            }else{
                ROS_ERROR_STREAM("Failed to READ data from USB-CAN adapter. Status: " << usbcan_handle.getStatusString());
            }


        }else{
            ROS_ERROR_STREAM("Error detected: " << usbcan_handle.getStatusString());
            
            ROS_WARN_STREAM("Reconnecting to USB-CAN adapter and opening port...");

            usbcan_handle.open(tty,VSCAN_MODE_NORMAL,VSCAN_SPEED_1M);

        }

        rate.sleep();
    }


    return 0;
}


