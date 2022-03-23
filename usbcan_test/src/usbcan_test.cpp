#include <vscan_usbcan_api/usbcan.h>
#include <std_msgs/Int16MultiArray.h>

int main(int argc, char **argv)
{
    std::string n_name = "usbcan_test";
    std::string devname = "/dev/ttyUSB0";
    DWORD mode = VSCAN_MODE_NORMAL;
    auto can_baudrate = VSCAN_SPEED_500K;

    // char tty[] = "/dev/ttyUSB0";
    char * tty;
    if(argc>1)
    {
        tty = argv[1];
        if(argc>2)
        {
            if(!strcmp(argv[2],"listen"))
            {
                mode = VSCAN_MODE_LISTEN_ONLY;
            }else if(!strcmp(argv[2],"self")){
                mode = VSCAN_MODE_SELF_RECEPTION;
            }
        }
    }else{
        tty = new char[devname.length()+1];
        strcpy(tty,devname.c_str());
    }

    ros::init(argc, argv, n_name);
    ros::NodeHandle nh;

    ros::Publisher pos_pub = nh.advertise<std_msgs::Int16MultiArray>("/motor_pos",1);

    std_msgs::Int16MultiArray motor_pos_msg;

    motor_pos_msg.data.resize(2,0);

    int read_buff_size = 10,
        write_buff_size = 3;

// USB-CAN init handle
    vscan_api::usbcan_handle usbcan_handle; 

// open CAN port
    // you can use VSCAN_FIRST_FOUND instead tty
    usbcan_handle.open(tty,mode,can_baudrate);
    

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

    int16_t test_value = 2773;

    int16_t enc = 0, pot = 0;

    VSCAN_MSG test_frame;
    test_frame.Id = 0x12fff;
    test_frame.Size = 4;
    test_frame.Flags = VSCAN_FLAGS_STANDARD;
    // test_frame.Data[0] = test_value>>24;
    // test_frame.Data[1] = test_value>>16;
    // test_frame.Data[2] = test_value>>8;
    // test_frame.Data[3] = test_value;
    usbcan_handle.wrapMsgData(test_frame,test_value);

    test_write_buffer.push_back(test_frame);

    VSCAN_MSG system_err;
    system_err.Id = 0x7ff;
    system_err.Size = 0;
    system_err.Flags = VSCAN_FLAGS_REMOTE;

    // test_write_buffer.push_back(system_err);

    // ros::Rate rate(1000);

    while (ros::ok())
    {
        if(usbcan_handle.noError())
        {
        
    // write request
            // if(usbcan_handle.writeRequest(test_write_buffer.data(),test_write_buffer.size()))
            // {
            //     // if write request SUCCESS --> it means, that write frames, stored in write buffer, were successfully wrote to CAN
            //     if(usbcan_handle.Flush())
            //     {
            //         ROS_INFO_STREAM("Wrote "<< usbcan_handle.getActualWriteNum() <<" CAN-frames!");
            //     }
            // }

            // sleep(0.1);

    // read request
            if(usbcan_handle.readRequest(test_read_buffer.data(),test_read_buffer.size()))
            {
                // if read request SUCCESS --> frames, read from CAN, store in read buffer
                if(usbcan_handle.getActualReadNum()>0)
                {
                    // ROS_INFO_STREAM("Read " << usbcan_handle.getActualReadNum() << " CAN-frames.");
                    for(VSCAN_MSG read_msg : test_read_buffer)
                    {
                        // ROS_INFO("Got CAN-frame with ID: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", read_msg.Id, read_msg.Data[0], read_msg.Data[1], read_msg.Data[2], read_msg.Data[3], read_msg.Data[4], read_msg.Data[5], read_msg.Data[6], read_msg.Data[7]);
                        if(read_msg.Id==0x004)
                        {
                            pot = usbcan_handle.getDatafromMsg(read_msg);
                            enc = usbcan_handle.getDatafromMsg(read_msg,4);
                            motor_pos_msg.data[0] = pot;
                            motor_pos_msg.data[1] = enc;
                            // ROS_INFO("enc: %i, pot: %i", enc, pot);
                            pos_pub.publish(motor_pos_msg);
                        }
                    }
                }
            }


        }else{
            ROS_WARN_STREAM("Reconnecting to USB-CAN adapter and opening port...");
            usbcan_handle.open(VSCAN_FIRST_FOUND,mode,can_baudrate);
            sleep(5);
        }

        ros::spinOnce();
        // rate.sleep();
    }


    return 0;
}


