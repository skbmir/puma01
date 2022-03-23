#include <vscan_usbcan_api/usbcan.h>
#include <std_msgs//Int16MultiArray.h>


class usbcan_test_write
{
private:
    ros::Publisher info_pub_;
    ros::Subscriber pos_cmd_sub_;

    std_msgs::Int16MultiArray info_msg_;

    int read_buff_size_,
        write_buff_size_;

    vscan_api::usbcan_handle usbcan_handle_; 

    char * tty_;
    void * can_baudrate_;
    DWORD mode_;

    std::vector<VSCAN_MSG> read_buffer_;
    std::vector<VSCAN_MSG> write_buffer_;

    int rot_dir_;
    int16_t pos_cmd_, enc_, pot_, vel_;

public:
    usbcan_test_write() {}
    ~usbcan_test_write() {}

    void init(ros::NodeHandle &nh, char * tty, DWORD mode, void * can_baudrate) 
    {

        can_baudrate_ = can_baudrate;
        mode_ = mode;

        read_buff_size_ = 10;
        write_buff_size_ = 1;

        pos_cmd_ = 0;
        enc_ = 0;
        pot_ = 0;
        vel_ = 0;
        rot_dir_ = 0;

        read_buffer_.resize(read_buff_size_);
        // write_buffer_.resize(write_buff_size_);

        VSCAN_MSG cmd_frame;
        cmd_frame.Id = 0x002;
        cmd_frame.Size = 4;
        cmd_frame.Flags = VSCAN_FLAGS_STANDARD;
        // cmd_frame.Data[0] = pos_cmd_>>24;
        // cmd_frame.Data[1] = pos_cmd_>>16;
        // cmd_frame.Data[2] = pos_cmd_>>8;
        // cmd_frame.Data[3] = pos_cmd_;
        usbcan_handle_.wrapMsgData(cmd_frame,pos_cmd_);


        // int16_t test_val_1 = 1, test_val_2 = 1, test_val_3 = 0, test_val_4 = 0;
        // VSCAN_MSG test_frame;
        // test_frame.Id = 0x002;
        // test_frame.Size = 8;
        // test_frame.Flags = VSCAN_FLAGS_STANDARD;
        // usbcan_handle_.wrapMsgData(test_frame,test_val_1);
        // usbcan_handle_.wrapMsgData(test_frame,test_val_2,2);
        // usbcan_handle_.wrapMsgData(test_frame,test_val_3,4);
        // usbcan_handle_.wrapMsgData(test_frame,test_val_4,6);

        // ROS_INFO("%i, %i, %i, %i", test_val_1, test_val_2, test_val_3, test_val_4);
        // ROS_INFO("Data: %02x %02x %02x %02x %02x %02x %02x %02x", test_frame.Data[0], test_frame.Data[1], test_frame.Data[2], test_frame.Data[3], test_frame.Data[4], test_frame.Data[5], test_frame.Data[6], test_frame.Data[7]);
                           
        // test_val_1 = usbcan_handle_.getDatafromMsg(test_frame);
        // test_val_2 = usbcan_handle_.getDatafromMsg(test_frame,2);
        // test_val_3 = usbcan_handle_.getDatafromMsg(test_frame,4);
        // test_val_4 = usbcan_handle_.getDatafromMsg(test_frame,6);

        // ROS_INFO("%i, %i, %i, %i", test_val_1, test_val_2, test_val_3, test_val_4);

        info_pub_ = nh.advertise<std_msgs::Int16MultiArray>("/motor_pos",1);
        pos_cmd_sub_ = nh.subscribe("/motor_pos_cmd",1,&usbcan_test_write::poscmdCB,this);  

        info_msg_.data.resize(3,0);

    // open CAN port
    // you can use VSCAN_FIRST_FOUND instead tty
        usbcan_handle_.open(tty,mode,can_baudrate);

    }

    void loop()
    {
       if(usbcan_handle_.noError())
        {
            if(usbcan_handle_.readRequest(read_buffer_.data(),read_buffer_.size())) // read request
            {
                if(usbcan_handle_.getActualReadNum()>0) // if read request SUCCESS --> frames, read from CAN, store in read buffer
                {
                    // ROS_INFO_STREAM("Read " << usbcan_handle_.getActualReadNum() << " CAN-frames.");
                    for(VSCAN_MSG read_msg : read_buffer_)
                    {
                        if(read_msg.Id==0x004)
                        {
                            // ROS_INFO("Got CAN-frame with ID: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", read_msg.Id, read_msg.Data[0], read_msg.Data[1], read_msg.Data[2], read_msg.Data[3], read_msg.Data[4], read_msg.Data[5], read_msg.Data[6], read_msg.Data[7]);
                            pot_ = usbcan_handle_.getDatafromMsg(read_msg);
                            enc_ = usbcan_handle_.getDatafromMsg(read_msg,2);
                            vel_ = usbcan_handle_.getDatafromMsg(read_msg,4);
                            // ROS_INFO("pot: %i, enc: %i", pot_, enc_);
                            info_msg_.data[0] = pot_;
                            info_msg_.data[1] = enc_;
                            info_msg_.data[2] = vel_;
                            // ROS_INFO("enc: %i, pot: %i", enc, pot);
                            
                        }
                        // if(read_msg.Id==0x006)
                        // {
                        //     // ROS_INFO("Got CAN-frame with ID: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", read_msg.Id, read_msg.Data[0], read_msg.Data[1], read_msg.Data[2], read_msg.Data[3], read_msg.Data[4], read_msg.Data[5], read_msg.Data[6], read_msg.Data[7]);
                        //     info_msg_.data[2] = usbcan_handle_.getDatafromMsg(read_msg);
                        // }
                        info_pub_.publish(info_msg_);
                    }
                }
            }
   
            // if(usbcan_handle_.writeRequest(write_buffer_.data(),write_buffer_.size())) // write request
            // {       
            //     if(usbcan_handle_.Flush()) // if write request SUCCESS --> it means, that write frames, stored in write buffer, were successfully wrote to CAN
            //     {
            //         // ROS_INFO_STREAM("Wrote command: "<< pos_cmd_);
            //     }
            // }

        }else{
            ROS_WARN_STREAM("Reconnecting to USB-CAN adapter and opening port...");
            usbcan_handle_.open(VSCAN_FIRST_FOUND,mode_,can_baudrate_);
            ros::Duration dura(5);
            dura.sleep(); 
        }
    }

    void poscmdCB(const std_msgs::Int16MultiArrayConstPtr &cmd)
    {
        pos_cmd_ = cmd->data[0];

        // write_buffer_[0].Data[0] = pos_cmd_>>24;
        // write_buffer_[0].Data[1] = pos_cmd_>>16;
        // write_buffer_[0].Data[2] = pos_cmd_>>8;
        // write_buffer_[0].Data[3] = pos_cmd_;
        // write_buffer_[0].Data[4] = rot_dir_;
        usbcan_handle_.wrapMsgData(write_buffer_[0],pos_cmd_);
        

        if(usbcan_handle_.noError())
        {        
            if(usbcan_handle_.writeRequest(write_buffer_.data(),write_buffer_.size())) // write request
            {       
                if(usbcan_handle_.Flush()) // if write request SUCCESS --> it means, that write frames, stored in write buffer, were successfully wrote to CAN
                {
                    // ROS_INFO_STREAM("Wrote command: "<< pos_cmd_);
                }
            }
        }
    } 

}; // class

int main(int argc, char **argv)
{
    std::string n_name = "usbcan_test_write";
    std::string devname = "/dev/ttyUSB0";
    DWORD mode = VSCAN_MODE_NORMAL;
    void * can_baudrate = VSCAN_SPEED_500K;

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
    
    usbcan_test_write usbcan_test_write_inst;

    usbcan_test_write_inst.init(nh, tty, mode, can_baudrate);

    ros::Duration dura(0.05);
    dura.sleep();

    // ros::Rate rate(1000);

    while (ros::ok())
    {
        
        usbcan_test_write_inst.loop();
        // rate.sleep();
        ros::spinOnce();
    }


    return 0;
}


