#include <vscan_usbcan_api/usbcan.h>
#include <std_msgs//Int16MultiArray.h>

ros::Publisher info_pub_;
ros::Subscriber pos_cmd_sub_;

std_msgs::Int16MultiArray info_msg_;

int read_buff_size_ = 20,
    write_buff_size_ = 1;

vscan_api::usbcan_handle usbcan_handle_; 

char * tty_;
void * can_baudrate_;
DWORD mode_;

std::vector<VSCAN_MSG> read_buffer_;
std::vector<VSCAN_MSG> write_buffer_;

int16_t pos_cmd_1_ = 0, pos_cmd_2_ = 0, pos_cmd_3_ = 0, 
        enc_1_ = 0, enc_2_ = 0, enc_3_ = 0,
        pot_1_ = 0, pot_2_ = 0, pot_3_ = 0,  
        cur_1_ = 0, cur_2_ = 0, cur_3_ = 0;

VSCAN_MSG cmd_frame_, heartbeat_frame_;

uint32_t feedback_1_id_ = DRV_STATE_ID | DRV_1_CODE | MOTOR_POT_ENC_CUR, feedback_2_id_ = DRV_STATE_ID | DRV_2_CODE | MOTOR_POT_ENC_CUR, feedback_3_id_ = DRV_STATE_ID | DRV_3_CODE | MOTOR_POT_ENC_CUR;


void loop()
{
    if(usbcan_handle_.noError())
    {
        if(usbcan_handle_.readRequest(read_buffer_.data(),read_buffer_.size())) // read request
        {
            if(usbcan_handle_.getActualReadNum()>0) // if read request SUCCESS --> frames, read from CAN, store in read buffer
            {
                ROS_INFO_STREAM("Read " << usbcan_handle_.getActualReadNum() << " CAN-frames.");
                for(VSCAN_MSG read_msg : read_buffer_)
                {
                    if(read_msg.Id==feedback_1_id_)
                    {
                        ROS_INFO("Got CAN-frame with ID: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", read_msg.Id, read_msg.Data[0], read_msg.Data[1], read_msg.Data[2], read_msg.Data[3], read_msg.Data[4], read_msg.Data[5], read_msg.Data[6], read_msg.Data[7]);
                        pot_1_ = usbcan_handle_.getDatafromMsg(read_msg);
                        enc_1_ = usbcan_handle_.getDatafromMsg(read_msg,2);
                        cur_1_ = usbcan_handle_.getDatafromMsg(read_msg,4);
                        info_msg_.data[0] = pot_1_;
                        info_msg_.data[1] = enc_1_;
                        info_msg_.data[2] = cur_1_;
                    }
                    else if(read_msg.Id==feedback_2_id_)
                    {
                        ROS_INFO("Got CAN-frame with ID: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", read_msg.Id, read_msg.Data[0], read_msg.Data[1], read_msg.Data[2], read_msg.Data[3], read_msg.Data[4], read_msg.Data[5], read_msg.Data[6], read_msg.Data[7]);
                        pot_2_ = usbcan_handle_.getDatafromMsg(read_msg);
                        enc_2_ = usbcan_handle_.getDatafromMsg(read_msg,2);
                        cur_2_ = usbcan_handle_.getDatafromMsg(read_msg,4);
                        info_msg_.data[3] = pot_2_;
                        info_msg_.data[4] = enc_2_;
                        info_msg_.data[5] = cur_2_;
                    }
                    else if(read_msg.Id==feedback_3_id_)
                    {
                        ROS_INFO("Got CAN-frame with ID: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", read_msg.Id, read_msg.Data[0], read_msg.Data[1], read_msg.Data[2], read_msg.Data[3], read_msg.Data[4], read_msg.Data[5], read_msg.Data[6], read_msg.Data[7]);
                        pot_3_ = usbcan_handle_.getDatafromMsg(read_msg);
                        enc_3_ = usbcan_handle_.getDatafromMsg(read_msg,2);
                        cur_3_ = usbcan_handle_.getDatafromMsg(read_msg,4);
                        info_msg_.data[6] = pot_3_;
                        info_msg_.data[7] = enc_3_;
                        info_msg_.data[8] = cur_3_;
                    }

                    info_pub_.publish(info_msg_);
                }
            }
        }

        static int time_index_ = 0;
        // if(time_index_>=1000)
        // {
            if(usbcan_handle_.writeRequest(&heartbeat_frame_,1)) // write request
            {       
                if(usbcan_handle_.Flush()) // if write request SUCCESS --> it means, that write frames, stored in write buffer, were successfully wrote to CAN
                {
                    ROS_INFO_STREAM("CAN heartbeat!");
                }
            }
        // }
        // time_index_++;

    }else{
        ROS_WARN_STREAM("Reconnecting to USB-CAN adapter and opening port...");
        usbcan_handle_.open(VSCAN_FIRST_FOUND,mode_,can_baudrate_);
        ros::Duration dura(5);
        dura.sleep(); 
    }
}

void poscmdCB(const std_msgs::Int16MultiArrayConstPtr &cmd)
{
    pos_cmd_1_ = cmd->data[0];
    pos_cmd_2_ = cmd->data[1];
    pos_cmd_3_ = cmd->data[2];

    usbcan_handle_.wrapMsgData(cmd_frame_,pos_cmd_1_, 0);
    usbcan_handle_.wrapMsgData(cmd_frame_,pos_cmd_2_, 2);
    usbcan_handle_.wrapMsgData(cmd_frame_,pos_cmd_3_, 4);
    

    if(usbcan_handle_.noError())
    {        
        if(usbcan_handle_.writeRequest(&cmd_frame_,1)) // write request
        {       
            if(usbcan_handle_.Flush()) // if write request SUCCESS --> it means, that write frames, stored in write buffer, were successfully wrote to CAN
            {
                // ROS_INFO_STREAM("Wrote command: "<< pos_cmd_);
            }
        }
    }
} 

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
    
    can_baudrate_ = can_baudrate;
    mode_ = mode;

    read_buffer_.resize(read_buff_size_);
    // write_buffer_.resize(write_buff_size_);

    cmd_frame_.Id = TRAJ_CMD_ID | TO_ALL_CODE | MOTOR_POS;
    cmd_frame_.Size = 6;
    cmd_frame_.Flags = VSCAN_FLAGS_STANDARD;
    usbcan_handle_.wrapMsgData(cmd_frame_,pos_cmd_1_, 0);
    usbcan_handle_.wrapMsgData(cmd_frame_,pos_cmd_2_, 2);
    usbcan_handle_.wrapMsgData(cmd_frame_,pos_cmd_3_, 4);
    // write_buffer_.push_back(cmd_frame_);

    heartbeat_frame_.Id = CAN_HEARTBEAT_ID;
    heartbeat_frame_.Size = 0;
    heartbeat_frame_.Flags = VSCAN_FLAGS_STANDARD;
    // write_buffer_.push_back(heartbeat_frame_);

    info_pub_ = nh.advertise<std_msgs::Int16MultiArray>("/motor_pos",10);
    pos_cmd_sub_ = nh.subscribe("/motor_pos_cmd",1,&poscmdCB);  

    info_msg_.data.resize(9,0);

// open CAN port
// you can use VSCAN_FIRST_FOUND instead tty
    usbcan_handle_.open(tty,mode,can_baudrate);

    ros::Duration dura(0.05);
    dura.sleep();

    ros::Rate rate(10);

    while (ros::ok())
    {
        
        loop();
        rate.sleep();
        ros::spinOnce();
    }


    return 0;
}


