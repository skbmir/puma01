#include <vscan_usbcan_api/usbcan.h>
#include <vscan_usbcan_api/puma_parameters.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/JointState.h>

ros::Publisher puma_joint_states_pub_, info_pub_;
ros::Subscriber pos_cmd_sub_;
ros::Timer cmd_pub_timer_;

sensor_msgs::JointState joint_states_msg_;
std_msgs::Int16MultiArray info_msg_;

int read_buff_size_ = 20,
    write_buff_size_ = 1;

double  start_time_ = 0.0, cmd_timer_dura_ = 0.01,
        joint_pos_1_ = 0.0, joint_pos_2_ = 0.0, joint_pos_3_ = 0.0,
        joint_pos_cmd_1_ = 0.0, joint_pos_cmd_2_ = 0.0, joint_pos_cmd_3_ = 0.0,
        enc_to_joint_const_1_ = MOTOR_1_ENC_TO_JOINT_CONST, enc_to_joint_const_2_ = MOTOR_2_ENC_TO_JOINT_CONST, enc_to_joint_const_3_ = MOTOR_3_ENC_TO_JOINT_CONST;

vscan_api::usbcan_handle usbcan_handle_; 

char * tty_;
void * can_baudrate_;
DWORD mode_;

std::vector<VSCAN_MSG> read_buffer_;
std::vector<VSCAN_MSG> write_buffer_;

int16_t pos_cmd_1_ = 0, pos_cmd_2_ = 0, pos_cmd_3_ = 0, 
        enc_1_ = 0, enc_2_ = 0, enc_3_ = 0,
        pot_1_ = 0, pot_2_ = 0, pot_3_ = 0,  
        cur_1_ = 0, cur_2_ = 0, cur_3_ = 0,
        vel_1_ = 0, vel_2_ = 0, vel_3_ = 0;

VSCAN_MSG cmd_frame_, heartbeat_frame_, calibration_frame_;

uint32_t feedback_1_id_ = DRV_STATE_ID | DRV_1_CODE | MOTOR_POT_ENC_CUR, feedback_2_id_ = DRV_STATE_ID | DRV_2_CODE | MOTOR_POT_ENC_CUR, feedback_3_id_ = DRV_STATE_ID | DRV_3_CODE | MOTOR_POT_ENC_CUR;

void cmdTimerCB(const ros::TimerEvent& event)
{
    double current_time = event.current_real.toSec();

    // ROS_INFO("Start time: %F",current_time);

    joint_pos_cmd_1_ = (MATH_PI_/6)*sin(0.6*(current_time - start_time_));
    joint_pos_cmd_2_ = (MATH_PI_/12)*sin(0.6*(current_time - start_time_));
    joint_pos_cmd_3_ = (MATH_PI_/8)*sin(0.9*(current_time - start_time_));

    // pos_cmd_1_ = (int16_t)(joint_pos_cmd_1_/enc_to_joint_const_1_);
    // pos_cmd_1_ = 0;
    // if(joint_pos_cmd_1_ > 0){pos_cmd_1_ = 50;} else if(joint_pos_cmd_1_ < 0 ){pos_cmd_1_ = -50;}

    // pos_cmd_2_ = (int16_t)(joint_pos_cmd_2_/enc_to_joint_const_2_);
    // pos_cmd_3_ = (int16_t)(joint_pos_cmd_3_/enc_to_joint_const_3_);

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

        if(usbcan_handle_.writeRequest(&heartbeat_frame_,1)) // write request
        {       
            if(usbcan_handle_.Flush()) // if write request SUCCESS --> it means, that write frames, stored in write buffer, were successfully wrote to CAN
            {
                // ROS_INFO_STREAM("CAN heartbeat!");
            }
        }

    }

    info_msg_.data[0] = pos_cmd_1_;
    info_msg_.data[1] = pos_cmd_2_;
    info_msg_.data[2] = pos_cmd_3_;
    info_pub_.publish(info_msg_);

    // joint_states_msg_.position[0] = joint_pos_cmd_1_;
    // joint_states_msg_.position[1] = joint_pos_cmd_2_;
    // joint_states_msg_.position[2] = joint_pos_cmd_3_;
    // joint_states_msg_.header.stamp = ros::Time::now();
    // puma_joint_states_pub_.publish(joint_states_msg_);

} 

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
                        vel_1_ = usbcan_handle_.getDatafromMsg(read_msg,6);
                        joint_states_msg_.position[0] = enc_1_;
                        joint_states_msg_.effort[0] = cur_1_;
                        joint_states_msg_.velocity[0] = vel_1_;
                    }
                    else if(read_msg.Id==feedback_2_id_)
                    {
                        ROS_INFO("Got CAN-frame with ID: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", read_msg.Id, read_msg.Data[0], read_msg.Data[1], read_msg.Data[2], read_msg.Data[3], read_msg.Data[4], read_msg.Data[5], read_msg.Data[6], read_msg.Data[7]);
                        pot_2_ = usbcan_handle_.getDatafromMsg(read_msg);
                        enc_2_ = usbcan_handle_.getDatafromMsg(read_msg,2);
                        cur_2_ = usbcan_handle_.getDatafromMsg(read_msg,4);
                        vel_2_ = usbcan_handle_.getDatafromMsg(read_msg,6);
                        joint_states_msg_.position[1] = enc_2_;
                        joint_states_msg_.effort[1] = cur_2_;
                        joint_states_msg_.velocity[1] = vel_2_;
                    }
                    else if(read_msg.Id==feedback_3_id_)
                    {
                        ROS_INFO("Got CAN-frame with ID: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", read_msg.Id, read_msg.Data[0], read_msg.Data[1], read_msg.Data[2], read_msg.Data[3], read_msg.Data[4], read_msg.Data[5], read_msg.Data[6], read_msg.Data[7]);
                        pot_3_ = usbcan_handle_.getDatafromMsg(read_msg);
                        enc_3_ = usbcan_handle_.getDatafromMsg(read_msg,2);
                        cur_3_ = usbcan_handle_.getDatafromMsg(read_msg,4);
                        vel_3_ = usbcan_handle_.getDatafromMsg(read_msg,6);
                        joint_states_msg_.position[2] = enc_3_;
                        joint_states_msg_.effort[2] = cur_3_;
                        joint_states_msg_.velocity[2] = vel_3_;
                    }

                    joint_states_msg_.header.stamp = ros::Time::now();
                    puma_joint_states_pub_.publish(joint_states_msg_);
                }
            }
        }

    }else{
        ROS_WARN_STREAM("Reconnecting to USB-CAN adapter and opening port...");
        usbcan_handle_.open(VSCAN_FIRST_FOUND,mode_,can_baudrate_);
        ros::Duration dura(5);
        dura.sleep(); 
    }
}

int main(int argc, char **argv)
{
    std::string n_name = "three_dof_test";
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

    cmd_frame_.Id = TRAJ_CMD_ID | DRV_123_CODE | MOTOR_POS;
    cmd_frame_.Size = 6;
    cmd_frame_.Flags = VSCAN_FLAGS_STANDARD;
    usbcan_handle_.wrapMsgData(cmd_frame_,pos_cmd_1_, 0);
    usbcan_handle_.wrapMsgData(cmd_frame_,pos_cmd_2_, 2);
    usbcan_handle_.wrapMsgData(cmd_frame_,pos_cmd_3_, 4);

    heartbeat_frame_.Id = CAN_HEARTBEAT_ID;
    heartbeat_frame_.Size = 0;
    heartbeat_frame_.Flags = VSCAN_FLAGS_STANDARD;

    calibration_frame_.Id = DRV_MODE_ID | MODE_CALIBR;
    calibration_frame_.Size = 0;
    calibration_frame_.Flags = VSCAN_FLAGS_STANDARD;

    puma_joint_states_pub_ = nh.advertise<sensor_msgs::JointState>("/puma_joint_states",10);
    info_pub_ = nh.advertise<std_msgs::Int16MultiArray>("/puma_joint_states_test_info",10);
    cmd_pub_timer_ = nh.createTimer(ros::Duration(cmd_timer_dura_), cmdTimerCB);
    // pos_cmd_sub_ = nh.subscribe("/motor_pos_cmd",1,&poscmdCB);  

    joint_states_msg_.position.resize(3, 0.0);
    joint_states_msg_.velocity.resize(3, 0.0);
    joint_states_msg_.effort.resize(3, 0.0);
    joint_states_msg_.name = {"joint_1", "joint_2", "joint_3"};
    joint_states_msg_.header.frame_id = ""; //???

    info_msg_.data.resize(3, 0);

// open CAN port
// you can use VSCAN_FIRST_FOUND instead tty
    usbcan_handle_.open(tty,mode,can_baudrate);

    ros::Duration dura(0.05);
    dura.sleep();

    // ros::Rate rate(1000);

    start_time_ = ros::Time::now().toSec();
    // ros::Time::init();

    // ROS_INFO("Start time: %F",start_time_);

    while (ros::ok())
    {
        
        loop();
        // rate.sleep();
        ros::spinOnce();
    }

    if(usbcan_handle_.writeRequest(&calibration_frame_,1)) // write request
    {       
        if(usbcan_handle_.Flush()) // if write request SUCCESS --> it means, that write frames, stored in write buffer, were successfully wrote to CAN
        {

        }
    }
    usbcan_handle_.close();

    return 0;
}


