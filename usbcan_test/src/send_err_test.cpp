#include <vscan_usbcan_api/usbcan.h>
#include <vscan_usbcan_api/puma_parameters.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/JointState.h>

int read_buff_size_ = 20,
    write_buff_size_ = 1;

double  start_time_ = 0.0, cmd_timer_dura_ = 0.005;

vscan_api::usbcan_handle usbcan_handle_; 

char * tty_;
void * can_baudrate_;
DWORD mode_;

VSCAN_MSG cmd_frame_, heartbeat_frame_, calibration_frame_, cfg_frame_, emergency_frame_, normal_mode_frame_;

int main(int argc, char **argv)
{
    std::string n_name = "three_dof_test";
    std::string devname = "/dev/ttyUSB0";
    DWORD mode = VSCAN_MODE_NORMAL;
    void * can_baudrate = VSCAN_SPEED_100K;

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

    float new_gain = 1.1;
    cfg_frame_.Id = DRV_CFG_ID | CFG_PID_P | DRV_1_CODE;
    // cfg_frame_.Id = DRV_CFG_ID | CFG_PID_I | DRV_1_CODE;
    // cfg_frame_.Id = DRV_CFG_ID | CFG_PID_D | DRV_1_CODE;
    cfg_frame_.Size = 4;
    cfg_frame_.Flags = VSCAN_FLAGS_STANDARD;

    emergency_frame_.Id = CAN_EMERGENCY_ID;
    emergency_frame_.Size = 0;
    emergency_frame_.Flags = VSCAN_FLAGS_STANDARD;

    heartbeat_frame_.Id = CAN_HEARTBEAT_ID;
    heartbeat_frame_.Size = 0;
    heartbeat_frame_.Flags = VSCAN_FLAGS_STANDARD;

    calibration_frame_.Id = DRV_MODE_ID | MODE_CALIBR;
    calibration_frame_.Size = 0;
    calibration_frame_.Flags = VSCAN_FLAGS_STANDARD;

    normal_mode_frame_.Id = DRV_MODE_ID | MODE_NORMAL;
    normal_mode_frame_.Size = 0;
    normal_mode_frame_.Flags = VSCAN_FLAGS_STANDARD;


// open CAN port
// you can use VSCAN_FIRST_FOUND instead tty
    usbcan_handle_.open(VSCAN_FIRST_FOUND,mode,can_baudrate);

    ros::Duration dura(0.1);
    // dura.sleep();

    // if(usbcan_handle_.noError())
    // {
    //     new_gain = 1.1;
    //     cfg_frame_.Id = DRV_CFG_ID | CFG_PID_P | DRV_1_CODE;
    //     usbcan_handle_.wrapMsgData(cfg_frame_,new_gain);
    //     usbcan_handle_.writeRequest(&cfg_frame_,1);
    //     ROS_INFO("<< WRITE: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", cfg_frame_.Id, cfg_frame_.Data[0], cfg_frame_.Data[1], cfg_frame_.Data[2], cfg_frame_.Data[3], cfg_frame_.Data[4], cfg_frame_.Data[5], cfg_frame_.Data[6], cfg_frame_.Data[7]);
    //     // dura.sleep();

    //     new_gain = 0.01;
    //     cfg_frame_.Id = DRV_CFG_ID | CFG_PID_I | DRV_1_CODE;
    //     usbcan_handle_.wrapMsgData(cfg_frame_,new_gain);
    //     usbcan_handle_.writeRequest(&cfg_frame_,1);
    //     ROS_INFO("<< WRITE: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", cfg_frame_.Id, cfg_frame_.Data[0], cfg_frame_.Data[1], cfg_frame_.Data[2], cfg_frame_.Data[3], cfg_frame_.Data[4], cfg_frame_.Data[5], cfg_frame_.Data[6], cfg_frame_.Data[7]);
    //     // dura.sleep();

    //     new_gain = 0.8;
    //     cfg_frame_.Id = DRV_CFG_ID | CFG_PID_D | DRV_1_CODE;
    //     usbcan_handle_.wrapMsgData(cfg_frame_,new_gain);
    //     usbcan_handle_.writeRequest(&cfg_frame_,1);
    //     ROS_INFO("<< WRITE: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", cfg_frame_.Id, cfg_frame_.Data[0], cfg_frame_.Data[1], cfg_frame_.Data[2], cfg_frame_.Data[3], cfg_frame_.Data[4], cfg_frame_.Data[5], cfg_frame_.Data[6], cfg_frame_.Data[7]);
    //     usbcan_handle_.Flush();

        // usbcan_handle_.writeRequest(&emergency_frame_,1);

        // usbcan_handle_.writeRequest(&normal_mode_frame_,1);

         // float test_float = usbcan_handle_.getFloatDatafromMsg(cfg_frame_);
        // ROS_INFO("%f",test_float);
    // }

    ros::Duration dura_while(0.005); //cmd_timer_dura_
    ros::Duration dura_error(5); 

    start_time_ = ros::Time::now().toSec();
    // ros::Time::init();

     // ROS_INFO("Start time: %F",start_time_);
    
    double time_last_ = ros::Time::now().toSec();
    double max_period = 0.0;
    int16_t new_int = 0;
    while (ros::ok())
    {
        // double time_now = ros::Time::now().toSec();
        // double cycle_period = time_now - time_last_;
        // time_last_ = time_now; 
        // if(cycle_period > max_period)
        // {
        //     max_period = cycle_period;
        // }

        // cfg_frame_.Id = DRV_CFG_ID | CFG_PID_D | DRV_2_CODE;
        // usbcan_handle_.wrapMsgData(cfg_frame_,new_int);
        // usbcan_handle_.writeRequest(&cfg_frame_,1);
        // new_int++;   
        // if(new_int==12)
        // {
        //     new_int = 0;
        // }
 
        new_gain = 1.1;
        cfg_frame_.Id = 0x1;
        usbcan_handle_.wrapMsgData(cfg_frame_,(int16_t)1);
        usbcan_handle_.writeRequest(&cfg_frame_,1);
        // ROS_INFO("<< WRITE: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", cfg_frame_.Id, cfg_frame_.Data[0], cfg_frame_.Data[1], cfg_frame_.Data[2], cfg_frame_.Data[3], cfg_frame_.Data[4], cfg_frame_.Data[5], cfg_frame_.Data[6], cfg_frame_.Data[7]);
        // dura.sleep();

  
  
        new_gain = 0.01;
        cfg_frame_.Id = 0x2;
        usbcan_handle_.wrapMsgData(cfg_frame_,(int16_t)2);
        usbcan_handle_.writeRequest(&cfg_frame_,1);
  
  
  
        // ROS_INFO("<< WRITE: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", cfg_frame_.Id, cfg_frame_.Data[0], cfg_frame_.Data[1], cfg_frame_.Data[2], cfg_frame_.Data[3], cfg_frame_.Data[4], cfg_frame_.Data[5], cfg_frame_.Data[6], cfg_frame_.Data[7]);
        // dura.sleep();

        new_gain = 0.8;
        cfg_frame_.Id = 0x3;
        usbcan_handle_.wrapMsgData(cfg_frame_,(int16_t)3);
        usbcan_handle_.writeRequest(&cfg_frame_,1);
        // ROS_INFO("<< WRITE: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", cfg_frame_.Id, cfg_frame_.Data[0], cfg_frame_.Data[1], cfg_frame_.Data[2], cfg_frame_.Data[3], cfg_frame_.Data[4], cfg_frame_.Data[5], cfg_frame_.Data[6], cfg_frame_.Data[7]);
        // usbcan_handle_.Flush();

        usbcan_handle_.writeRequest(&emergency_frame_,1);
        usbcan_handle_.writeRequest(&normal_mode_frame_,1);

        dura_while.sleep();
        if (!usbcan_handle_.getErrorFlag())
        {
            // ROS_INFO("period = %f",max_period);
            // max_period = 0;
            // dura_error.sleep();
            // usbcan_handle_.Flush();
        }
        

        dura_while.sleep();
    }

    if(usbcan_handle_.writeRequest(&calibration_frame_,1)) // write request
    {       
        if(usbcan_handle_.Flush()) // if write request SUCCESS --> it means, that write frames, stored in write buffer, were successfully wrote to CAN
        {

        }
    }

    sleep(0.5);

    usbcan_handle_.close();


    return 0;
}


