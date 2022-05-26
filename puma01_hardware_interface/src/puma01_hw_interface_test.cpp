
#include <puma01_hardware_interface/puma01_hw_interface.h>



namespace puma01_hw_interface_ns
{
puma01HWInterface::puma01HWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
 	 : 	ros_control_boilerplate::GenericHWInterface(nh, urdf_model), 
	  	name_("puma01_hw_interface"), 
		force_controller_ac_("force_controller", true),
		force_control_ac_connected_(false)
{
	// for simulation
	sim_joint_states_sub_ = nh.subscribe("/puma01_sim/joint_states",1,&puma01HWInterface::SimJointStatesCB,this);
	wrench_command_sub_ = nh.subscribe("/puma01/wrench_command",1,&puma01HWInterface::wrench_command_CB,this);
	new_pid_sub_ = nh.subscribe("/puma01/new_pids",1,&puma01HWInterface::getNewPidCB,this);
	sim_cmd_pub_ = nh.advertise<std_msgs::Float64MultiArray>("/puma01_sim/sim_computed_torque_controller/command",1);

	init();
}

// puma01HWInterface::~puma01HWInterface()
// {
// 	usbcan_handle_.close();
// }

void puma01HWInterface::init()
{
	// Call parent class version of this function
	// ros_control_boilerplate::GenericHWInterface::init();

	// OR do:
	/*
		- num_joints <- joint_names.size()
		- resize telemetry vectors of variables (pos, vel, maybe eff)
		- resize vectors of commands (pos, vel, acc)
		- resize vectors of joint limits
		- for joint id do: ...
			- JointStateInterface: register JointStateHandle
			- joint handle: define PosVelAccJointHandle
			- PosVelAccJointInterface: register PosVelAccJointHandle
			- register joint limits
		- register joint interfaces (JointStateInterface and PosVelAccJointInterface)
	*/

	initCAN();

	num_joints_ = joint_names_.size();  // std::size_t

	initJointInterfaces(); // call it after num_joints_ has been defined!

	wrench_command_.force.x = 0.0;
	wrench_command_.force.y = 0.0;
	wrench_command_.force.z = 0.0;
	wrench_command_.torque.x = 0.0;
	wrench_command_.torque.y = 0.0;
	wrench_command_.torque.z = 0.0;

	force_control_position_corr_.fill(0.0);

	p_gains_.fill(0.0);
	i_gains_.fill(0.0);
	d_gains_.fill(0.0);

	int_pos_cmds_.fill(0.0);
	int_vel_cmds_.fill(0.0);

	double as_wait_timeout = 5.0;

	ROS_INFO_NAMED(name_, "Waiting for force controller action server to be started in %d secs...",(int)as_wait_timeout);
	force_controller_ac_.waitForServer(ros::Duration(as_wait_timeout)); // waiting for force controller action server to start
	if(force_controller_ac_.isServerConnected())
	{
		ROS_INFO_NAMED(name_, "Connection with force controller established.");
		force_control_ac_connected_ = true;
	}else{
		ROS_ERROR_NAMED(name_, "No force_controller with action interface found! Ignoring.");
	}

}

void puma01HWInterface::wrench_command_CB(const geometry_msgs::Wrench& wrench)
{ 
	// define Goal for force controller
	wrench_command_.torque.x = wrench.torque.x;
	wrench_command_.torque.y = wrench.torque.y;
	wrench_command_.torque.z = wrench.torque.z;
	wrench_command_.force.x = wrench.force.x;
	wrench_command_.force.y = wrench.force.y;
	wrench_command_.force.z = wrench.force.z;

	// ROS_INFO_NAMED(name_, "Got new wrench command: fz = %4.4f",wrench_command_.force.z);
	
}

void puma01HWInterface::read(ros::Duration& elapsed_time)
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
					uint32_t joint_n = 0;

					switch (read_msg.Id)
					{
					// case DRV_STATE_ID | DRV_1_CODE | MOTOR_POT_ENC_CUR:
					// 	break;

					case DRV_STATE_ID | DRV_2_CODE | MOTOR_POT_ENC_CUR:
						joint_n = 1;
						break;

					case DRV_STATE_ID | DRV_3_CODE | MOTOR_POT_ENC_CUR:
						joint_n = 2;
						break;			

					default:
						break;
					}

					// ROS_INFO("Got CAN-frame with ID: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", read_msg.Id, read_msg.Data[0], read_msg.Data[1], read_msg.Data[2], read_msg.Data[3], read_msg.Data[4], read_msg.Data[5], read_msg.Data[6], read_msg.Data[7]);

					joint_position_[joint_n] = (double)usbcan_handle_.getDatafromMsg(read_msg,2) * enc_to_joint_consts_[joint_n];	
					joint_velocity_[joint_n] = (double)usbcan_handle_.getDatafromMsg(read_msg,6) * enc_to_joint_consts_[joint_n];	

				}
			}
		}
	}

}

void puma01HWInterface::write(ros::Duration& elapsed_time)
{
	// Safety !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	// enforceLimits(elapsed_time);	

	static bool new_cmd_arrived = false;

	for(std::size_t i=0; i<6; i++)
	{
		if(i<3)
		{
			int_traj_cmds_[i] = joint_position_command_[i]/enc_to_joint_consts_[i];
			int_traj_cmds_[i+num_joints_] = joint_velocity_command_[i]/enc_to_joint_consts_[i];

			if(int_pos_cmds_[i] != int_traj_cmds_[i])
			{
				int_pos_cmds_[i] = int_traj_cmds_[i];
				usbcan_handle_.wrapMsgData(pos_cmd_vscan_msg_, int_pos_cmds_[i], i*2);
				new_cmd_arrived = true;
			}
			if(int_vel_cmds_[i] != int_traj_cmds_[i+num_joints_])
			{
				int_vel_cmds_[i] = int_traj_cmds_[i+num_joints_];
				// usbcan_handle_.wrapMsgData(vel_cmd_vscan_msg_, int_vel_cmds_[i], i*2);
				new_cmd_arrived = true;
			}

			// traj_cmd_.data[i] = joint_position_command_[i];
			// traj_cmd_.data[i+num_joints_] = joint_velocity_command_[i];
			// traj_cmd_.data[i+traj_cmd_acc_offset_] = joint_acceleration_command_[i];

		}
		else
		{
			joint_position_[i] = joint_position_command_[i];
			// joint_velocity_[i] = joint_velocity_command_[i];
		}

	}

	if(new_cmd_arrived)
	{
		if(usbcan_handle_.writeRequest(&pos_cmd_vscan_msg_,1))
		{
			usbcan_handle_.Flush();
		}
		// usbcan_handle_.writeRequest(&vel_cmd_vscan_msg_,1);
		ROS_INFO("Write new command to CAN: joint_2: pos_cmd = %i		joint_3: pos_cmd = %i",int_pos_cmds_[1],int_pos_cmds_[2]);
		new_cmd_arrived = false;
	}

	if(usbcan_handle_.writeRequest(&heartbeat_frame_,1))
	{
		usbcan_handle_.Flush();
	}

}

void puma01HWInterface::enforceLimits(ros::Duration& period)
{
	// Enforces position and velocity
	// pos_jnt_sat_interface_.enforceLimits(period);
}

void puma01HWInterface::initCAN()
{
    DWORD mode = VSCAN_MODE_NORMAL;
    void * can_baudrate = VSCAN_SPEED_500K;
    usbcan_handle_.open(VSCAN_FIRST_FOUND,mode,can_baudrate);

	drv_code_ids_ = {DRV_1_CODE, DRV_2_CODE, DRV_3_CODE};

	enc_to_joint_consts_ = {MOTOR_1_ENC_TO_JOINT_CONST, 
							MOTOR_2_ENC_TO_JOINT_CONST, 
							MOTOR_3_ENC_TO_JOINT_CONST};

	drv_feedback_ids_ ={DRV_STATE_ID | DRV_1_CODE | MOTOR_POT_ENC_CUR, 
						DRV_STATE_ID | DRV_2_CODE | MOTOR_POT_ENC_CUR, 
						DRV_STATE_ID | DRV_3_CODE | MOTOR_POT_ENC_CUR};

	pos_cmd_vscan_msg_.Id = TRAJ_CMD_ID | DRV_123_CODE | MOTOR_POS;
    pos_cmd_vscan_msg_.Size = 6;
    pos_cmd_vscan_msg_.Flags = VSCAN_FLAGS_STANDARD;

	vel_cmd_vscan_msg_.Id = TRAJ_CMD_ID | DRV_123_CODE | MOTOR_VEL;
    vel_cmd_vscan_msg_.Size = 6;
    vel_cmd_vscan_msg_.Flags = VSCAN_FLAGS_STANDARD;

	for(std::size_t i=0; i < 3; i++)
	{
		usbcan_handle_.wrapMsgData(pos_cmd_vscan_msg_, int_pos_cmds_[i], i*2);
		usbcan_handle_.wrapMsgData(vel_cmd_vscan_msg_, int_vel_cmds_[i], i*2);

		new_pid_p_vscan_msgs_[i].Id = DRV_CFG_ID | CFG_PID_P | drv_code_ids_[i];
		new_pid_p_vscan_msgs_[i].Size = 4;
		new_pid_p_vscan_msgs_[i].Flags = VSCAN_FLAGS_STANDARD;
		usbcan_handle_.wrapMsgData(new_pid_p_vscan_msgs_[i], 0.0F);

		new_pid_i_vscan_msgs_[i].Id = DRV_CFG_ID | CFG_PID_I | drv_code_ids_[i];
		new_pid_i_vscan_msgs_[i].Size = 4;
		new_pid_i_vscan_msgs_[i].Flags = VSCAN_FLAGS_STANDARD;
		usbcan_handle_.wrapMsgData(new_pid_i_vscan_msgs_[i], 0.0F);

		new_pid_d_vscan_msgs_[i].Id = DRV_CFG_ID | CFG_PID_D | drv_code_ids_[i];
		new_pid_d_vscan_msgs_[i].Size = 4;
		new_pid_d_vscan_msgs_[i].Flags = VSCAN_FLAGS_STANDARD;
		usbcan_handle_.wrapMsgData(new_pid_d_vscan_msgs_[i], 0.0F);
	}

	heartbeat_frame_.Id = CAN_HEARTBEAT_ID;
    heartbeat_frame_.Size = 0;
    heartbeat_frame_.Flags = VSCAN_FLAGS_STANDARD;

	read_buff_size_ = 20;
	read_buffer_.resize(read_buff_size_);

	ROS_INFO("CAN interface initialized!");

}

void puma01HWInterface::initJointInterfaces()
{
	// resize sim cmd vector
	traj_cmd_full_size_ = num_joints_*4;
	traj_cmd_acc_offset_ = num_joints_*2;
	traj_cmd_.data.resize(traj_cmd_full_size_, 0.0);

	// Status
	joint_position_.resize(num_joints_, 0.0);
	joint_velocity_.resize(num_joints_, 0.0);
	joint_effort_.resize(num_joints_, 0.0); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	// Command
	joint_position_command_.resize(num_joints_, 0.0);
	joint_velocity_command_.resize(num_joints_, 0.0);
	joint_acceleration_command_.resize(num_joints_, 0.0);
	joint_effort_command_.resize(num_joints_, 0.0); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	// Limits
	joint_position_lower_limits_.resize(num_joints_, 0.0);
	joint_position_upper_limits_.resize(num_joints_, 0.0);
	joint_velocity_limits_.resize(num_joints_, 0.0);
	joint_effort_limits_.resize(num_joints_, 0.0);

	// Initialize interfaces for each joint
	for (std::size_t joint_id = 0; joint_id < num_joints_; ++joint_id)
	{
		ROS_DEBUG_STREAM_NAMED(name_, "Loading joint name: " << joint_names_[joint_id]);

		// Create joint state interface
		joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(
			joint_names_[joint_id], &joint_position_[joint_id], &joint_velocity_[joint_id], &joint_effort_[joint_id]));

		// Add command interfaces to joints
		hardware_interface::PosVelAccJointHandle joint_handle_posvelacc = hardware_interface::PosVelAccJointHandle(
			joint_state_interface_.getHandle(joint_names_[joint_id]), 
			&joint_position_command_[joint_id],
			&joint_velocity_command_[joint_id],
			&joint_acceleration_command_[joint_id]);

		posvelacc_joint_interface_.registerHandle(joint_handle_posvelacc);

		// Load the joint limits !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		// registerJointLimits(joint_handle_position, joint_handle_velocity, joint_handle_effort, joint_id); 

	}  // end for each joint

	registerInterface(&joint_state_interface_);     // From RobotHW base class.
	registerInterface(&posvelacc_joint_interface_);    // From RobotHW base class.

}

void puma01HWInterface::SimJointStatesCB(const sensor_msgs::JointState::ConstPtr& msg)
{
	for(std::size_t i=0; i<num_joints_; i++)
	{
		joint_position_[i] = msg->position[i];	
		joint_velocity_[i] = msg->velocity[i];				
	}
}

void puma01HWInterface::getNewPidCB(const control_msgs::JointControllerStateConstPtr& new_pids_msg)
{
	uint32_t joint_n = new_pids_msg->header.seq;

	if(joint_n < 3)
	{
		if(p_gains_[joint_n] != new_pids_msg->p) 
		{
			p_gains_[joint_n] = new_pids_msg->p;

			usbcan_handle_.wrapMsgData(new_pid_p_vscan_msgs_[joint_n],p_gains_[joint_n]);

			if(usbcan_handle_.writeRequest(&new_pid_p_vscan_msgs_[joint_n],1))
			{
				usbcan_handle_.Flush();
			}
		
			ROS_INFO("Got new P-gain for joint_%i",joint_n+1);
			ROS_INFO("Write config CAN-frame with ID: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", new_pid_p_vscan_msgs_[joint_n].Id, new_pid_p_vscan_msgs_[joint_n].Data[0], new_pid_p_vscan_msgs_[joint_n].Data[1], new_pid_p_vscan_msgs_[joint_n].Data[2], new_pid_p_vscan_msgs_[joint_n].Data[3], new_pid_p_vscan_msgs_[joint_n].Data[4], new_pid_p_vscan_msgs_[joint_n].Data[5], new_pid_p_vscan_msgs_[joint_n].Data[6], new_pid_p_vscan_msgs_[joint_n].Data[7]);
		}

		if(i_gains_[joint_n] != new_pids_msg->i) 
		{
			i_gains_[joint_n] = new_pids_msg->i;

			usbcan_handle_.wrapMsgData(new_pid_i_vscan_msgs_[joint_n],i_gains_[joint_n]);

			usbcan_handle_.writeRequest(&new_pid_i_vscan_msgs_[joint_n],1);

			if(usbcan_handle_.writeRequest(&new_pid_i_vscan_msgs_[joint_n],1))
			{
				usbcan_handle_.Flush();
			}

			ROS_INFO("Got new I-gain for joint_%i",joint_n+1);
			ROS_INFO("Write config CAN-frame with ID: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", new_pid_i_vscan_msgs_[joint_n].Id, new_pid_i_vscan_msgs_[joint_n].Data[0], new_pid_i_vscan_msgs_[joint_n].Data[1], new_pid_i_vscan_msgs_[joint_n].Data[2], new_pid_i_vscan_msgs_[joint_n].Data[3], new_pid_i_vscan_msgs_[joint_n].Data[4], new_pid_i_vscan_msgs_[joint_n].Data[5], new_pid_i_vscan_msgs_[joint_n].Data[6], new_pid_i_vscan_msgs_[joint_n].Data[7]);
		}
			
		if(d_gains_[joint_n] != new_pids_msg->d) 
		{
			d_gains_[joint_n] = new_pids_msg->d;

			usbcan_handle_.wrapMsgData(new_pid_d_vscan_msgs_[joint_n],d_gains_[joint_n]);

			usbcan_handle_.writeRequest(&new_pid_d_vscan_msgs_[joint_n],1);

			if(usbcan_handle_.writeRequest(&new_pid_d_vscan_msgs_[joint_n],1))
			{
				usbcan_handle_.Flush();
			}

			ROS_INFO("Got new D-gain for joint_%i",joint_n+1);
			ROS_INFO("Write config CAN-frame with ID: %03x, Data: %02x %02x %02x %02x %02x %02x %02x %02x", new_pid_d_vscan_msgs_[joint_n].Id, new_pid_d_vscan_msgs_[joint_n].Data[0], new_pid_d_vscan_msgs_[joint_n].Data[1], new_pid_d_vscan_msgs_[joint_n].Data[2], new_pid_d_vscan_msgs_[joint_n].Data[3], new_pid_d_vscan_msgs_[joint_n].Data[4], new_pid_d_vscan_msgs_[joint_n].Data[5], new_pid_d_vscan_msgs_[joint_n].Data[6], new_pid_d_vscan_msgs_[joint_n].Data[7]);
		}
		
	}
	else
	{
		ROS_WARN("getNewPidCB: Invalid joint number (%i >= 3)!",joint_n);
	}
}

void puma01HWInterface::force_controller_ac_DoneCB(const actionlib::SimpleClientGoalState &state, const puma01_force::ForceControlResultConstPtr &result)
{
	for(std::size_t i=0; i<num_joints_; i++)
	{
		// joint_effort_command_[i] = result->output_torques.data[i];	
		force_control_position_corr_[i] = result->output_torques.data[i];		
		// joint_position_command_[i] -= result->output_torques.data[i];	
	}		
}

}  // namespace 
