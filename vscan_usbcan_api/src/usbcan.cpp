#include <vscan_usbcan_api/usbcan.h>

namespace vscan_api
{

usbcan_handle::usbcan_handle()
{

}

usbcan_handle::~usbcan_handle()
{
    // need to check if device is ready, to close it???
    // if(isReady()) 
    // {
        
    // }
}

bool usbcan_handle::open(CHAR * device, DWORD mode, void * speed)
{
    vscan_handle_ = VSCAN_Open(device, mode);   
    vscan_status_ = vscan_handle_;
    if(vscan_handle_>0)
    {
        ROS_INFO("[USB-CAN adapter] Successfuly connected on port %s",device);

        if(mode==VSCAN_MODE_NORMAL){ROS_INFO("[USB-CAN adapter] Mode: normal");}
        else if(mode==VSCAN_MODE_LISTEN_ONLY){ROS_INFO("[USB-CAN adapter] Mode: listen only");}
        else if(mode==VSCAN_MODE_SELF_RECEPTION){ROS_INFO("[USB-CAN adapter] Mode: self-reception");}

        setSpeed(speed);
        return true;
    }else{
        ROS_ERROR("[USB-CAN adapter] Failed to connect.");
        return false;
    }
}

void usbcan_handle::close()
{
    ROS_INFO("[USB-CAN adapter] Shutting down connection...");
    vscan_status_ = VSCAN_Close(vscan_handle_);
} 

char * usbcan_handle::getStatusString()
{
    VSCAN_GetErrorString(vscan_status_, error_string_, sizeof(error_string_));
    return error_string_;
}

bool  usbcan_handle::readRequest(VSCAN_MSG * read_buffer, DWORD read_buffer_size)
{
    vscan_status_ = VSCAN_Read(vscan_handle_, read_buffer, read_buffer_size, &actual_read_frame_number_);
    return noError();
}

bool  usbcan_handle::writeRequest(VSCAN_MSG * write_buffer, DWORD write_buffer_size)
{
    vscan_status_ = VSCAN_Write(vscan_handle_, write_buffer, write_buffer_size, &actual_write_frame_number_);
    return noError();
}

bool  usbcan_handle::Flush()
{
    vscan_status_ = VSCAN_Flush(vscan_handle_);
    return noError();
}


bool usbcan_handle::setSpeed(void * speed)
{
    vscan_status_ = VSCAN_Ioctl(vscan_handle_, VSCAN_IOCTL_SET_SPEED, speed);
    if(noError())
    {
        int speed_val = 0;
        if(speed==VSCAN_SPEED_100K){speed_val = 100000;}
        else if(speed==VSCAN_SPEED_125K){speed_val = 125000;}
        else if(speed==VSCAN_SPEED_1M){speed_val = 1000000;}
        else if(speed==VSCAN_SPEED_20K){speed_val = 20000;}
        else if(speed==VSCAN_SPEED_250K){speed_val = 250000;}
        else if(speed==VSCAN_SPEED_500K){speed_val = 500000;}
        else if(speed==VSCAN_SPEED_50K){speed_val = 50000;}
        else if(speed==VSCAN_SPEED_800K){speed_val = 800000;}
        ROS_INFO("[USB-CAN adapter] CAN baudrate set: %i",speed_val);
        return true;
    }else{
        return false;
    }
}


bool usbcan_handle::getErrorFlag()
{
    vscan_status_ = VSCAN_Ioctl(vscan_handle_, VSCAN_IOCTL_GET_FLAGS, &vscan_flags_);
    if(vscan_flags_!=0)
    {
        ROS_WARN("[USB-CAN adapter] Error flags: %lx",vscan_flags_);

        if(vscan_flags_&VSCAN_IOCTL_FLAG_RX_FIFO_FULL != 0)
        {
            ROS_INFO("[USB-CAN adapter]     RX FIFO FULL: %li",vscan_flags_&VSCAN_IOCTL_FLAG_RX_FIFO_FULL);
        }

        if((vscan_flags_ >> 1)&(VSCAN_IOCTL_FLAG_TX_FIFO_FULL >> 1))
        {
            ROS_INFO("[USB-CAN adapter]     TX FIFO full: %li",(vscan_flags_ >> 1)&(VSCAN_IOCTL_FLAG_TX_FIFO_FULL >> 1));
        }

        if((vscan_flags_ >> 2)&(VSCAN_IOCTL_FLAG_ERR_WARNING >> 2) !=0)
        {
            ROS_INFO("[USB-CAN adapter]      ERR WARNING: %li",(vscan_flags_ >> 2)&(VSCAN_IOCTL_FLAG_ERR_WARNING >> 2));
        }

        if((vscan_flags_ >> 3)&(VSCAN_IOCTL_FLAG_DATA_OVERRUN >> 3) != 0)
        {
            ROS_INFO("[USB-CAN adapter]     DATA OVERRUN: %li",(vscan_flags_ >> 3)&(VSCAN_IOCTL_FLAG_DATA_OVERRUN >> 3));
        }

        if((vscan_flags_ >> 4)&(VSCAN_IOCTL_FLAG_UNUSED >> 4) != 0)
        {
            ROS_INFO("[USB-CAN adapter]           UNUSED: %li",(vscan_flags_ >> 4)&(VSCAN_IOCTL_FLAG_UNUSED >> 4));
        }

        if((vscan_flags_ >> 5)&(VSCAN_IOCTL_FLAG_ERR_PASSIVE >> 5) != 0)
        {
            ROS_INFO("[USB-CAN adapter]      ERR PASSIVE: %li",(vscan_flags_ >> 5)&(VSCAN_IOCTL_FLAG_ERR_PASSIVE >> 5));
        }

        if((vscan_flags_ >> 6)&(VSCAN_IOCTL_FLAG_ARBIT_LOST >> 6) != 0)
        {
            ROS_INFO("[USB-CAN adapter]       ARBIT LOST: %li",(vscan_flags_ >> 6)&(VSCAN_IOCTL_FLAG_ARBIT_LOST >> 6));
        }

        if((vscan_flags_ >> 7)&(VSCAN_IOCTL_FLAG_BUS_ERROR >> 7) != 0)
        {
            ROS_INFO("[USB-CAN adapter]        BUS ERROR: %li",(vscan_flags_ >> 7)&(VSCAN_IOCTL_FLAG_BUS_ERROR >> 7));
        }

        if((vscan_flags_ >> 16)&(VSCAN_IOCTL_FLAG_API_RX_FIFO_FULL >> 16) != 0)
        {
            ROS_INFO("[USB-CAN adapter] API RX FIFO FULL: %li",(vscan_flags_ >> 16)&(VSCAN_IOCTL_FLAG_API_RX_FIFO_FULL >> 16));
        }

        return false;
    }
    return true;
}

unsigned long usbcan_handle::getActualWriteNum(){ return actual_write_frame_number_;}
unsigned long usbcan_handle::getActualReadNum(){ return actual_read_frame_number_;}

bool usbcan_handle::noError()
{ 
    if(vscan_status_ == VSCAN_ERR_OK)
    {
        return true;
    }else{
        ROS_ERROR("[USB-CAN adapter] Error: %s",getStatusString());
        return false;
    }
}

void usbcan_handle::wrapMsgData(VSCAN_MSG &msg, int16_t val, size_t byte_offset)
{
    msg.Data[byte_offset]   = val >> 8;
    msg.Data[byte_offset+1] = val;
}

int16_t usbcan_handle::getDatafromMsg(VSCAN_MSG &msg, size_t byte_offset)
{
    return msg.Data[byte_offset]<<8 | msg.Data[byte_offset+1];
}

void usbcan_handle::wrapMsgData(VSCAN_MSG &msg, float val, size_t byte_offset)
{
    unsigned char * data_addr = (unsigned char *)&val; // float ptr --> uchar ptr
    msg.Data[byte_offset] = *data_addr; 
    msg.Data[byte_offset+1] = *++data_addr;
    msg.Data[byte_offset+2] = *++data_addr;
    msg.Data[byte_offset+3] = *++data_addr;
    
}

float usbcan_handle::getFloatDatafromMsg(VSCAN_MSG &msg, size_t byte_offset)
{
    unsigned char data_field[] = {msg.Data[0], msg.Data[1], msg.Data[2], msg.Data[3]};
    float val_out = *(float *)&data_field;
    return val_out;
}


} // namespace
