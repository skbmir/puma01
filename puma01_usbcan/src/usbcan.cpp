#include <puma01_usbcan/usbcan.h>

namespace puma01_usbcan
{

VSCAN_serial_handler::VSCAN_serial_handler()
{

}

VSCAN_serial_handler::~VSCAN_serial_handler()
{
    close();

    // need to check if device is ready, to close it???
    // if(isReady()) 
    // {
        
    // }
}

bool VSCAN_serial_handler::open(CHAR * device, DWORD mode, void * speed)
{
    vscan_handle_ = VSCAN_Open(device, mode);   
    if(vscan_handle_>0)
    {
        setSpeed(speed);
        return true;
    }else{
        vscan_status_ = vscan_handle_;
        return false;
    }
}

void VSCAN_serial_handler::close()
{
    vscan_status_ = VSCAN_Close(vscan_handle_);
} 

bool VSCAN_serial_handler::isReady()
{
    if(vscan_status_==VSCAN_ERR_NO_DEVICE_FOUND || vscan_status_==VSCAN_ERR_INVALID_HANDLE){
        return false;
    }else{
        return true;
    }
}

char * VSCAN_serial_handler::getStatusString()
{
    VSCAN_GetErrorString(vscan_status_, error_string_, sizeof(error_string_));
    return error_string_;
}

bool  VSCAN_serial_handler::readRequest(VSCAN_MSG * read_buffer, DWORD read_buffer_size)
{
    vscan_status_ = VSCAN_Read(vscan_handle_, read_buffer, read_buffer_size, &actual_read_frame_number_);

    if(vscan_status_!=VSCAN_ERR_OK) 
    {
        return false;
    }else{
        return true;
    }
}

bool  VSCAN_serial_handler::writeRequest(VSCAN_MSG * write_buffer, DWORD write_buffer_size)
{
    vscan_status_ = VSCAN_Write(vscan_handle_, write_buffer, write_buffer_size, &actual_write_frame_number_);

    if((vscan_status_!=VSCAN_ERR_OK)) 
    {
        return false;
    }else{
        return true;
    }
}

bool  VSCAN_serial_handler::Flush()
{
    if(VSCAN_Flush(vscan_handle_)!= VSCAN_ERR_OK) 
    {
        return false;
    }else{
        return true;
    }
}


void VSCAN_serial_handler::setSpeed(void * speed)
{
    vscan_status_ = VSCAN_Ioctl(vscan_handle_, VSCAN_IOCTL_SET_SPEED, speed);
}

unsigned long VSCAN_serial_handler::getActualWriteNum(){ return actual_write_frame_number_;}
unsigned long VSCAN_serial_handler::getActualReadNum(){ return actual_read_frame_number_;}

bool VSCAN_serial_handler::noError()
{ 
    if(vscan_status_ == VSCAN_ERR_OK)
    {
        return true;
    }else{
        return false;
    }
}

bool pushFrametoBuffer(UINT32 Id, UINT8 Size, UINT8 * Data, UINT8 Flags=VSCAN_FLAGS_STANDARD)
{
    return true;
}


} // namespace
