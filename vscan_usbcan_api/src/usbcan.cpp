#include <vscan_usbcan_api/usbcan.h>

namespace vscan_api
{

usbcan_handle::usbcan_handle()
{

}

usbcan_handle::~usbcan_handle()
{
    close();

    // need to check if device is ready, to close it???
    // if(isReady()) 
    // {
        
    // }
}

bool usbcan_handle::open(CHAR * device, DWORD mode, void * speed)
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

void usbcan_handle::close()
{
    vscan_status_ = VSCAN_Close(vscan_handle_);
} 

bool usbcan_handle::isReady()
{
    if(vscan_status_==VSCAN_ERR_NO_DEVICE_FOUND || vscan_status_==VSCAN_ERR_INVALID_HANDLE){
        return false;
    }else{
        return true;
    }
}

char * usbcan_handle::getStatusString()
{
    VSCAN_GetErrorString(vscan_status_, error_string_, sizeof(error_string_));
    return error_string_;
}

bool  usbcan_handle::readRequest(VSCAN_MSG * read_buffer, DWORD read_buffer_size)
{
    vscan_status_ = VSCAN_Read(vscan_handle_, read_buffer, read_buffer_size, &actual_read_frame_number_);

    if(vscan_status_!=VSCAN_ERR_OK) 
    {
        return false;
    }else{
        return true;
    }
}

bool  usbcan_handle::writeRequest(VSCAN_MSG * write_buffer, DWORD write_buffer_size)
{
    vscan_status_ = VSCAN_Write(vscan_handle_, write_buffer, write_buffer_size, &actual_write_frame_number_);

    if((vscan_status_!=VSCAN_ERR_OK)) 
    {
        return false;
    }else{
        return true;
    }
}

bool  usbcan_handle::Flush()
{
    if(VSCAN_Flush(vscan_handle_)!= VSCAN_ERR_OK) 
    {
        return false;
    }else{
        return true;
    }
}


void usbcan_handle::setSpeed(void * speed)
{
    vscan_status_ = VSCAN_Ioctl(vscan_handle_, VSCAN_IOCTL_SET_SPEED, speed);
}

unsigned long usbcan_handle::getActualWriteNum(){ return actual_write_frame_number_;}
unsigned long usbcan_handle::getActualReadNum(){ return actual_read_frame_number_;}

bool usbcan_handle::noError()
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
