#include <puma01_usbcan/usbcan.h>

namespace puma01_usbcan
{

VSCAN_serial_handler::VSCAN_serial_handler(int write_size, int read_size) : write_buffer_size_(write_size), read_buffer_size_(read_size) // int --> DWORD!!!!
{
    write_buffer_.resize(write_buffer_size_);
    read_buffer_.resize(read_buffer_size_);
}

VSCAN_serial_handler::~VSCAN_serial_handler()
{
    close();
    // if(isReady()) //????????????????????????????????????????????????????????
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

bool  VSCAN_serial_handler::readRequest()
{
    vscan_status_ = VSCAN_Read(vscan_handle_, read_buffer_.data(), read_buffer_size_, &actual_read_frame_number_);

    if(vscan_status_!=VSCAN_ERR_OK) 
    {
        return false;
    }else{
        return true;
    }
}

bool  VSCAN_serial_handler::writeRequest()
{
    vscan_status_ = VSCAN_Write(vscan_handle_, write_buffer_.data(), write_buffer_size_, &actual_write_frame_number_);

    if((vscan_status_!=VSCAN_ERR_OK)) 
    {
        return false;
    }else{
        return true;
    }
}

std::vector<VSCAN_MSG> VSCAN_serial_handler::getWriteBuffer(){return write_buffer_;}
std::vector<VSCAN_MSG> VSCAN_serial_handler::getReadBuffer(){return read_buffer_;}

unsigned long VSCAN_serial_handler::getWriteBufferSize(){return write_buffer_size_;}
unsigned long VSCAN_serial_handler::getReadBufferSize(){return read_buffer_size_;}

unsigned long VSCAN_serial_handler::getActualWriteNum(){return actual_write_frame_number_;}
unsigned long VSCAN_serial_handler::getActualReadNum(){return actual_read_frame_number_;}

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

} // namespace
