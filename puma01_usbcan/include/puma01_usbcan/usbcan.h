#pragma once
#include <vector>
#include <puma01_usbcan/vs_can_api.h>

namespace puma01_usbcan
{

class VSCAN_serial_handler
{

public:

    VSCAN_serial_handler(); // constructor 
    ~VSCAN_serial_handler(); // destructor

    bool open(CHAR * device, DWORD mode, void * speed); // open port 
    void close(); // closing port 

    void setSpeed(void * speed); // set speed

    bool isReady(); // 

    char * getStatusString();
    
    bool readRequest(VSCAN_MSG * read_buffer, DWORD read_buffer_size);
    bool writeRequest(VSCAN_MSG * write_buffer, DWORD write_buffer_size);
    bool Flush();

    unsigned long getActualWriteNum();
    unsigned long getActualReadNum();

    bool noError();


// not defined functions (

    DWORD getMode(); 
    VSCAN_HANDLE getHandle();

    VSCAN_STATUS enableTimeStamp();
    VSCAN_STATUS disableTimeStamp();
    VSCAN_STATUS enableReadBlockingMode();
    VSCAN_STATUS disableReadBlockingMode();
// ) not defined functions 

private:

    // std::vector<VSCAN_MSG> write_buffer_; //????????????????????????????????????????????????????????
    // std::vector<VSCAN_MSG> read_buffer_; //????????????????????????????????????????????????????????

    // DWORD write_buffer_size_;
    // DWORD read_buffer_size_;

    VSCAN_HANDLE vscan_handle_=-1; //addres of serial port, or actual status code
    VSCAN_STATUS vscan_status_=-1; //actual status code, use getStatusString() to print it in human-readable form

    CHAR error_string_[33]; //???????????????????????????????????????????????????????? 

    DWORD actual_read_frame_number_; //actual number of read CAN-fames
    DWORD actual_write_frame_number_; //actual number of written CAN-fames

}; //class

} //namespace



/* 

TO-DO:
- check array issues
- private members??

*/