#pragma once
#include <vector>
#include <ros/ros.h>
#include <vscan_usbcan_api/vs_can_api.h>
#include <vscan_usbcan_api/can_frames.h>

namespace vscan_api
{

class usbcan_handle
{

public:

    usbcan_handle(); // constructor 
    ~usbcan_handle(); // destructor

    // open port of chosen device with set baudrate
    bool open(CHAR * device, DWORD mode, void * speed); 

    void close(); // closing port 

    // setting CAN baudrate
    bool setSpeed(void * speed); // set speed

    char * getStatusString();
    
    bool readRequest(VSCAN_MSG * read_buffer, DWORD read_buffer_size);
    bool writeRequest(VSCAN_MSG * write_buffer, DWORD write_buffer_size);
    bool Flush();

    // get the actual number of CAN-messages write to CAN after last write request
    unsigned long getActualWriteNum();

    // get the actual number of CAN-messages got from buffer after last read request
    unsigned long getActualReadNum();

    // check if there's no error got from VSCAN USB-CAN driver
    bool noError();

    // insert given 'uint32_t' number in Data field of given VSCAN_MSG structure with given byte offset 
    void wrapMsgData(VSCAN_MSG &msg, uint32_t &val, int byte_offset = 0);

    // get 'uint32_t' number from Data field of given VSCAN_MSG structure
    uint32_t getDatafromMsg(VSCAN_MSG &msg, int byte_offset = 0);


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