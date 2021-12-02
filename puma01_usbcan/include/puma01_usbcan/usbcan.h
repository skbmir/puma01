#pragma once
#include <vector>
#include <puma01_usbcan/vs_can_api.h>

namespace puma01_usbcan
{

class VSCAN_serial_handler
{

public:

    VSCAN_serial_handler(int write_size, int read_size); // constructor with args to specify buffers sizes
    ~VSCAN_serial_handler(); // destructor

    bool open(CHAR * device, DWORD mode, void * speed); // open port 
    void close(); // closing port 

    void setSpeed(void * speed); // set speed

    bool isReady(); // 

    char * getStatusString();
    
    bool readRequest();
    bool writeRequest();
    bool Flush();

    std::vector<VSCAN_MSG> getWriteBuffer();
    std::vector<VSCAN_MSG> getReadBuffer();

    unsigned long getWriteBufferSize();
    unsigned long getReadBufferSize();

    unsigned long getActualWriteNum();
    unsigned long getActualReadNum();

    VSCAN_STATUS getStatus();

    bool noError();


// not defined functions (
    bool pushFrametoBuffer(UINT32 Id, UINT8 Size, UINT8 * Data, UINT8 Flags=VSCAN_FLAGS_STANDARD); 

    DWORD getMode(); 
    VSCAN_HANDLE getHandle();

    VSCAN_STATUS enableTimeStamp();
    VSCAN_STATUS disableTimeStamp();
    VSCAN_STATUS enableReadBlockingMode();
    VSCAN_STATUS disableReadBlockingMode();
// ) not defined functions 

private:
    std::vector<VSCAN_MSG> read_buffer_; //????????????????????????????????????????????????????????
    std::vector<VSCAN_MSG> write_buffer_; //????????????????????????????????????????????????????????

    DWORD write_buffer_size_;
    DWORD read_buffer_size_;

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