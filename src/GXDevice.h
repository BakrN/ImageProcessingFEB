
#include <camera/DxImageProc.h>
#include <camera/GxIAPI.h>
#include <vector>
#include <unordered_map>
#include <stdio.h>
// ROS includes 
#include <ros/ros.h> 

//ROSO ENDS 
static std::unordered_map<int, char*> GX_ERROR_CODE_MAP = {
    {-1,"GX_STATUS_ERROR"},          ///< There is an unspecified internal error that is not expected to occur
    {-2,"GX_STATUS_NOT_FOUND_TL"},          ///< The TL library cannot be found
    {-3,"GX_STATUS_NOT_FOUND_DEVICE"},         ///< The device is not found
    {-4,"GX_STATUS_OFFLINE"}           ,                ///< The current device is in an offline status
    {-5,"GX_STATUS_INVALID_PARAMETER"} ,              ///< Invalid parameter. Generally, the pointer is NULL or the input IP and other parameter formats are invalid
    {-6,"GX_STATUS_INVALID_HANDLE"}    ,                 ///< Invalid handle
    {-7,"GX_STATUS_INVALID_CALL"}    ,                  ///< The interface is invalid, which refers to software interface logic error
    {-8,"GX_STATUS_INVALID_ACCESS"}  ,                   ///< The function is currently inaccessible or the device access mode is incorrect
    {-9,"GX_STATUS_NEED_MORE_BUFFER"}  ,                ///< The user request buffer is insufficient: the user input buffer size during the read operation is less than the actual need
    {-10,"GX_STATUS_ERROR_TYPE"}       ,                ///< The type of FeatureID used by the user is incorrect, such as an integer interface using a floating-point function code
    {-11,"GX_STATUS_OUT_OF_RANGE"}   ,                ///< The value written by the user is crossed
    {-12,"GX_STATUS_NOT_IMPLEMENTED"}  ,               ///< This function is not currently supported
    {-13,"GX_STATUS_NOT_INIT_API"}    ,              ///< There is no call to initialize the interface
    {-14,"GX_STATUS_TIMEOUT"}                       ///< Timeout error

};
inline bool GX_CALL(GX_STATUS* status,const GX_STATUS& x, int line = 0) {
    if(x!= GX_STATUS_SUCCESS){
        // error handling 
        *status = x; 
        ROS_ERROR("FAILED in line %d with error code: %d - %s", line, *status, GX_ERROR_CODE_MAP[*status]); 
        return false; 
    }
    *status = x; 
}
struct GXDeviceAquisitionSettings{

}; 
enum AquisitionMode{

}; 
class GXDevice{
    private: 
      GX_STATUS m_Status; 
     GX_DEV_HANDLE  m_DevHandle; 
     GX_OPEN_PARAM m_OpenParams; 
     PGX_FRAME_BUFFER m_ImageBuffer; 
     bool OpenDevice(){
        GX_CALL(&m_Status, GXOpenDevice(&m_OpenParams, &m_DevHandle),__LINE__); 
    }
     bool CloseDevice(){
        GX_CALL(&m_Status, GXCloseDevice(m_DevHandle)); 
    }

  

    public: 
    // This class is assuming 1 device 
   
     GXDevice() = default; 
     ~GXDevice() = default; 
     bool Init(){
         m_ImageBuffer = new GX_FRAME_BUFFER(); 
        m_OpenParams.accessMode = GX_ACCESS_EXCLUSIVE;
        m_OpenParams.openMode = GX_OPEN_INDEX;
        m_OpenParams.pszContent = "1";
        m_DevHandle = NULL; 
       GX_CALL(&m_Status, GXInitLib(),__LINE__);  
       uint32_t count;  
       GX_CALL(&m_Status,GXUpdateDeviceList(&count, 1000),__LINE__);
       if(count <1 ){
           return false; 
           // no device connected; 
       }
       OpenDevice(); // Open Device  
        GX_CALL(&m_Status,GXSetEnum(m_DevHandle, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED),__LINE__);
    //GX_CALL(&m_Status,GXSetEnum(m_DevHandle, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS),__LINE__);
    GX_CALL(&m_Status,GXSetFloat(m_DevHandle, GX_FLOAT_GAIN, 16),__LINE__);
    GX_CALL(&m_Status,GXSetEnum(m_DevHandle, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED),__LINE__);
    GX_CALL(&m_Status,GXSetFloat(m_DevHandle, GX_FLOAT_BALANCE_RATIO, 1.4),__LINE__);
    GX_CALL(&m_Status,GXSetEnum(m_DevHandle, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE),__LINE__);
    GX_CALL(&m_Status,GXSetFloat(m_DevHandle, GX_FLOAT_BALANCE_RATIO, 1.5),__LINE__);
       return m_Status; 
    }
     bool Shutdown(){
        // 
        if(m_DevHandle){
            
        }
        delete m_ImageBuffer; 
        return GX_CALL(&m_Status, GXCloseLib()); 
    }
    

    /*
        I decided to aquire images one frame at a time, so that we're able to process an image before we receive the new one ; meaning that the program wait until a new frame is aquired

    */
    
    bool StartStream(){
        GX_CALL(&m_Status,GXStreamOn(m_DevHandle)); 
    }
    bool StopStream(){
        GX_CALL(&m_Status,GXStreamOff(m_DevHandle)); 
    }
    PGX_FRAME_BUFFER AquireImage(){
        // Calls GXQBuf to put the image buffer back into the library
                    // and continue acquiring.
                    
          GX_CALL(&m_Status, GXQBuf(m_DevHandle, m_ImageBuffer),__LINE__);
          GX_CALL(&m_Status, GXDQBuf(m_DevHandle, &m_ImageBuffer,1000),__LINE__); 
          
          return m_ImageBuffer;
    }
    
     /*bool AquireNewImage(){
        // Test speed of aquisition
        GX_CALL(&m_Status, GXSendCommand(m_DevHandle, GX_COMMAND_ACQUISITION_START)); 
        GX_CALL(&m_Status, GXSendCommand(m_DevHandle, GX_COMMAND_ACQUISITION_STOP));
        // test end 
        return true;  
    }
    
     bool SetDeviceCallback(void (*callback)(GX_FRAME_CALLBACK_PARAM* p_FrameData)){
        GX_CALL(&m_Status, GXRegisterCaptureCallback(m_DevHandle,NULL,callback)); 
        return true; 
    }
    */

    /*  GX_CALL(&m_Status,GXSetEnum(m_DevHandle, GX_ENUM_EXPOSURE_MODE, GX_EXPOSURE_MODE_TIMED));
    GX_CALL(&m_Status,GXSetEnum(m_DevHandle, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS));
    GX_CALL(&m_Status,GXSetFloat(m_DevHandle, GX_FLOAT_GAIN, 16));
    GX_CALL(&m_Status,GXSetEnum(m_DevHandle, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_RED));
    GX_CALL(&m_Status,GXSetFloat(m_DevHandle, GX_FLOAT_BALANCE_RATIO, 1.4));
    GX_CALL(&m_Status,GXSetEnum(m_DevHandle, GX_ENUM_BALANCE_RATIO_SELECTOR, GX_BALANCE_RATIO_SELECTOR_BLUE));
    GX_CALL(&m_Status,GXSetFloat(m_DevHandle, GX_FLOAT_BALANCE_RATIO, 1.5));*/
}; 