#ifndef EMM42_V5_0_DRIVER_HPP_
#define EMM42_V5_0_DRIVER_HPP_
#include "HardwareSerial.h"

#define MAX_RETRY 3 //最大重试次数

enum Emm42_CMDstatus{
  Emm42_OK,      //命令成功 
  Emm42_False,   //命令条件不满足
  Emm42_Error,    //错误命令
  Emm42_SerialError //串口未初始化
};

typedef enum {
    S_VER   = 0,      /* 读取固件版本和对应的硬件版本 */
    S_RL    = 1,      /* 读取读取相电阻和相电感 */
    S_PID   = 2,      /* 读取PID参数 */
    S_VBUS  = 3,      /* 读取总线电压 */
    S_CPHA  = 5,      /* 读取相电流 */
    S_ENCL  = 7,      /* 读取经过线性化校准后的编码器值 */
    S_TPOS  = 8,      /* 读取电机目标位置角度 */
    S_VEL   = 9,      /* 读取电机实时转速 */
    S_CPOS  = 10,     /* 读取电机实时位置角度 */
    S_PERR  = 11,     /* 读取电机位置误差角度 */
    S_FLAG  = 13,     /* 读取使能/到位/堵转状态标志位 */
    S_Conf  = 14,     /* 读取驱动参数 */
    S_State = 15,     /* 读取系统状态参数 */
    S_ORG   = 16,     /* 读取正在回零/回零失败状态标志位 */
}SysParams_t;


//Emm42步进驱动类
class Emm42_V5_0_Driver {
public:

    /**
    * @brief    默认构造函数
    * @param    void
    * @retval   void
   */
    Emm42_V5_0_Driver(){
        id = 1;
        this->serial = &Serial;
        check_Byte = 0x6B;
        is_serialEnable = false;
        is_enable = false;
        bool block_protect = true;
    }

    /**
    * @brief    默认构造函数
    * @param    Serial：设备绑定的串口
    * @param    ID：设备id
    * @param    Check_Byte：校验字节
    * @param    Block_protect：堵转保护(默认开启)
    * @retval   void
   */
    Emm42_V5_0_Driver(HardwareSerial *Serial,uint8_t ID,uint8_t Check_Byte = 0x6B,bool Block_protect = true){
        serial = Serial;
        id = ID;
        check_Byte = Check_Byte;
        is_serialEnable = false;
        is_enable = false;
        block_protect = Block_protect;
    }

    /**
    * @brief    串口初始化
    * @param    void
    * @retval   void
    */
    void serial_enable(){
        this->serial->begin(115200); //设置串口默认速率
        this->is_serialEnable = true;
    }

     /**
    * @brief    电机使能
    * @param    void
    * @retval   Emm42_CMDstatus：命令执行状态
    * @description: 发送使能指令，并接收电机返回值，返回值可能是成功，条件不满足，错误命令，串口未初始化。
    */
    Emm42_CMDstatus enable(){
        if(is_serialEnable == true){
        uint8_t len = 6;
        uint8_t cmd[6] = {id,0xF3,0xAB,0x01,0x00,check_Byte};
        
        //发送使能指令
        send(cmd,len); 
        //接受电机返回值
        read(cmd,4);
        switch (cmd[2])
        {
        case 0x02:
            is_enable = true;
            return Emm42_OK;
            break;
        case 0xE2:
            is_enable = false;
            return Emm42_False;
            break;
        case 0xEE:
            is_enable = false;
            return Emm42_Error;
            break;
        }
        is_enable = false;
        return Emm42_Error;
       }
       is_enable = false;
       return Emm42_SerialError ;   
    }

    
    /**
    * @brief    电机同步
    * @param    void
    * @retval   Emm42_CMDstatus：命令执行状态
    * @description: 发送使能指令，并接收电机返回值，返回值可能是成功，条件不满足，错误命令，串口未初始化。
    */
    void sync_enable(){
        
    }

private:
    HardwareSerial*serial;//设备绑定的串口
    uint8_t id;           //设备id
    uint8_t check_Byte;   //校验字节
    bool is_serialEnable;        //串口是否初始化
    bool is_enable;       //电机是否使能
    bool block_protect;   //堵转保护(默认开启)

    /**
    * @brief    发送数据
    * @param    data：数据数组
    * @param    len：数据长度
    * @retval   空
   */
    void send(uint8_t*data,uint8_t len){
        if(serial->available()){
            serial->flush();
        }
        for(int i = 0; i<len;++i){
            serial->write(data[i]);
        }
        delay(1);
    }
     /*
    * @brief    读取串口数据
    * @param    data:存放的数组
    * @param    len: 数据长度
    * @param    need_delay：是否需要延时
    * @retval   空
   */
    void read(uint8_t *data,uint8_t len,bool need_delay = true){
        if(need_delay)
            delay(1);
        for (int i = 0; i < MAX_RETRY; i++){
            if (this->serial->available()){
                data[0]=this->serial->read();
                if (data[0] == this->id){
                    for (int j = 1; j < len; j++){
                        data[j]=this->serial->read();
                    }
                    return ;
                }
            }
            delay(1);
        }
    }
     
    /*
    * @brief    读取系统数据参数
    * @param    param：想要读取什么参数
    * @param    need_delay：是否需要延时

   */
    /*
    void read_(SysParams_t param,bool need_delay = true){
        if(need_delay){
            delay(1);
        };
        uint8_t i = 0;
        uint8_t cmd[16] = {0};
        //装载id 
        cmd[i++] = this->id;
        //装载对应的指令
        switch(param)
        {
            case S_VER:  cmd[i++] = 0x1F;break; //读取固件版本和对应的硬件版本
            case S_RL:   cmd[i++] = 0x20;break; //读取相电阻和相电感
            case S_PID:  cmd[i++] = 0x21;break; //读取位置环PID参数
            case S_VBUS: cmd[i++] = 0x24;break; //读取总线电压
            case S_CPHA: cmd[i++] = 0x27;break; //读取相电流
            case S_ENCL: cmd[i++] = 0x31;break; //读取经过线性化校准后的编码器值
            case S_TPOS: cmd[i++] = 0x33;break; //读取电机目标位置
            case S_VEL:  cmd[i++] = 0x35;break; //读取电机实时转速
            case S_CPOS: cmd[i++] = 0x36;break; //读取电机实时位置
            case S_PERR: cmd[i++] = 0x37;break; //读取电机位置误差
            case S_FLAG: cmd[i++] = 0x3A;break; //读取电机状态标志位
            case S_ORG:  cmd[i++] = 0x3B;break; //读取回零状态标志位
            case S_Conf: cmd[i++] = 0x42;cmd[i++] = 0x6C;break; //读取驱动配置参数
            case S_State:cmd[i++] = 0x43;cmd[i++] = 0x7A; break;
            default: break;
        }
        cmd[i++] = this->check_Byte;  //校验字节

        //发送指令
        Serial.write(cmd,i);
    }
    */
};


#endif /* EMM42_V5_0_DRIVER_HPP_ */
