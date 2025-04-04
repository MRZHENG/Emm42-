#ifndef EMM42_V5_0_DRIVER_HPP_
#define EMM42_V5_0_DRIVER_HPP_
#include "HardwareSerial.h"

//当前电机的修改是思路 定义系统参数类

#define MAX_RETRY 3 //最大重试次数
#define Nearest  0   /*单圈就近回零*/
#define Dir      1   /*单圈方向回零*/
#define Senless  2   /*无限位回零*/
#define Endstop  3   /*限位回零*/

using  Trigger_ZeroMode = uint8_t;


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
}SysParams_t;         /* 系统参数类*/



// 原点回零参数类型
struct Emm42_Zero_Param {
    Trigger_ZeroMode mode;        /*回零模式*/
    uint8_t  dir;                 /*回零方向*/
    uint16_t Rpm;               /*回零速度*/
    uint32_t OutTime;             /*回零超时时间*/
    uint16_t Senless_CheckRPM;    /*无限位回零检测转速*/
    uint16_t Senless_CheckMA;     /*无限位回零检测电流*/
    uint16_t Senless_CheckTime;   /*无限位回零检测时间*/
    uint8_t  auto_ToZero_Flag;    /*上电自动触发回零*/
};

// 回零状态标志位
struct Emm42_Zero_StatusFlag{
    bool Backing_zero_Flag;    /*正在回零标志位*/
    bool Coder_Ready_Flag;     /*编码器就绪标志位*/
    bool Backing_False_Flag;   /*回零失败标志位*/
    bool CheckDirgram_Flag;    /*校准表就绪标志位*/
};

// 电机状态标志位
struct  Emm42_Motor_StatusFlag
{
   bool  Enable_Flag;        /*使能标志位*/
   bool  InPosition_Flag;    /*到位标志位*/
   bool  Block_Flag;         /*堵转标志位*/
   bool  Block_Protect_Flag; /*堵转保护标志位*/
};



//Emm42步进驱动类
class Emm42_V5_0_Driver {
public:

/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓构造函数↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
    /**
    * @brief    默认构造函数
    * @param    void
    * @retval   void
   */
    Emm42_V5_0_Driver(){
        id = 1;
        this->serial = &Serial2;
        check_Byte = 0x6B;
        is_serialMotor_enable = false;
        //是否使能
        Motor_Status.Enable_Flag = false;
        //是否开启堵转保护
        Motor_Status.Block_Protect_Flag = true;
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
        is_serialMotor_enable = false;
        Motor_Status.Enable_Flag =false;
        block_protect = Block_protect;
    }
/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/


/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓使能函数↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
    
    /**
    * @brief    串口初始化
    * @param    void
    * @retval   void
    */
    void serial_enable(){
        this->serial->begin(115200); //设置串口默认速率
        this->is_serialMotor_enable = true;
    }

     /**
    * @brief    电机使能
    * @param    void
    * @retval   Emm42_CMDstatus：命令执行状态
    * @description: 发送使能指令，并接收电机返回值，返回值可能是成功，条件不满足，错误命令，串口未初始化。
    */
    Emm42_CMDstatus Emm_V5_Motor_enable(){
        //初始化串口
        serial_enable();
        
        if(is_serialMotor_enable == true){
        uint8_t len = 6;
        uint8_t cmd[6] = {id,0xF3,0xAB,0x01,0x00,check_Byte};
        
        //发送使能指令
        send(cmd,len); 
        //接受电机返回值
        read(cmd,4);
        switch (cmd[2])
        {
        case 0x02:
            Motor_Status.Enable_Flag = true;
            return Emm42_OK;
            break;
        case 0xE2:
            Motor_Status.Enable_Flag = false;
            return Emm42_False;
            break;
        case 0xEE:
            Motor_Status.Enable_Flag = false;
            return Emm42_Error;
            break;
        }
        Motor_Status.Enable_Flag = false;
        return Emm42_Error;
       }
       Motor_Status.Enable_Flag = false;
       return Emm42_SerialError ;   
    }

    
    /**
    * @brief    开启电机同步
    * @param    void
    * @retval   Emm42_CMDstatus：命令执行状态
    * @description: 让多个电机进行同步运动。
    */
   Emm42_CMDstatus sync_Motor_enable(){
      uint8_t cmd[16] = {0};
      
      //装载命令
      cmd[0] = id;       //控制目标电机
      cmd[1] = 0xFF;     //功能码
      cmd[2] = 0x66;     //辅助码
      cmd[3] = this->check_Byte; //校验码
      
      send(cmd,4);
      read(cmd,4);
      if(cmd[2] == 0xE2){
        return Emm42_False;
      }else if(cmd[2] == 0xEE){
        return Emm42_Error;
      }
      return Emm42_OK;
    }
/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/


/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓控制函数↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
    
    /**
    * @brief    将当前位置角度清零
    * @param    void
    * @retval   Emm42_CMDstatus
    * @description: 让多个电机进行同步运动。
    */
   Emm42_CMDstatus Emm_V5_Reset_CurPos_To_Zero(){
     uint8_t cmd[16] = {0};

     //装载命令
     cmd[0] = id;       //控制目标电机
     cmd[1] = 0x0A;     //功能码
     cmd[2] = 0x6D;     //辅助码
     cmd[3] = this->check_Byte; //校验码
     
     send(cmd,4);
     read(cmd,4);
     if(cmd[2]==0xEE)
     return Emm42_Error;
     return Emm42_OK;
   }

    /**
    * @brief    速度模式
    * @param    dir ：方向       ，0为CW，其余值为CCW
    * @param    vel ：速度       ，范围0 - 5000RPM
    * @param    snF ：多机同步标志，false为不启用，true为启用
    * @param    acc ：加速度     ，范围0 - 255，注意：0是直接启动(不需要加速度)
    * @retval   Emm42_CMDstatus
    * @description: 电机以指定速度和加速度运动,如果返回Emm42_False，可能是电机没使能或者触发了堵转保护
    */
   Emm42_CMDstatus Emm_V5_Vel_Control(uint8_t dir,uint8_t vel,bool snF = false,uint8_t acc=0){
     uint8_t cmd[16] = {0};
     //对速度限制函数
     vel = vel>5000?5000:vel;
     vel = vel<0?0:vel;

     //装载命令
     cmd[0] = id;       //控制目标电机
     cmd[1] = 0xF6;     //功能码
     cmd[2] = dir;      //方向
     cmd[3] = (uint8_t)vel>>8;  //速度高8位
     cmd[4] = (uint8_t)vel;     //速度低8位
     cmd[5] = acc;      //加速度
     cmd[6] = snF;      //多机同步标志
     cmd[7] = this->check_Byte; //校验码
     //发送
     send(cmd,8);
     read(cmd,4);
     if(cmd[2]== 0x02){
        return Emm42_OK;
     }else if(cmd[2]== 0xE2){
        return Emm42_False;
     }
     return Emm42_Error;
    }

    /**
    * @brief    位置模式
    * @param    dir ：方向        ，0为CW，其余值为CCW
    * @param    vel ：速度(RPM)   ，范围0 - 5000RPM
    * @param    clk ：脉冲数      ，范围0- (2^32 - 1)个  (默认16细分 3200为转一圈)
    * @param    snF ：多机同步标志 ，false为不启用，true为启用(默认不启动同步控制)
    * @param    raF ：相位/绝对标志，false为相对运动，true为绝对值运动(默认为绝对运动)
    * @param    acc ：加速度      ，范围0 - 255，注意：0是直接启动
    * @retval   Emm42_CMDstatus
    */
   Emm42_CMDstatus Emm_V5_Pos_Control(uint8_t dir, uint16_t vel,  uint32_t clk, bool snF=false, bool raF=true,uint8_t acc=0)
    {
    uint8_t cmd[16] = {0};
    //对速度限制函数
    vel = vel>5000?5000:vel;
    vel = vel<0?0:vel;
    //装载命令
    cmd[0] = this->id;  
    cmd[1]  =  0xFD;                      // 功能码
    cmd[2]  =  dir;                       // 方向
    cmd[3]  =  (uint8_t)(vel >> 8);       // 速度(RPM)高8位字节
    cmd[4]  =  (uint8_t)(vel >> 0);       // 速度(RPM)低8位字节 
    cmd[5]  =  acc;                       // 加速度，注意：0是直接启动
    cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数(bit24 - bit31)
    cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数(bit16 - bit23)
    cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数(bit8  - bit15)
    cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数(bit0  - bit7 )
    cmd[10] =  raF;                       // 相位/绝对标志，false为相对运动，true为绝对值运动
    cmd[11] =  snF;                       // 多机同步运动标志，false为不启用，true为启用
    cmd[12] =  this->check_Byte;          // 校验字节
    
    send(cmd,13);
    //读取返回命令
    read(cmd,4);
    if(cmd[2] == 0x02){
        return Emm42_OK;
    }else if(cmd[2]==0xE2){
        return Emm42_False;
    }else{
        return Emm42_Error; 
    };
    }


    /**
     * @brief    立即停止（所有控制模式都通用）
     * @param    snF   ：多机同步标志，false为不启用，true为启用(默认不启用)
     * @retval   Emm42_CMDstatus
     */
    Emm42_CMDstatus Emm_V5_Stop_Now(bool snF = false)
    {
    uint8_t cmd[16] = {0};
    
    // 装载命令
    cmd[0] =  id;                       // 地址
    cmd[1] =  0xFE;                       // 功能码
    cmd[2] =  0x98;                       // 辅助码
    cmd[3] =  snF;                        // 多机同步运动标志
    cmd[4] =  this->check_Byte;                       // 校验字节
    // 发送命令
    send(cmd, 5);
    // 读取电机回传命令
    read(cmd,4);
    if(cmd[2] == 0x02){
        return Emm42_OK;
    }else if(cmd[2] == 0xE2){
        return Emm42_False;
    }
    return Emm42_Error;
    }


/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/

/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓回零函数↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
   
    /**
     * @brief    触发回零
     * @param    snF   ：多机同步标志，false为不启用，true为启用
     * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
     * @retval   地址 + 功能码 + 命令状态 + 校验字节
     */
    Emm42_CMDstatus Emm_V5_TriggerToZero(bool snF = false,uint8_t o_mode = 2){
        uint8_t cmd[16] = {0};
        // 装载命令
        cmd[0] =  id;                  // 地址
        cmd[1] =  0x9A;                // 功能码
        cmd[2] =  o_mode;              // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
        cmd[3] =  snF;                 // 多机同步运动标志，false为不启用，true为启用
        cmd[4] =  this->check_Byte;    // 校验字节
            
        // 发送命令
        send(cmd,5);
        // 接受命令
        read(cmd,4);
        
        if(cmd[2] == 0x02){
            return Emm42_OK;
        }else if(cmd[2] == 0xE2){
            return Emm42_False;
        }
        return Emm42_Error;
    }

    
     /**
     * @brief    读取回零函数参数
     * @retval   Emm42_CMDstatus
     */
    Emm42_CMDstatus Emm_V5_ReadZeroParam(){
        uint8_t cmd[3] = {0};
        uint8_t data[18] = {0};
        
        // 装载命令
        cmd[0] = id;
        cmd[1] = 0x22;
        cmd[2] = this->check_Byte;

        // 发送命令
        send(cmd,3);
        delay(10);
        // 接受命令
        if(this->serial->available()==4){
            serial->flush();
            return Emm42_Error;
        }
        read(data,18);
        this->zero_Param.mode = data[2];
        this->zero_Param.dir  = data[3];
        this->zero_Param.Rpm  = (uint16_t)data[4]<<8 | data[5];
        this->zero_Param.OutTime = (uint32_t)data[6]<<24 | (uint32_t)data[7]<<16 | (uint32_t)data[8]<<8 | data[9];
        this->zero_Param.Senless_CheckRPM = (uint16_t)data[10]<<8 | data[11];
        this->zero_Param.Senless_CheckMA = (uint16_t)data[12]<<8 | data[13];
        this->zero_Param.Senless_CheckTime = (uint16_t)data[14]<<8 | data[15];
        this->zero_Param.auto_ToZero_Flag = data[16];
        return Emm42_OK;
    }
    
    /**
     * @brief    修改回零参数
     * @param    addr  ：电机地址
     * @param    o_mode ：回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
     * @param    o_dir  ：回零方向，0为CW，其余值为CCW
     * @param    o_vel  ：回零速度，单位：RPM（转/分钟）
     * @param    o_tm   ：回零超时时间，单位：毫秒
     * @param    sl_vel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
     * @param    sl_ma  ：无限位碰撞回零检测电流，单位：Ma（毫安）
     * @param    sl_ms  ：无限位碰撞回零检测时间，单位：Ms（毫秒）
     * @param    potF   ：上电自动触发回零，false为不使能，true为使能  
     * @param    svF   ：是否存储标志，false为不存储，true为存储 (默认存储)
     * @retval   Emm42_CMDstatus  返回命令执行状态
     */
    Emm42_CMDstatus Emm_V5_Modify_ZeroParams(  uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF,bool svF = true)
    {
    uint8_t cmd[32] = {0};
    
    // 装载命令
    cmd[0] =  id;                         // 地址
    cmd[1] =  0x4C;                       // 功能码
    cmd[2] =  0xAE;                       // 辅助码
    cmd[3] =  svF;                        // 是否存储标志，false为不存储，true为存储
    cmd[4] =  o_mode;                     // 回零模式，0为单圈就近回零，1为单圈方向回零，2为多圈无限位碰撞回零，3为多圈有限位开关回零
    cmd[5] =  o_dir;                      // 回零方向
    cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度(RPM)高8位字节
    cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度(RPM)低8位字节 
    cmd[8]  =  (uint8_t)(o_tm >> 24);     // 回零超时时间(bit24 - bit31)
    cmd[9]  =  (uint8_t)(o_tm >> 16);     // 回零超时时间(bit16 - bit23)
    cmd[10] =  (uint8_t)(o_tm >> 8);      // 回零超时时间(bit8  - bit15)
    cmd[11] =  (uint8_t)(o_tm >> 0);      // 回零超时时间(bit0  - bit7 )
    cmd[12] =  (uint8_t)(sl_vel >> 8);    // 无限位碰撞回零检测转速(RPM)高8位字节
    cmd[13] =  (uint8_t)(sl_vel >> 0);    // 无限位碰撞回零检测转速(RPM)低8位字节 
    cmd[14] =  (uint8_t)(sl_ma >> 8);     // 无限位碰撞回零检测电流(Ma)高8位字节
    cmd[15] =  (uint8_t)(sl_ma >> 0);     // 无限位碰撞回零检测电流(Ma)低8位字节 
    cmd[16] =  (uint8_t)(sl_ms >> 8);     // 无限位碰撞回零检测时间(Ms)高8位字节
    cmd[17] =  (uint8_t)(sl_ms >> 0);     // 无限位碰撞回零检测时间(Ms)低8位字节
    cmd[18] =  potF;                      // 上电自动触发回零，false为不使能，true为使能
    cmd[19] =  this->check_Byte;          // 校验字节
    // 发送命令
    send(cmd,20);
    // 读取接收的命令
    read(cmd,4);
    if(cmd[2] == 0x02){
        return Emm42_OK;
    }
    return Emm42_Error;  
    }

    /**
     * @brief    将当前位置清零
     * @retval   Emm42_CMDstatus  返回命令执行状态
     */
    Emm42_CMDstatus Emm_V5_ClearPos(){
        uint8_t cmd[4] = {0};
        //装载命令
        cmd[0] = id;                      // 地址
        cmd[1] = 0x0A;                    // 功能码
        cmd[2] = 0x6D;                    // 状态码
        cmd[3] = this->check_Byte;        // 校验字节
        //发送命令
        send(cmd,4);
        //接受命令
        read(cmd,4);
        if(cmd[2] == 0x02){
            return Emm42_OK;
        }
        return Emm42_Error ;
    }
/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/


/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓读取函数↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/
    /**
     * @brief    读取电机的目标位置
     * @retval   float返回电机的目标圈的位置
     */
    float Emm_V5_ReadTargetPos(){
        uint8_t cmd[16] = {0};
        //装载命令
        cmd[0] = id;
        cmd[1] = 0x33;
        cmd[2] = check_Byte;
        //发送命令
        send(cmd,3);
        //接受命令
        delay(3);
        if(serial->available()==4){
            serial->flush();
            return  0xFFFFFFFF;
        }else{
            read(cmd,8);
        }
        int32_t data = (int32_t)cmd[3]<<24 | (int32_t)cmd[4]<<16 | (int32_t)cmd[5]<<8 | cmd[6];
        data = cmd[2]==0x00?data:-data;
        return (float)data*360/65536;
    }

    /**
     * @brief    读取电机的当前位置
     * @retval   float返回电机当前的位置
     */
    float Emm_V5_ReadCurPos(){
        uint8_t cmd[16] = {0};
        //装载命令
        cmd[0] = id;
        cmd[1] = 0x36;
        cmd[2] = check_Byte;
        //发送命令
        send(cmd,3);
        //接受命令
        delay(3);
        if (serial->available() == 4)   
        {
            serial->flush();
            return 0xFFFFFFFF;
        }
        else{
            read(cmd,8);
        }
        int32_t data = (int32_t)cmd[3]<<24 | (int32_t)cmd[4]<<16 | (int32_t)cmd[5]<<8 | cmd[6];
        data = cmd[2]==0x00?data:-data;
        return (float)data*360/65536;
    }


    /**
     * @brief    读取电机的实时转速
     * @retval   float返回电机当前的位置
     */
    uint16_t EmmV5_ReadCurRPM(){
        uint8_t cmd[16] = {0};
        uint16_t curRPM = 0;
    
        //装载命令
        cmd[0] = id;
        cmd[1] = 0x35;
        cmd[2] = check_Byte;
        //发送命令
        send(cmd,3);
        //接受命令
        delay(1);
        if(serial->available()==4){
            serial->flush();
            return  0xFFFF;
        }else{
            read(cmd,6);
            curRPM = (uint16_t)cmd[3]<<8 | cmd[4];
            curRPM = cmd[2]==0x00?curRPM:-curRPM;
        }
        return curRPM;
    }
    
   
     /**
     * @brief    读取电机状态标志位
     * @retval   Emm42_CMDstatus  返回命令执行状态
     */
    Emm42_CMDstatus Emm_V5_Read_MotorStatusFlag(){
        uint8_t cmd[16] = {0};
        //装载命令
        cmd[0] = id;
        cmd[1] = 0x3A;
        cmd[2] = check_Byte;
        //发送命令
        send(cmd,3);
        //接受命令
        read(cmd,4);
        if(cmd[1] == 0x00){
            return Emm42_Error;
        }
        uint8_t tmp = cmd[2];
        Motor_Status.Enable_Flag = tmp & 0x01;
        Motor_Status.InPosition_Flag = tmp & 0x02;
        Motor_Status.Block_Flag = tmp & 0x04;
        Motor_Status.Block_Protect_Flag = tmp & 0x08;
        return Emm42_OK;
    }

/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/


/*↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓修改参数函数↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*/



/*↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*/


private:
    HardwareSerial*serial;//设备绑定的串口
    uint8_t id;           //设备id
    uint8_t check_Byte;   //校验字节
    bool is_serialMotor_enable;        //串口是否初始化
    bool block_protect;   //堵转保护(默认开启)
    Emm42_Motor_StatusFlag  Motor_Status; //电机状态标志位

    // 新添加的参数(需要后续添加到构造函数中去的)
    Emm42_Zero_Param zero_Param; //回零参数
    Emm42_Zero_StatusFlag zero_StatusFlag; //回零状态标志位(暂时没有用到)


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
