#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <iostream>
#include <iomanip>
#include <cstdint>
#include <serial/serial.h>
#include <ros/ros.h>
#include "byh_uav/uav_ublox_nav_pvt.h"
#include "byh_uav/uav_ublox_rxm_data.h"
#include "byh_uav/uav_ublox_rxm_rawx.h"
#include "byh_uav/uav_ublox_rxm_sfrbx.h"

/* 串口配置 */    
    std::string uart_port_name;
    // 串口通信波特率
    int serial_baud_rate;
    // 串口描述符
    serial::Serial ubx_Serial;  
/* 串口配置 */

struct UBX_NAV_PVT_Pack
{
    uint8_t CLASS;			// GNSS类
    uint8_t ID;				// ID类 
    uint16_t length;		// 数据包长度
    uint32_t iTOW;			// GPS时间信息（ms）
    uint16_t year;			// UTC年份信息
    uint8_t month;			// UTC月份信息
    uint8_t day;			// UTC日信息
    uint8_t hour;			// UTC时信息
    uint8_t min;			// UTC分信息
    uint8_t sec;			// UTC秒信息
    uint8_t valid;			// 数据是否有效
    uint32_t t_Acc;			// 时间精度估计（ns）
    int32_t nano;			// Fraction of second（ns）
    uint8_t fix_type;		// Fix类型
    uint8_t flags;			// Fix status flags
    uint8_t flags2;			// Additional flags
    uint8_t numSV;			// 导航解决方案中使用的卫星数量
    int32_t lon;			// 经度（deg）
    int32_t lat;			// 纬度（deg）
    int32_t height;			// 椭球以上的高度（mm）
    int32_t hMSL;			// 高于平均海平面的高度（mm）
    uint32_t hAcc;			// 水平精度估计（mm）
    uint32_t vAcc;			// 垂直精度估计（mm）
    int32_t velN;			// NED 向北速度（mm/s）
    int32_t velE;			// NED 向东速度（mm/s）
    int32_t velD;			// NED 下降速度（mm/s）
    int32_t gSpeed;			// 地速(2-D)（mm/s）
    int32_t headMot;		// Heading of motion (2-D)（deg）
    uint32_t sAcc;			// 速度精度估计（mm/s）
    uint32_t headAcc;		// 速度精度估计（deg）
    uint16_t pDOP;			// Position DOP
    uint8_t res1[6];		// Additional flags(附加标志)
    int32_t headVeh;		// 车辆航向(二维)，仅当设置 headVehValid 时才有效，否则输出置为运动航向
    uint8_t res2[4];		// 磁偏角。仅支持 ADR 4.10 及更高版本。
}__attribute__((packed));

struct RXM_DATA_Pack
{
    double   prMes;				// 伪距测量值（m）
    double   cpMes;				// 载波相位（T）
    float    doMes;				// 多普勒测量值（HZ)
    uint8_t  gnssId;			// 使用的GNSS系统
    uint8_t  svId;				// 卫星标识符
    uint8_t  reserved2;			// rev
    uint8_t  freqId;			// 频率标识符
    uint16_t locktime;			// 锁定时间（ms）
    uint8_t  cno;				// 信噪比（dB-Hz）
    uint8_t  prStdev;			// 伪距测量标准偏差
    uint8_t  cpStdev;			// 载波相位测量标准偏差
    uint8_t  doStdev;			// 多普勒测量标准偏差
    uint8_t  trkStat;			// 跟踪状态的附加信息
    uint8_t  reserved3;			// rev
}__attribute__((packed)); 

struct UBX_RXM_RAWX_Pack
{
    uint8_t  CLASS;				// GNSS 类
    uint8_t  ID;				// GNSS ID
    uint16_t length;			// 数据包长度
    double rcvTow;				// 接收机测量GPS时间
    uint16_t week;				// GPS周
    int8_t	 leapS;				// 闰秒
    uint8_t  numMeas;			// 观测值数量
    uint8_t  recStat;			// 接收机状态的额外信息
    uint8_t  reserved1[3];      // rev
}__attribute__((packed));              

struct UBX_RXM_SFRBX_Pack
{
    uint8_t  CLASS;				// GNSS 类
    uint8_t  ID;				// GNSS ID
    uint16_t length;			// 数据包长度
    uint8_t gnssId;				// 使用的GNSS系统
    uint8_t svId;				// 卫星标识符
    uint8_t reserved1;			// rev
    uint8_t freqId;				// 频率标识符
    uint8_t numWords;			// 消息数量
    uint8_t reserved2;			// rev	
    uint8_t version;			// 消息版本
    uint8_t reserved3;			// rev
}__attribute__((packed));

// GPS 状态信息
struct GPS_State_Machine
{
	// 帧数据标号
	uint32_t frame_datas_ind = 0;
	// 帧数据长度
	uint16_t frame_datas_length;
	// 读取速率
	uint8_t read_state = 0;	
	// 校验码
	uint8_t CK_A , CK_B;	
};	

static inline void ResetRxStateMachine( GPS_State_Machine* state_machine )
{
	state_machine->read_state=state_machine->frame_datas_ind=0;
}

/** 
 * @author WeiXuan
 * @brief GPS数据接收
 * @param state_machine GPS状态
 * @param frame_datas 帧数组
 * @param r_data 接收数据
 * @returns 
 */
static inline uint16_t GPS_ParseByte( GPS_State_Machine* state_machine, uint8_t* frame_datas, uint8_t r_data )
{
	frame_datas[ state_machine->frame_datas_ind++ ] = r_data;
	switch( state_machine->read_state )
	{
		// 找包头
		case 0:
		{				
			if( state_machine->frame_datas_ind == 1 )
			{
				if( r_data != 0xb5 )
					state_machine->frame_datas_ind = 0;
			}
			else
			{
				// 帧头确定
				if( r_data == 0x62 )	
				{
					// 跳转状态
					state_machine->read_state = 1;
					state_machine->CK_A = state_machine->CK_B = 0;
				}		
				else
					state_machine->frame_datas_ind = 0;
			}	
			break;
		}
		// 读Class ID和包长度
		case 1:	
		{
			state_machine->CK_A += r_data;
			state_machine->CK_B += state_machine->CK_A;
			if( state_machine->frame_datas_ind == 6 )
			{
				state_machine->frame_datas_length = (*(unsigned short*)&frame_datas[4]) + 6;
				if( state_machine->frame_datas_length > 4 && state_machine->frame_datas_length < 4096 )
					state_machine->read_state = 2;						
				else
					ResetRxStateMachine(state_machine);
			}
			break;
		}
		// 读包内容
		case 2:	
		{
			state_machine->CK_A += r_data;
			state_machine->CK_B += state_machine->CK_A;
			
			if( state_machine->frame_datas_ind == state_machine->frame_datas_length )
			{
				// 接收完成
				state_machine->read_state = 3;
			}
			break;
		}
		// 校验
		case 3:
		{
			if( state_machine->frame_datas_ind == state_machine->frame_datas_length + 1 )
			{
				if( r_data != state_machine->CK_A )
					ResetRxStateMachine(state_machine);
			}
			else
			{
				ResetRxStateMachine(state_machine);
				if( r_data == state_machine->CK_B )
					return state_machine->frame_datas_length;				
			}
			break;
		}
	}
	return 0;
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc,argv,"ubx_data_pub");
    // ROS节点句柄
    ros::NodeHandle nh;

     /* 串口配置 */
        // 固定串口号
        nh.param<std::string>("uart_port_name", uart_port_name, "/dev/ttyTHS0"); 
        // 和下位机通信波特率
        nh.param<int>("uart_baud_rate", serial_baud_rate, 921600);
    /* 串口配置 */

    // 发布器
    ros::Publisher rxm_rawx_pack_pub = nh.advertise<byh_uav::uav_ublox_rxm_rawx>("/byh_uav/ublox/rxm_rawx",10);
    ros::Publisher rxm_data_pack_pub = nh.advertise<byh_uav::uav_ublox_rxm_data>("/byh_uav/ublox/rxm_data",10);
    ros::Publisher rxm_sfrbx_pack_pub = nh.advertise<byh_uav::uav_ublox_rxm_sfrbx>("/byh_uav/ublox/rxm_sfrbx",10);
    ros::Publisher nav_pvt_pack_pub =nh.advertise<byh_uav::uav_ublox_nav_pvt>("/byh_uav/ublox/nav_pvt",10);

	// 复位所有状态
	GPS_State_Machine gps_state;
	ResetRxStateMachine(&gps_state);
	// 数据
	static uint8_t frame_datas[4096];
	frame_datas[0] = frame_datas[1] = 0; 

    try
    { 
        // 尝试初始化与开启串口
        ubx_Serial.setPort(uart_port_name); 
        ubx_Serial.setBaudrate(serial_baud_rate); 
        // 超时等待
        serial::Timeout _time = serial::Timeout::simpleTimeout(2000); 
        ubx_Serial.setTimeout(_time);
        ubx_Serial.open();
        
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("BYH_UAV_UART can not open serial port,Please check the serial port cable! ");
        return -1;
    }
    if(ubx_Serial.isOpen())
    {
        ROS_INFO_STREAM("ubx_Serial_UART serial port opened!");
    }

    std::cout << "Starting continuous read (press Ctrl+C to stop)..." << std::endl;

    while(ros::ok())
    {
        // 下位机数据
        uint8_t data[1];

        // 读取数据
        ubx_Serial.read(data,1); 
        uint16_t pack_length = GPS_ParseByte( &gps_state, frame_datas, data[0] );

        // 解析成功
        if( pack_length )
        {
            // UBX-NAV-PVT
            if( frame_datas[2]==0x01 && frame_datas[3]==0x07 )
            {
                // ROS_INFO("Get UBX-NAV-PVT \n");
                UBX_NAV_PVT_Pack* rpack = (UBX_NAV_PVT_Pack*)&frame_datas[2];
                // 协议
                byh_uav::uav_ublox_nav_pvt pvt_pack ;
                // 解析数据,赋值
                // pvt_pack.header.stamp = ros::Time::now(); 
                pvt_pack.CLASS=rpack->CLASS;
                pvt_pack.ID=rpack->ID;
                pvt_pack.length=rpack->length;
                pvt_pack.iTOW=rpack->iTOW;
                pvt_pack.year=rpack->year;
                pvt_pack.month=rpack->month;
                pvt_pack.day=rpack->day;
                pvt_pack.hour=rpack->hour;
                pvt_pack.min=rpack->min;
                pvt_pack.sec=rpack->sec;
                pvt_pack.valid=rpack->valid;
                pvt_pack.t_Acc=rpack->t_Acc;
                pvt_pack.nano=rpack->nano;
                pvt_pack.fix_type=rpack->fix_type;
                pvt_pack.flags=rpack->flags;
                pvt_pack.flags2=rpack->flags2;
                pvt_pack.numSV=rpack->numSV;
                pvt_pack.lon=rpack->lon;
                pvt_pack.lat=rpack->lat;
                pvt_pack.height=rpack->height;
                pvt_pack.hMSL=rpack->hMSL;
                pvt_pack.hAcc=rpack->hAcc;
                pvt_pack.vAcc=rpack->vAcc;
                pvt_pack.velN=rpack->velN;
                pvt_pack.velE=rpack->velE;
                pvt_pack.velD=rpack->velD;
                pvt_pack.gSpeed=rpack->gSpeed;
                pvt_pack.headMot=rpack->headMot;
                pvt_pack.sAcc=rpack->sAcc;
                pvt_pack.headAcc=rpack->headAcc;
                pvt_pack.pDOP=rpack->pDOP;
                pvt_pack.headVeh=rpack->headVeh;
                nav_pvt_pack_pub.publish(pvt_pack);
            }

            // UBX-RXM-RAWX	
            else if( frame_datas[2]==0x02 && frame_datas[3]==0x15 )
            {
                // ROS_INFO("Get UBX-RXM-RAWX \n");
                UBX_RXM_RAWX_Pack* rpack = (UBX_RXM_RAWX_Pack*)&frame_datas[2];
                byh_uav::uav_ublox_rxm_rawx pack;
                // pack.header.stamp = ros::Time::now(); 
                // 解析数据,赋值
                pack.CLASS=rpack->CLASS;
                pack.ID=rpack->ID;
                pack.length=rpack->length;
                pack.rcvTOW=rpack->rcvTow;
                pack.week=rpack->week;
                pack.leapS=rpack->leapS;
                pack.numMeas=rpack->numMeas;
                pack.recStat=rpack->recStat;
                rxm_rawx_pack_pub.publish(pack);
                // 搜星数
                uint8_t length = pack.numMeas;
                
                // 解析数据
                RXM_DATA_Pack *rdata;
                for(int i=0;i<length;i++)	
                {
                    rdata = (RXM_DATA_Pack*) &frame_datas[22+i*32];
                    byh_uav::uav_ublox_rxm_data data;
                    // 解析数据,赋值
                    data.prMes = rdata->prMes;
                    data.cpMes = rdata->cpMes;
                    data.doMes = rdata->doMes;
                    data.gnssId = rdata->gnssId;
                    data.svId = rdata->svId;
                    data.reserved2 = rdata->reserved2;
                    data.freqId = rdata->freqId;
                    data.lockTime = rdata->locktime;
                    data.cno = rdata->cno;
                    data.prStdev = rdata->prStdev;
                    data.cpStdev = rdata->cpStdev;
                    data.doStdev = rdata->doStdev;
                    data.trkStat = rdata->trkStat;
                    data.reserved3 = rdata->reserved3;
                    rxm_data_pack_pub.publish(data);
                }
            }
            
            // UBX-RXM-SFRBX	
            else if( frame_datas[2]==0x02 && frame_datas[3]==0x13 )
            {
                // ROS_INFO("Get UBX-RXM-SFRBX \n");
                UBX_RXM_SFRBX_Pack* rpack = (UBX_RXM_SFRBX_Pack*)&frame_datas[2];
                byh_uav::uav_ublox_rxm_sfrbx pack ;
                // 解析数据,赋值
                // pack.header.stamp = ros::Time::now(); 
                pack.CLASS=rpack->CLASS;
                pack.ID=rpack->ID;
                pack.length=rpack->length;
                pack.gnssId=rpack->gnssId;
                pack.svId=rpack->svId;
                pack.reserved1=rpack->reserved1;
                pack.freqId=rpack->freqId;
                pack.numWords=rpack->numWords;
                pack.reserved2=rpack->reserved2;
                pack.version=rpack->version;
                pack.reserved3=rpack->reserved3;
                rxm_sfrbx_pack_pub.publish(pack);

                struct UBX_RXM_SFRBX_WORDS_Pack
                {
                    uint32_t  words;				
                }__attribute__((packed));
                UBX_RXM_SFRBX_WORDS_Pack *data;

                // 数据字长度
                uint8_t length = rpack->numWords;
                uint32_t words[rpack->numWords];
                for(int i=0;i<length;i++)	
                {
                    data = (UBX_RXM_SFRBX_WORDS_Pack*) &frame_datas[14+i*4];
                    words[i] = data->words;
                    // 解析星历数据字，分系统实现

                }
            }	
        }
        
        ros::spinOnce();
    }
    return 0;
}
