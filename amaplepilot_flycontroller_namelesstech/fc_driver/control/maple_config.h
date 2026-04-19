



/*****************遥控器行程设置**********************/
#define  Climb_Up_Speed_Max    400//向上最大攀爬速度，cm/s  400  300
#define  Climb_Down_Speed_Max  250//向下最大下降速度，cm/s  200  150
#define  Flight_Safe_Vbat  14200   //14200mv=14.2V
/*****************一键起飞高度设置，单位为cm，比如100表示原地起飞到相对初始位置1米高的位置**********************/
#define  Auto_Launch_Target    150//一键起飞的目标高度，相对起飞高度，使用超声波时，请勿超过超过声波量程，推荐200以下
#define  Nav_Safety_Height     800				//距离返航点较远时，返航所需的安全巡航高度，单位cm
#define  Flight_Max_Height     5000//最大飞行高度5000M
#define  Flight_Max_Radius     5000//最大飞行半径5000M
#define  Nav_Speed_Max  500//最大期望水平速度为5m/s
#define Nav_Near_Ground_Height_Default 100//接近地面的高度，单位cm
typedef enum 
{
  GPS_M8N=1,
  THIRD_PARTY_STATE=2,
}RESERVED_UART_MAP;


#define RESERVED_UART_DEFAULT GPS_M8N//THIRD_PARTY_STATE;
//GPS_M8N
//THIRD_PARTY_STATE

#define nUART_DEFAULT 0x00


typedef enum 
{
  OPTICALFLOW0=1,
  RPISLAM=2,
}_INDOOR_POSITION_TYPE;


typedef enum 
{
  TOFSENSE=1,	
  VL53L8=2,
  UP_Tx=3,
  MT1=4,
  GYTOF10M=5,
  TFMINI=6,
}_RANGE_TYPE;





