# RXM_DATA_PACK Message
# Header header
uint8 CLASS #Messsage CLASS
uint8 ID   #Message ID
uint16 length #Length of payload
float64 rcvTOW # Receiver Time of Week (s)
uint16 week # GPS week number of the receiver time of week
int8 leapS # Leap seconds (s)
uint8 numMeas # Number of measurements to follow
uint8 recStat # 接收机状态
uint8 reserved # Reserved
