# NAVPVT Message
# GNSS class and ID
# Header header
uint8 CLASS
uint8 ID
uint16 length

# GPS time information
uint32 iTOW

# UTC date and time
uint16 year
uint8 month
uint8 day
uint8 hour
uint8 min
uint8 sec

# Validity and accuracy
uint8 valid
uint32 t_Acc
int32 nano

# Fix information
uint8 fix_type
uint8 flags
uint8 flags2
uint8 numSV

# Position coordinates
int32 lon           # Longitude (deg * 1e-7)
int32 lat           # Latitude (deg * 1e-7)
int32 height        # Height above ellipsoid (mm)
int32 hMSL          # Height above mean sea level (mm)

# Accuracy estimates
uint32 hAcc         # Horizontal accuracy estimate (mm)
uint32 vAcc         # Vertical accuracy estimate (mm)

# Velocity information
int32 velN          # NED north velocity (mm/s)
int32 velE          # NED east velocity (mm/s)
int32 velD          # NED down velocity (mm/s)
int32 gSpeed        # Ground speed (2D) (mm/s)

# Heading information
int32 headMot       # Heading of motion (2D) (deg * 1e-5)
uint32 sAcc         # Speed accuracy estimate (mm/s)
uint32 headAcc      # Heading accuracy estimate (deg * 1e-5)

# DOP information
uint16 pDOP         # Position DOP

# Reserved fields additional flags 附加标志
uint8[6] res1

# Vehicle heading
int32 headVeh       # Vehicle heading (2D) (deg * 1e-5)

# Additional reserved fields 磁偏角
uint8[4] res2