# GNSS RXM-RAWX Measurement Data --- IGNORE ---

# Header header
float64 prMes      # Pseudorange measurement (m)
float64 cpMes      # Carrier phase measurement (m)
float32 doMes       # Doppler measurement (Hz)
uint8 gnssId     # GNSS identifier
uint8 svId      # Satellite identifier
uint8 reserved2  # Reserved
uint8 freqId   # Frequency identifier
uint16 lockTime # Lock time indicator
uint8 cno     # Carrier to noise ratio (dB-Hz) 
uint8 prStdev # Pseudorange standard deviation (m)
uint8 cpStdev # Carrier phase standard deviation (m)
uint8 doStdev # Doppler standard deviation (Hz)
uint8 trkStat # Tracking status bitfield
uint8 reserved3 # Reserved
