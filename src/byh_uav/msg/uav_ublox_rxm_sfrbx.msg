# GNSS SFRBX (Subframe Buffer) Message
# This message contains the pack of subframe data from GNSS satellites.
# no detailed information about the navigation data is provided here.

# Header header
uint8 CLASS   # Message class
uint8 ID      # Message ID
uint16 length  # Length of the payload
uint8 gnssId   # GNSS identifier
uint8 svId     # Satellite vehicle ID
uint8 reserved1 # reserved field
uint8 freqId    # Frequency ID
uint8 numWords # Number of words (max 10)
uint8 reserved2 # reserved field
uint8 version   # Version of the message
uint8 reserved3 # reserved field