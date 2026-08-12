# Sync character
string sync

# The ASCII name of the log or command
string message

# The name of the port from which the log was generated.(0,1 ~ 31)
uint8 port

# Used for multiple related logs.(N-1 ~ 0)
int16 sequence_number

# The minimum percentage of time the processor is idle, calculated once per second.
float32 idle_time

# The quality of the GPS reference time.
string time_status

# GPS reference week number.
uint32 week

# Seconds from the beginning of the GPS
# reference week; accurate to the
# millisecond level
float32 seconds

uint32 receiver_status

# BDS time offset to GPS second
uint16 bds_time

# UTC time offset to GPS second. Current leap seconds
uint32 utc_time