# DRPVA ASCII

# Header of DRPVA ASCII
HeaderA header

string  sol_status
string  pos_type
string  datum_id
int8[4] reserved
float32 dr_age
float32 sol_age

float64 lat
float64 lon
float64 hgt

# Geoidal separation: the difference between the MSL and the WGS84
float32 undulation

# Standard deviation of lat, lon, hgt
float32 lat_sig
float32 lon_sig
float32 hgt_sig

# Velocity in ENU frame
float64 ve
float64 vn
float64 vu

# Standard deviation of velocity
float32 ve_sig
float32 vn_sig
float32 vu_sig

# Roll, Pitch, Yaw(=Heading)
float64 heading
float64 pitch
float64 roll

# Standard deviation of Roll, Pitch, Yaw(=Heading)
float32 heading_sig
float32 pitch_sig
float32 roll_sig

int32[4]   reserved1
float32[4] reserved2
float64[4] reserved3