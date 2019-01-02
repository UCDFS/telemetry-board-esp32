#ifndef UBX_PROTOCOL_H
#define UBX_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define UBX_HEADER_SYNC_CHAR_1 0xB5
#define UBX_HEADER_SYNC_CHAR_2 0x62

#define UBX_MESSAGE_MAX_LENGTH 512
#define UBX_MESSAGE_PAYLOAD_MAX_LENGTH 506

/**
 * UBX Class IDs
 */
typedef enum {
	UBX_MESSAGE_CLASS_NAV = 0x01, // Navigation Results Messages: Position, Speed, Time, Acceleration, Heading, DOP, SVs used,
	UBX_MESSAGE_CLASS_RXM = 0x02, // Receiver Manager Messages: Satellite Status, RTC Status
	UBX_MESSAGE_CLASS_INF = 0x04, // Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
	UBX_MESSAGE_CLASS_ACK = 0x05, // Ack/Nak Messages: Acknowledge or Reject messages to CFG input messages
	UBX_MESSAGE_CLASS_CFG = 0x06, // Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
	UBX_MESSAGE_CLASS_UPD = 0x09, // Firmware Update Messages: Memory/Flash erase/write, Reboot, Flash identification, etc.
	UBX_MESSAGE_CLASS_MON = 0x0A, // Monitoring Messages: Communication Status, CPU Load, Stack Usage, Task Status
	UBX_MESSAGE_CLASS_TIM = 0x0D, // Timing Messages: Time Pulse Output, Time Mark Results
	UBX_MESSAGE_CLASS_ESF = 0x10, // External Sensor Fusion Messages: External Sensor Measurements and Status Information
	UBX_MESSAGE_CLASS_MGA = 0x13, // Multiple GNSS Assistance Messages: Assistance data for various GNSS
	UBX_MESSAGE_CLASS_LOG = 0x21, // Logging Messages: Log creation, deletion, info and retrieval
	UBX_MESSAGE_CLASS_SEC = 0x27, // Security Feature Messages
	UBX_MESSAGE_CLASS_HNR = 0x28, // High Rate Navigation Results Messages: High rate time, position, speed, heading

	UBX_MESSAGE_CLASS_NMEA = 0xF0 // NMEA messages class when using CFG_MSG
} ubx_message_class_t;

/**
 * UBX message IDs
 */
typedef enum {
	/**
	 * ACK
	 */
	UBX_MESSAGE_ID_ACK_ACK = 0x01, // Message Acknowledged
	UBX_MESSAGE_ID_ACK_NAK = 0x00, // Message Not-Acknowledged

	/**
	 * CFG
	 */
	UBX_MESSAGE_ID_CFG_RST = 0x04, // Reset Receiver / Clear Backup Data Structures
	UBX_MESSAGE_ID_CFG_VALDEL = 0x8C, // Delete configuration values
	UBX_MESSAGE_ID_CFG_VALGET = 0x8B, // Get configuration values
	UBX_MESSAGE_ID_CFG_VALSET = 0x8A, // Set configuration values


	/**
	 * ESF
	 */
	UBX_MESSAGE_ID_ESF_INS = 0x15, // Vehicle dynamics information
	UBX_MESSAGE_ID_ESF_MEAS = 0x02, // External Sensor Fusion Measurements
	UBX_MESSAGE_ID_ESF_RAW = 0x03, // Raw sensor measurements
	UBX_MESSAGE_ID_ESF_STATUS = 0x10, // External Sensor Fusion (ESF) status information

	/**
	 * HNR
	 */
	UBX_MESSAGE_ID_HNR_PVT = 0x00, // High Rate Output of PVT Solution

	/**
	 * INF
	 */
	UBX_MESSAGE_ID_INF_DEBUG = 0x04, // ASCII output with debug contents
	UBX_MESSAGE_ID_INF_ERROR = 0x00, // ASCII output with error contents
	UBX_MESSAGE_ID_INF_NOTICE = 0x02, // ASCII output with informational contents
	UBX_MESSAGE_ID_INF_TEST = 0x03, // ASCII output with test contents
	UBX_MESSAGE_ID_INF_WARNING = 0x01, // ASCII output with warning contents

	/**
	 * LOG
	 */
	UBX_MESSAGE_ID_LOG_BATCH = 0x11, // Batched data
	UBX_MESSAGE_ID_LOG_CREATE = 0x07, // Create Log File
	UBX_MESSAGE_ID_LOG_ERASE = 0x03, // Erase Logged Data
	UBX_MESSAGE_ID_LOG_FINDTIME = 0x0E, // Find index of a log entry based on a given time
	                                    // Response to FINDTIME request.
	UBX_MESSAGE_ID_LOG_INFO = 0x08, // Poll for log information
	                                // Log information
	UBX_MESSAGE_ID_LOG_RETRIEVEBATCH = 0x10, // Request batch data
	UBX_MESSAGE_ID_LOG_RETRIEVEPOSEXTRA = 0x0f, // Odometer log entry
	UBX_MESSAGE_ID_LOG_RETRIEVEPOS = 0x0b, // Position fix log entry
	UBX_MESSAGE_ID_LOG_RETRIEVESTRING = 0x0d, // Byte string log entry
	UBX_MESSAGE_ID_LOG_RETRIEVE = 0x09, // Request log data
	UBX_MESSAGE_ID_LOG_STRING = 0x04, // Store arbitrary string in on-board flash

	/**
	 * MGA
	 */
	UBX_MESSAGE_ID_MGA_ACK_DATA0 = 0x60, // Multiple GNSS Acknowledge message
	UBX_MESSAGE_ID_MGA_ANO = 0x20, // Multiple GNSS AssistNow Offline Assistance
	UBX_MESSAGE_ID_MGA_BDS_EPH = 0x03, // BDS Ephemeris Assistance
	UBX_MESSAGE_ID_MGA_BDS_ALM = 0x03, // BDS Almanac Assistance
	UBX_MESSAGE_ID_MGA_BDS_HEALTH = 0x03, // BDS Health Assistance
	UBX_MESSAGE_ID_MGA_BDS_UTC = 0x03, // BDS UTC Assistance
	UBX_MESSAGE_ID_MGA_BDS_IONO = 0x03, // BDS Ionospheric Assistance
	UBX_MESSAGE_ID_MGA_DBD = 0x80, // Poll the Navigation Database
	                               // Navigation Database Dump Entry
	UBX_MESSAGE_ID_MGA_FLASH_DATA = 0x21, // Transfer MGA-ANO data block to flash
	UBX_MESSAGE_ID_MGA_FLASH_STOP = 0x21, // Finish flashing MGA-ANO data
	UBX_MESSAGE_ID_MGA_FLASH_ACK = 0x21, // Acknowledge last FLASH-DATA or -STOP
	UBX_MESSAGE_ID_MGA_GAL_EPH = 0x02, // Galileo Ephemeris Assistance
	UBX_MESSAGE_ID_MGA_GAL_ALM = 0x02, // Galileo Almanac Assistance
	UBX_MESSAGE_ID_MGA_GAL_TIMEOFFSET = 0x02, // Galileo GPS time offset assistance
	UBX_MESSAGE_ID_MGA_GAL_UTC = 0x02, // Galileo UTC Assistance
	UBX_MESSAGE_ID_MGA_GLO_EPH = 0x06, // GLONASS Ephemeris Assistance
	UBX_MESSAGE_ID_MGA_GLO_ALM = 0x06, // GLONASS Almanac Assistance
	UBX_MESSAGE_ID_MGA_GLO_TIMEOFFSET = 0x06, // GLONASS Auxiliary Time Offset Assistance
	UBX_MESSAGE_ID_MGA_GPS_EPH = 0x00, // GPS Ephemeris Assistance
	UBX_MESSAGE_ID_MGA_GPS_ALM = 0x00, // GPS Almanac Assistance
	UBX_MESSAGE_ID_MGA_GPS_HEALTH = 0x00, // GPS Health Assistance
	UBX_MESSAGE_ID_MGA_GPS_UTC = 0x00, // GPS UTC Assistance
	UBX_MESSAGE_ID_MGA_GPS_IONO = 0x00, // GPS Ionosphere Assistance
	UBX_MESSAGE_ID_MGA_INI_POS_XYZ = 0x40, // Initial Position Assistance
	UBX_MESSAGE_ID_MGA_INI_POS_LLH = 0x40, // Initial Position Assistance
	UBX_MESSAGE_ID_MGA_INI_TIME_UTC = 0x40, // Initial Time Assistance
	UBX_MESSAGE_ID_MGA_INI_TIME_GNSS = 0x40, // Initial Time Assistance
	UBX_MESSAGE_ID_MGA_INI_CLKD = 0x40, // Initial Clock Drift Assistance
	UBX_MESSAGE_ID_MGA_INI_FREQ = 0x40, // Initial Frequency Assistance
	UBX_MESSAGE_ID_MGA_INI_EOP = 0x40, // Earth Orientation Parameters Assistance
	UBX_MESSAGE_ID_MGA_QZSS_EPH = 0x05, // QZSS Ephemeris Assistance
	UBX_MESSAGE_ID_MGA_QZSS_ALM = 0x05, // QZSS Almanac Assistance
	UBX_MESSAGE_ID_MGA_QZSS_HEALTH = 0x05, // QZSS Health Assistance

	/**
	 * MON
	 */
	UBX_MESSAGE_ID_MON_BATCH = 0x32, // Data batching buffer status
	UBX_MESSAGE_ID_MON_COMMS = 0x36, // Comm port information
	UBX_MESSAGE_ID_MON_GNSS = 0x28, // Information message major GNSS selection
	UBX_MESSAGE_ID_MON_HW3 = 0x37, // HW I/O pin information
	UBX_MESSAGE_ID_MON_PATCH = 0x27, // Poll Request for installed patches
	                                 // Output information about installed patches.
	UBX_MESSAGE_ID_MON_RF = 0x38, // RF information
	UBX_MESSAGE_ID_MON_RXR = 0x21, // Receiver Status Information
	UBX_MESSAGE_ID_MON_SMGR = 0x2E, // Synchronization Manager Status
	UBX_MESSAGE_ID_MON_VER = 0x04, // Poll Receiver/Software Version
	                               // Receiver/Software Version

	/**
	 * NAV
	 */
	UBX_MESSAGE_ID_NAV_AOPSTATUS = 0x60, // AssistNow Autonomous Status
	UBX_MESSAGE_ID_NAV_ATT = 0x05, // Attitude Solution
	UBX_MESSAGE_ID_NAV_CLOCK = 0x22, // Clock Solution
	UBX_MESSAGE_ID_NAV_DGPS = 0x31, // DGPS Data Used for NAV
	UBX_MESSAGE_ID_NAV_DOP = 0x04, // Dilution of precision
	UBX_MESSAGE_ID_NAV_EOE = 0x61, // End Of Epoch
	UBX_MESSAGE_ID_NAV_GEOFENCE = 0x39, // Geofencing status
	UBX_MESSAGE_ID_NAV_HPPOSECEF = 0x13, // High Precision Position Solution in ECEF
	UBX_MESSAGE_ID_NAV_HPPOSLLH = 0x14, // High Precision Geodetic Position Solution
	UBX_MESSAGE_ID_NAV_ODO = 0x09, // Odometer Solution
	UBX_MESSAGE_ID_NAV_ORB = 0x34, // GNSS Orbit Database Info
	UBX_MESSAGE_ID_NAV_POSECEF = 0x01, // Position Solution in ECEF
	UBX_MESSAGE_ID_NAV_POSLLH = 0x02, // Geodetic Position Solution
	UBX_MESSAGE_ID_NAV_PVT = 0x07, // Navigation Position Velocity Time Solution
	UBX_MESSAGE_ID_NAV_RELPOSNED = 0x3C, // Relative Positioning Information in NED frame
	UBX_MESSAGE_ID_NAV_RESETODO = 0x10, // Reset odometer
	UBX_MESSAGE_ID_NAV_SAT = 0x35, // Satellite Information
	UBX_MESSAGE_ID_NAV_SBAS = 0x32, // SBAS Status Data
	UBX_MESSAGE_ID_NAV_SIG = 0x43, // Signal information
	UBX_MESSAGE_ID_NAV_SOL = 0x06, // Navigation Solution Information
	UBX_MESSAGE_ID_NAV_STATUS = 0x03, // Receiver Navigation Status
	UBX_MESSAGE_ID_NAV_SVINFO = 0x30, // Space Vehicle Information
	UBX_MESSAGE_ID_NAV_SVIN = 0x3B, // Survey-in data
	UBX_MESSAGE_ID_NAV_TIMEBDS = 0x24, // BDS Time Solution
	UBX_MESSAGE_ID_NAV_TIMEGAL = 0x25, // Galileo Time Solution
	UBX_MESSAGE_ID_NAV_TIMEGLO = 0x23, // GLO Time Solution
	UBX_MESSAGE_ID_NAV_TIMEGPS = 0x20, // GPS Time Solution
	UBX_MESSAGE_ID_NAV_TIMELS = 0x26, // Leap second event information
	UBX_MESSAGE_ID_NAV_TIMEUTC = 0x21, // UTC Time Solution
	UBX_MESSAGE_ID_NAV_VELECEF = 0x11, // Velocity Solution in ECEF
	UBX_MESSAGE_ID_NAV_VELNED = 0x12, // Velocity Solution in NED

	/**
	 * NMEA
	 */
	UBX_MESSAGE_ID_NMEA_DTM = 0x0A, // Datum Reference
	UBX_MESSAGE_ID_NMEA_GBQ = 0x44, // Poll a standard message (if the current Talker ID is GB)
	UBX_MESSAGE_ID_NMEA_GBS = 0x09, // GNSS Satellite Fault Detection
	UBX_MESSAGE_ID_NMEA_GGA = 0x00, // Global positioning system fix data
	UBX_MESSAGE_ID_NMEA_GLL = 0x01, // Latitude and longitude, with time of position fix and status
	UBX_MESSAGE_ID_NMEA_GLQ = 0x43, // Poll a standard message (if the current Talker ID is GL)
	UBX_MESSAGE_ID_NMEA_GNQ = 0x42, // Poll a standard message (if the current Talker ID is GN)
	UBX_MESSAGE_ID_NMEA_GNS = 0x0D, // GNSS fix data
	UBX_MESSAGE_ID_NMEA_GPQ = 0x40, // Poll a standard message (if the current Talker ID is GP)
	UBX_MESSAGE_ID_NMEA_GRS = 0x06, // GNSS Range Residuals
	UBX_MESSAGE_ID_NMEA_GSA = 0x02, // GNSS DOP and Active Satellites
	UBX_MESSAGE_ID_NMEA_GST = 0x07, // GNSS Pseudo Range Error Statistics
	UBX_MESSAGE_ID_NMEA_GSV = 0x03, // GNSS Satellites in View
	UBX_MESSAGE_ID_NMEA_RMC = 0x04, // Recommended Minimum data
	UBX_MESSAGE_ID_NMEA_TXT = 0x41, // Text Transmission
	UBX_MESSAGE_ID_NMEA_VLW = 0x0F, // Dual ground/water distance
	UBX_MESSAGE_ID_NMEA_VTG = 0x05, // Course over ground and Ground speed
	UBX_MESSAGE_ID_NMEA_ZDA = 0x08, // Time and Date


	/**
	 * RXM
	 */
	UBX_MESSAGE_ID_RXM_MEASX = 0x14, // Satellite Measurements for RRLP
	UBX_MESSAGE_ID_RXM_PMREQ = 0x41, // Requests a Power Management task
	                                 // Requests a Power Management task
	UBX_MESSAGE_ID_RXM_RAWX = 0x15, // Multi-GNSS Raw Measurement Data
	                                // Multi-GNSS Raw Measurement Data
	UBX_MESSAGE_ID_RXM_RLM = 0x59, // Galileo SAR Short-RLM report
	                               // Galileo SAR Long-RLM report
	UBX_MESSAGE_ID_RXM_RTCM = 0x32, // RTCM input status
	UBX_MESSAGE_ID_RXM_SFRBX = 0x13, // Broadcast Navigation Data Subframe
	                                 // Broadcast Navigation Data Subframe

	/**
	 * SEC
	 */
	UBX_MESSAGE_ID_SEC_SIGN = 0x01, // Signature of a previous message
	UBX_MESSAGE_ID_SEC_UNIQID = 0x03, // Unique Chip ID

	/**
	 * TIM
	 */
	UBX_MESSAGE_ID_TIM_DOSC = 0x11, // Disciplined oscillator control
	UBX_MESSAGE_ID_TIM_FCHG = 0x16, // Oscillator frequency changed notification
	UBX_MESSAGE_ID_TIM_HOC = 0x17, // Host oscillator control
	UBX_MESSAGE_ID_TIM_SMEAS = 0x13, // Source measurement
	UBX_MESSAGE_ID_TIM_SVIN = 0x04, // Survey-in data
	UBX_MESSAGE_ID_TIM_TM2 = 0x03, // Time mark data
	UBX_MESSAGE_ID_TIM_TOS = 0x12, // Time Pulse Time and Frequency Data
	UBX_MESSAGE_ID_TIM_TP = 0x01, // Time Pulse Timedata
	UBX_MESSAGE_ID_TIM_VCOCAL = 0x15, // Stop calibration
	                                  // VCO calibration extended command
	                                  // Results of the calibration
	UBX_MESSAGE_ID_TIM_VRFY = 0x06, // Sourced Time Verification

	/**
	 * UPD
	 */
	UBX_MESSAGE_ID_UPD_SOS = 0x14, // Poll Backup File Restore Status
	                               // Create Backup File in Flash
	                               // Clear Backup in Flash
	                               // Backup File Creation Acknowledge
	                               // System Restored from Backup
} ubx_message_id_t;

/**
 * UBX CFG Values
 */
typedef enum {
	CFG_UART2_ENABLED = 0x10530005
} ubx_cfg_val_key_t;

typedef struct {
	ubx_cfg_val_key_t key;
	uint64_t val;
	size_t size;
} ubx_cfg_val_val_t;

/**
 * Serial Communication Ports
 */
typedef enum {
	UBX_PORT_DDC = 0,
	UBX_PORT_UART = 1,
	UBX_PORT_USB = 3,
	UBX_PORT_SPI = 4
} ubx_port_t;

/**
 * UBX Header
 */
typedef struct {
	uint8_t sync_char_1;
	uint8_t sync_char_2;
	uint8_t message_class;
	uint8_t message_id;
	uint16_t payload_length;
} ubx_header_t;

/**
 * UBX checksum
 */
typedef struct {
	uint8_t ck_a;
	uint8_t ck_b;
} ubx_checksum_t;

/** ================================================================================ */
/**
 * ACK
 */
/**
* ACK-ACK
*
* Message Acknowledged
*/
typedef struct {
	ubx_message_class_t clsID :8; // Class ID of the (Not-)Acknowledged Message
	ubx_message_id_t msgId    :8; // Message ID of the (Not-)Acknowledged Message
} ubx_ack_ack_t;
typedef ubx_ack_ack_t ubx_ack_nack_t;

/** ================================================================================ */
/**
 * CFG
 */
/**
 * CFG-RST
 */
typedef struct {
	uint16_t navBbrMask :16; // BBR Sections to clear. The following Special Sets apply:
	                         // 0x0000 Hot start
	                         // 0x0001 Warm start
	                         // 0xFFFF Cold start
	uint8_t resetMode   :8 ; // Reset Type
	                         // 0x00 - Hardware reset (Watchdog) immediately
	                         // 0x01 - Controlled Software reset
	                         // 0x02 - Controlled Software reset (GNSS only)
	                         // 0x04 - Hardware reset (Watchdog) after shutdown
	                         // 0x08 - Controlled GNSS stop
	                         // 0x09 - Controlled GNSS start
	uint8_t reserved    :8 ; // Reserved
} ubx_cfg_rst_t;

/**
 * CFG-VAL*
 */
struct ubx_cfg_layers_t {
	bool ram            :1; // Apply to RAM layer
	bool bbr            :1; // Apply to BBR layer
	bool flash          :1; // Apply to flash layer
	uint8_t reserved2   :5;
};

enum ubx_cfg_layer_t {
	UBX_CFG_LAYER_RAM = 0,
	UBX_CFG_LAYER_BBR = 1,
	UBX_CFG_LAYER_FLASH = 2,
	UBX_CFG_LAYER_DEFAULT = 7
};

/**
 * CFG-VALDEL
 */
typedef struct {
	uint8_t version :8;
	struct ubx_cfg_layers_t layers;
	uint8_t transaction_action :2;
	uint16_t reserved :14;
} ubx_cfg_valdel_header_t;

/**
 * CFG-VALGET
 */
typedef struct {
	uint8_t version :8;
	enum ubx_cfg_layer_t layer :8;
	uint16_t reserved :16;
} ubx_cfg_valget_header_t;

/**
 * CFG-VALSET
 */
typedef struct {
	uint8_t version :8;
	struct ubx_cfg_layers_t layers;
	uint8_t transaction_action :2;
	uint16_t reserved :14;
} ubx_cfg_valset_header_t;

/** ================================================================================ */
/**
 * NAV
 */
/**
 * NAV-PVT
 *
 * Navigation Position Velocity Time Solution
 */
struct ubx_nav_pvt_bit_valid {
	bool validDate     :1 ; // 1 = valid UTC Date
	bool validTime     :1 ; // 1 = valid UTC Time of Day
	bool fullyResolved :1 ; // 1 = UTC Time of Day has been fully resolved (no seconds uncertainty)
	bool validMag      :1 ; // 1 = valid Magnetic declination
	uint8_t reserved   :4 ; // Reserved
};
struct ubx_nav_pvt_bit_flags {
	bool gnssFixOK     :1 ; // 1 = valid fix (i.e within DOP & accuracy masks)
	bool diffSoln      :1 ; // 1 = differential corrections were applied
	uint8_t psmState   :3 ; // Power Save Mode state :
	                        // 0: PSM is not active
	                        // 1: Enabled (an intermediate state before Acquisition state
	                        // 2: Acquisition
	                        // 3: Tracking
	                        // 4: Power Optimized Tracking
	                        // 5: Inactive
	bool headVehValid  :1 ; // 1 = heading of vehicle is valid
	uint8_t carrSoln   :2 ; // Carrier phase range solution status:
	                        // 0: no carrier phase range solution
	                        // 1: float solution (no fixed integer carrier phase measurements have been used to calculate the solution)
	                        // 2: fixed solution (one or more fixed integer carrier phase range measurements have been used to calculate the solution)
};
struct ubx_nav_pvt_bit_flags2 {
	uint8_t reserved   :5 ; // Reserved
	bool confirmedAvai :1 ; // 1 = information about UTC Date and Time of Day validity confirmation is available
	bool confirmedDate :1 ; // 1 = UTC Date validity could be confirmed
	bool confirmedTime :1 ; // 1 = UTC Time of Day could be confirmed
};

typedef struct {
	uint32_t iTOW      :32; // GPS time of week of the navigation epoch. (ms)
	uint16_t year      :16; // Year (UTC) (y)
	uint8_t month      :8 ; // Month, range 1..12 (UTC) (month)
	uint8_t day        :8 ; // Day of month, range 1..31 (UTC) (d)
	uint8_t hour       :8 ; // Hour of day, range 0..23 (UTC) (h)
	uint8_t min        :8 ; // Minute of hour, range 0..59 (UTC) (min)
	uint8_t sec        :8 ; // Seconds of minute, range 0..60 (UTC) (s)
	struct ubx_nav_pvt_bit_valid valid;   // Validity flags
	uint32_t tAcc      :32; // Time accuracy estimate (UTC) (ns)
	int32_t nano       :32; // Fraction of second, range -1e9 .. 1e9 (UTC) (ns)
	uint8_t fixType    :8 ; // GNSSfix Type:
	                        // 0: no fix
	                        // 1: dead reckoning only
	                        // 2: 2D-fix
	                        // 3: 3D-fix
	                        // 4: GNSS + dead reckoning combined
	                        // 5: time only fix
	struct ubx_nav_pvt_bit_flags flags;   // Fix status flags
	struct ubx_nav_pvt_bit_flags2 flags2; // Additional flags
	uint8_t numSV      :8 ; // Number of satellites used in Nav Solution
	int32_t lon        :32; // Longitude (deg 1e-7)
	int32_t lat        :32; // Latitude (deg 1e-7)
	int32_t height     :32; // Height above ellipsoid (mm)
	int32_t hMSL       :32; // Height above mean sea level (mm)
	uint32_t hAcc      :32; // Horizontal accuracy estimate (mm)
	uint32_t vAcc      :32; // Vertical accuracy estimate (mm)
	int32_t velN       :32; // NED north velocity (mm/s)
	int32_t velE       :32; // NED east velocity (mm/s)
	int32_t velD       :32; // NED down velocity (mm/s)
	int32_t gSpeed     :32; // Ground speed (2-D) (mm/s)
	int32_t headMot    :32; // Heading of motion (2-D) (deg 1e-5)
	uint32_t sAcc      :32; // Speed accuracy estimate (mm/s)
	uint32_t headAcc   :32; // Heading accuracy estimate (both motion and vehicle) (deg 1e-5)
	uint16_t pDOP      :16; // Position DOP (0.01)
	uint64_t reserved  :48;
	int32_t headVeh    :32; // Heading of vehicle (2-D) (deg 1e-5)
	int16_t magDec     :16; // Magnetic declination (deg 1e-2)
	uint16_t magAcc    :16; // Magnetic declination accuracy (deg 1e-2)
} ubx_nav_pvt_t;

/**
 * NAV-SAT
 *
 * Satellite Information
 */
struct ubx_nav_sat_bit_flags {
	uint8_t qualityInd  :3; // Signal quality indicator:
	                        // 0: no signal
	                        // 1: searching signal
	                        // 2: signal acquired
	                        // 3: signal detected but unusable
	                        // 4: code locked and time synchronized
	                        // 5, 6, 7: code and carrier locked and time synchronized
							// 		 Note: Since IMES signals are not time synchronized, a channel tracking an IMES signal can never reach a quality
	                        // indicator value of higher than 3
	bool svUsed         :1; // 1 = Signal in the subset specified in Signal Identifiers is currently being used for navigation
	uint8_t health      :2; // Signal health flag:
	                        // 0: unknown
	                        // 1: healthy
	                        // 2: unhealthy
	bool diffCorr       :1; // 1 = differential correction data is available for this SV
	bool smoothed       :1; // 1 = carrier smoothed pseudorange used
	uint8_t orbitSource :3; // Orbit source:
	                        // 0: no orbit information is available for this SV
	                        // 1: ephemeris is used
	                        // 2: almanac is used
	                        // 3: AssistNow Offline orbit is used
	                        // 4: AssistNow Autonomous orbit is used
	                        // 5, 6, 7: other orbit information is used
	bool ephAvail       :1; // 1 = ephemeris is available for this SV
	bool almAvail       :1; // 1 = almanac is available for this SV
	bool anoAvail       :1; // 1 = AssistNow Offline data is available for this SV
	bool aopAvail       :1; // 1 = AssistNow Autonomous data is available for this SV
	uint8_t reserved1   :1;
	bool sbasCorrUsed   :1; // 1 = SBAS corrections have been used for a signal in the subset specified in Signal Identifiers
	bool rtcmCorrUsed   :1; // 1 = RTCM corrections have been used for a signal in the subset specified in Signal Identifiers
	bool slasCorrUsed   :1; // 1 = QZSS SLAS corrections have been used for a signal in the subset specified in Signal Identifiers
	uint8_t reserved2   :1;
	bool prCorrUsed     :1; // 1 = Pseudorange corrections have been used for a signal in the subset specified in Signal Identifiers
	bool crCorrUsed     :1; // 1 = Carrier range corrections have been used for a signal in the subset specified in Signal Identifiers
	bool doCorrUsed     :1; // 1 = Range rate (Doppler) corrections have been used for a signal in the subset specified in Signal Identifiers
	uint16_t reserved3  :9;
};

typedef struct {
	uint8_t gnssId :8 ; // GNSS identifier (see Satellite Numbering) for assignment
	uint8_t svId   :8 ; // Satellite identifier (see Satellite Numbering) for assignment
	uint8_t cno    :8 ; // Carrier to noise ratio (signal strength) (dbHz)
	int8_t elev    :8 ; // Elevation (range: +/-90), unknown if out of range (deg)
	int16_t azim   :16; // Azimuth (range 0-360), unknown if elevation is out of range (deg)
	int16_t prRes  :16; // Pseudorange residual (m 0.1)
	struct ubx_nav_sat_bit_flags flags; // Satellite flags
} ubx_nav_sat_sat_t;

typedef struct {
	uint32_t iTOW      :32; // GPS time of week of the navigation epoch. (ms)
	uint8_t version    :8 ; // Message version
	uint8_t numSvs     :8 ; // Number of satellites
	uint16_t reserved  :16;
	ubx_nav_sat_sat_t satellites[]; // Satellites
} ubx_nav_sat_t;

#endif //UBX_PROTOCOL_H
