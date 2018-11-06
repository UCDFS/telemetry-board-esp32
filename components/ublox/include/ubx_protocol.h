#ifndef UBX_PROTOCOL_H
#define UBX_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

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
	UBX_MESSAGE_CLASS_AID = 0x0B, // AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
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
	 * AID
	 */
	UBX_MESSAGE_ID_AID_ALM = 0x30, // Poll GPS Aiding Almanac Data
	                               // Poll GPS Aiding Almanac Data for a SV
	                               // GPS Aiding Almanac Input/Output Message
	UBX_MESSAGE_ID_AID_AOP = 0x33, // Poll AssistNow Autonomous data, all satellites
	                               // Poll AssistNow Autonomous data, one GPS...
	                               // AssistNow Autonomous data
	UBX_MESSAGE_ID_AID_EPH = 0x31, // Poll GPS Aiding Ephemeris Data
	                               // Poll GPS Aiding Ephemeris Data for a SV
	                               // GPS Aiding Ephemeris Input/Output Message
	UBX_MESSAGE_ID_AID_HUI = 0x02, // Poll GPS Health, UTC, ionosphere parameters
	                               // GPS Health, UTC and ionosphere parameters
	UBX_MESSAGE_ID_AID_INI = 0x01, // Poll GPS Initial Aiding Data
	                               // Aiding position, time, frequency, clock drift

	/**
	 * CFG
	 */
	UBX_MESSAGE_ID_CFG_ANT = 0x13, // Antenna Control Settings
	UBX_MESSAGE_ID_CFG_BATCH = 0x93, // Get/Set data batching configuration
	UBX_MESSAGE_ID_CFG_CFG = 0x09, // Clear, Save and Load configurations
	UBX_MESSAGE_ID_CFG_DAT = 0x06, // Set User-defined Datum.
	                               // The currently defined Datum
	UBX_MESSAGE_ID_CFG_DGNSS = 0x70, // DGNSS configuration
	UBX_MESSAGE_ID_CFG_DOSC = 0x61, // Disciplined oscillator configuration
	UBX_MESSAGE_ID_CFG_DYNSEED = 0x85, // Programming the dynamic seed for the host...
	UBX_MESSAGE_ID_CFG_ESRC = 0x60, // External synchronization source configuration
	UBX_MESSAGE_ID_CFG_FIXSEED = 0x84, // Programming the fixed seed for host...
	UBX_MESSAGE_ID_CFG_GEOFENCE = 0x69, // Geofencing configuration
	UBX_MESSAGE_ID_CFG_GNSS = 0x3E, // GNSS system configuration
	UBX_MESSAGE_ID_CFG_HNR = 0x5C, // High Navigation Rate Settings
	UBX_MESSAGE_ID_CFG_INF = 0x02, // Poll configuration for one protocol
	                               // Information message configuration
	UBX_MESSAGE_ID_CFG_ITFM = 0x39, // Jamming/Interference Monitor configuration
	UBX_MESSAGE_ID_CFG_LOGFILTER = 0x47, // Data Logger Configuration
	UBX_MESSAGE_ID_CFG_MSG = 0x01, // Poll a message configuration
	                               // Set Message Rate(s)
	                               // Set Message Rate
	UBX_MESSAGE_ID_CFG_NAV5 = 0x24, // Navigation Engine Settings
	UBX_MESSAGE_ID_CFG_NAVX5 = 0x23, // Navigation Engine Expert Settings
	                                // Navigation Engine Expert Settings
	                                // Navigation Engine Expert Settings
	UBX_MESSAGE_ID_CFG_NMEA = 0x17, // NMEA protocol configuration (deprecated)
	                                // NMEA protocol configuration V0 (deprecated)
	                                // Extended NMEA protocol configuration V1
	UBX_MESSAGE_ID_CFG_ODO = 0x1E, // Odometer, Low-speed COG Engine Settings
	UBX_MESSAGE_ID_CFG_PM2 = 0x3B, // Extended Power Management configuration
	                               // Extended Power Management configuration
	                               // Extended Power Management configuration
	UBX_MESSAGE_ID_CFG_PMS = 0x86, // Power Mode Setup
	UBX_MESSAGE_ID_CFG_PRT = 0x00, // Polls the configuration for one I/O Port
	                               // Port Configuration for UART
	                               // Port Configuration for USB Port
	                               // Port Configuration for SPI Port
	                               // Port Configuration for DDC Port
	UBX_MESSAGE_ID_CFG_PWR = 0x57, // Put receiver in a defined power state.
	UBX_MESSAGE_ID_CFG_RATE = 0x08, // Navigation/Measurement Rate Settings
	UBX_MESSAGE_ID_CFG_RINV = 0x34, // Contents of Remote Inventory
	UBX_MESSAGE_ID_CFG_RST = 0x04, // Reset Receiver / Clear Backup Data Structures
	UBX_MESSAGE_ID_CFG_RXM = 0x11, // RXM configuration
	                               // RXM configuration
	UBX_MESSAGE_ID_CFG_SBAS = 0x16, // SBAS Configuration
	UBX_MESSAGE_ID_CFG_SMGR = 0x62, // Synchronization manager configuration
	UBX_MESSAGE_ID_CFG_TMODE2 = 0x3D, // Time Mode Settings 2
	UBX_MESSAGE_ID_CFG_TMODE3 = 0x71, // Time Mode Settings 3
	UBX_MESSAGE_ID_CFG_TP5 = 0x31, // Poll Time Pulse Parameters for Time Pulse 0
	                               // Poll Time Pulse Parameters
	                               // Time Pulse Parameters
	                               // Time Pulse Parameters
	UBX_MESSAGE_ID_CFG_TXSLOT = 0x53, // TX buffer time slots configuration
	UBX_MESSAGE_ID_CFG_USB = 0x1B, // USB Configuration

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
	UBX_MESSAGE_ID_MON_GNSS = 0x28, // Information message major GNSS selection
	UBX_MESSAGE_ID_MON_HW2 = 0x0B, // Extended Hardware Status
	UBX_MESSAGE_ID_MON_HW = 0x09, // Hardware Status
	UBX_MESSAGE_ID_MON_IO = 0x02, // I/O Subsystem Status
	UBX_MESSAGE_ID_MON_MSGPP = 0x06, // Message Parse and Process Status
	UBX_MESSAGE_ID_MON_PATCH = 0x27, // Poll Request for installed patches
	                                 // Output information about installed patches.
	UBX_MESSAGE_ID_MON_RXBUF = 0x07, // Receiver Buffer Status
	UBX_MESSAGE_ID_MON_RXR = 0x21, // Receiver Status Information
	UBX_MESSAGE_ID_MON_SMGR = 0x2E, // Synchronization Manager Status
	UBX_MESSAGE_ID_MON_TXBUF = 0x08, // Transmitter Buffer Status
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
	UBX_MESSAGE_ID_RXM_IMES = 0x61, // Indoor Messaging System Information
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
	UBX_MESSAGE_ID_RXM_SVSI = 0x20, // SV Status Info

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
typedef struct {
	ubx_message_class_t clsID :8; // Class ID of the (Not-)Acknowledged Message
	ubx_message_id_t msgId    :8; // Message ID of the (Not-)Acknowledged Message
} ubx_ack_ack_t;
typedef ubx_ack_ack_t ubx_ack_nack_t;
/**
 * ACK-ACK
 *
 * Message Acknowledged
 */

/** ================================================================================ */
/**
 * CFG
 */
/**
 * CFG-CFG
 *
 * Clear, Save and Load configurations
 */
typedef struct {
	uint32_t clearMask :32;
	uint32_t saveMask  :32;
	uint32_t loadMask  :32;
} ubx_cfg_cfg_t;

/**
 * CFG-MSG
 *
 * Set Message Rate(s)
 */
struct ubx_cfg_msg_rate {
	uint8_t ddc       :8;
	uint8_t uart      :8;
	uint8_t reserved1 :8;
	uint8_t usb       :8;
	uint8_t spi       :8;
	uint8_t reserved2 :8;
};
typedef struct {
	uint8_t msgClass :8; // Message class
	uint8_t msgID    :8; // Message identifier
	struct ubx_cfg_msg_rate rate; // Send rate on I/O Port (6 Ports)
} ubx_cfg_msg_t;

/**
 * CFG-PRT
 *
 * Port Configuration for UART
 */
struct ubx_cfg_prt_bit_txReady {
	bool en        :1 ; // Enable TX ready feature for this port
	bool pol       :1 ; // Polarity, 0 = High-active, 1 = Low-active
	uint8_t pin    :5 ; // PIO to be used (must not be in use already by another function)
	uint16_t thres :9 ; // Threshold
	// The given threshold is multiplied by 8 bytes.
	// The TX ready PIN goes active after >= thres*8 bytes are pending for the port and going inactive after the last
	// pending bytes have been written to hardware (0-4 bytes before end of stream).
	// 0x000 no threshold
	// 0x001 8byte
	// 0x002 16byte
	// ...
	// 0x1FE 4080byte
	// 0x1FF 4088byte
};
struct ubx_cfg_prt_bit_mode {
	uint8_t reserved1 :6 ; // Reserved
	uint8_t charLen   :2 ; // Character Length
	// 00 5bit (not supported)
	// 01 6bit (not supported)
	// 10 7bit (supported only with parity)
	// 11 8bit
	uint8_t reserved2 :1 ; // Reserved
	uint8_t parity    :3 ; // 000 Even Parity
	// 001 Odd Parity
	// 10X No Parity
	// X1X Reserved
	uint8_t nStopBits :2 ; // Number of Stop Bits
	// 00 1 Stop Bit
	// 01 1.5 Stop Bit
	// 10 2 Stop Bit
	// 11 0.5 Stop Bit
};
struct ubx_cfg_prt_bit_inProtoMask {
	bool inUbx         :1 ; // UBX protocol
	bool inNmea        :1 ; // NMEA protocol
	bool inRtcm        :1 ; // RTCM2 protocol
	uint8_t reserved1  :2 ; // Reserved
	bool inRtcm3       :1 ; // RTCM3 protocol (not supported in protocol versions less than 20)
	uint16_t reserved2 :10; // Reserved
};
struct ubx_cfg_prt_bit_outProtoMask {
	bool outUbx        :1 ; // UBX protocol
	bool outNmea       :1 ; // NMEA protocol
	uint8_t reserved1  :3 ; // Reserved
	bool outRtcm3      :1 ; // RTCM3 protocol (not supported in protocol versions less than 20)
	uint16_t reserved2 :10; // Reserved
};
struct ubx_cfg_prt_bit_flags {
	uint8_t reserved1      :1 ; // Reserved
	bool extendedTxTimeout :1 ; // Extended TX timeout: if set, the port will timeout if allocated TX memory >=4 kB and no activity for 1.5s.
	// If not set the port will timoout if no activity for 1.5s regardless on the amount of allocated TX memory
	uint16_t reserved2     :14; // Reserved
};
typedef struct {
	ubx_port_t portID       :8 ; // Port Identifier Number
	uint8_t reserved1       :8 ; // Reserved
	struct ubx_cfg_prt_bit_txReady txReady;       // TX ready PIN configuration
	struct ubx_cfg_prt_bit_mode mode;             // A bit mask describing the UART mode
	uint32_t baudRate       :32; // Baud rate (bits/s)
	struct ubx_cfg_prt_bit_inProtoMask inProtoMask;   // A mask describing which input protocols are active.
	struct ubx_cfg_prt_bit_outProtoMask outProtoMask; // A mask describing which output protocols are active.
	struct ubx_cfg_prt_bit_flags flags;               // Flags bit mask
	uint16_t reserved2      :16; // Reserved
} ubx_cfg_prt_t;

/**
 * CFG-RATE
 *
 * Navigation/Measurement Rate Settings
 */
typedef struct {
	uint16_t measRate :16; // The elapsed time between GNSS measurements (ms), which defines the rate,
 	                       // e.g. 100ms => 10Hz, 1000ms => 1Hz, 10000ms => 0.1Hz
	uint16_t navRate  :16; // The ratio between the number of measurements and the number of navigation
	                       // solutions
	uint16_t timeRef  :16; // The time system to which measurements are aligned:
	                       // 0: UTC time
	                       // 1: GPS time
	                       // 2: GLONASS time
	                       // 3: BeiDou time
	                       // 4: Galileo time
} ubx_cfg_rate_t;

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
	bool diffSOln      :1 ; // 1 = differential corrections were applied
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
	int32_t headMot    :32; // Heading of motion (2-D) (mm/s 1e-5)
	uint32_t sAcc      :32; // Speed accuracy estimate (mm/s)
	uint32_t headAcc   :32; // Heading accuracy estimate (both motion and vehicle) (deg 1e-5)
	uint16_t pDOP      :16; // Position DOP (0.01)
	uint64_t reserved2 :48; // Reserved
	int32_t headVeh    :32; // Heading of vehicle (2-D) (deg 1e-5)
	int16_t magDec     :16; // Magnetic declination (deg 1e-2)
	uint16_t magAcc    :16; // Magnetic declination accuracy (deg 1e-2)
} ubx_nav_pvt_t;

#endif //UBX_PROTOCOL_H
