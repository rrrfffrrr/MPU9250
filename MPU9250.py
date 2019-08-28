# MPU9250 library
# Original code is herer -> http://www.jkelec.co.kr/img/sensors/manual/mpu9250_gy9250/mpu9250_raw.zip

from I2CWrapper import I2CWrapper

#Magnetometer Registers
MPU9150_RA_MAG_ADDRESS = 0x0C
MPU9150_RA_MAG_XOUT_L = 0x03
MPU9150_RA_MAG_XOUT_H = 0x04
MPU9150_RA_MAG_YOUT_L = 0x05
MPU9150_RA_MAG_YOUT_H = 0x06
MPU9150_RA_MAG_ZOUT_L = 0x07
MPU9150_RA_MAG_ZOUT_H = 0x08

MPU9250_ADDRESS_AD0_LOW = 0x68 # address pin low (GND), default for InvenSense evaluation board
MPU9250_ADDRESS_AD0_HIGH = 0x69 # address pin high (VCC)
MPU9250_DEFAULT_ADDRESS = MPU9250_ADDRESS_AD0_LOW

MPU9250_RA_XG_OFFS_TC = 0x00 #[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
MPU9250_RA_YG_OFFS_TC = 0x01 #[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
MPU9250_RA_ZG_OFFS_TC = 0x02 #[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
MPU9250_RA_X_FINE_GAIN = 0x03 #[7:0] X_FINE_GAIN
MPU9250_RA_Y_FINE_GAIN = 0x04 #[7:0] Y_FINE_GAIN
MPU9250_RA_Z_FINE_GAIN = 0x05 #[7:0] Z_FINE_GAIN
MPU9250_RA_XA_OFFS_H = 0x06 #[15:0] XA_OFFS
MPU9250_RA_XA_OFFS_L_TC = 0x07
MPU9250_RA_YA_OFFS_H = 0x08 #[15:0] YA_OFFS
MPU9250_RA_YA_OFFS_L_TC = 0x09
MPU9250_RA_ZA_OFFS_H = 0x0A #[15:0] ZA_OFFS
MPU9250_RA_ZA_OFFS_L_TC = 0x0B
MPU9250_RA_XG_OFFS_USRH = 0x13 #[15:0] XG_OFFS_USR
MPU9250_RA_XG_OFFS_USRL = 0x14
MPU9250_RA_YG_OFFS_USRH = 0x15 #[15:0] YG_OFFS_USR
MPU9250_RA_YG_OFFS_USRL = 0x16
MPU9250_RA_ZG_OFFS_USRH = 0x17 #[15:0] ZG_OFFS_USR
MPU9250_RA_ZG_OFFS_USRL = 0x18
MPU9250_RA_SMPLRT_DIV = 0x19
MPU9250_RA_CONFIG = 0x1A
MPU9250_RA_GYRO_CONFIG = 0x1B
MPU9250_RA_ACCEL_CONFIG = 0x1C
MPU9250_RA_FF_THR = 0x1D
MPU9250_RA_FF_DUR = 0x1E
MPU9250_RA_MOT_THR = 0x1F
MPU9250_RA_MOT_DUR = 0x20
MPU9250_RA_ZRMOT_THR = 0x21
MPU9250_RA_ZRMOT_DUR = 0x22
MPU9250_RA_FIFO_EN = 0x23
MPU9250_RA_I2C_MST_CTRL = 0x24
MPU9250_RA_I2C_SLV0_ADDR = 0x25
MPU9250_RA_I2C_SLV0_REG = 0x26
MPU9250_RA_I2C_SLV0_CTRL = 0x27
MPU9250_RA_I2C_SLV1_ADDR = 0x28
MPU9250_RA_I2C_SLV1_REG = 0x29
MPU9250_RA_I2C_SLV1_CTRL = 0x2A
MPU9250_RA_I2C_SLV2_ADDR = 0x2B
MPU9250_RA_I2C_SLV2_REG = 0x2C
MPU9250_RA_I2C_SLV2_CTRL = 0x2D
MPU9250_RA_I2C_SLV3_ADDR = 0x2E
MPU9250_RA_I2C_SLV3_REG = 0x2F
MPU9250_RA_I2C_SLV3_CTRL = 0x30
MPU9250_RA_I2C_SLV4_ADDR = 0x31
MPU9250_RA_I2C_SLV4_REG = 0x32
MPU9250_RA_I2C_SLV4_DO = 0x33
MPU9250_RA_I2C_SLV4_CTRL = 0x34
MPU9250_RA_I2C_SLV4_DI = 0x35
MPU9250_RA_I2C_MST_STATUS = 0x36
MPU9250_RA_INT_PIN_CFG = 0x37
MPU9250_RA_INT_ENABLE = 0x38
MPU9250_RA_DMP_INT_STATUS = 0x39
MPU9250_RA_INT_STATUS = 0x3A
MPU9250_RA_ACCEL_XOUT_H = 0x3B
MPU9250_RA_ACCEL_XOUT_L = 0x3C
MPU9250_RA_ACCEL_YOUT_H = 0x3D
MPU9250_RA_ACCEL_YOUT_L = 0x3E
MPU9250_RA_ACCEL_ZOUT_H = 0x3F
MPU9250_RA_ACCEL_ZOUT_L = 0x40
MPU9250_RA_TEMP_OUT_H = 0x41
MPU9250_RA_TEMP_OUT_L = 0x42
MPU9250_RA_GYRO_XOUT_H = 0x43
MPU9250_RA_GYRO_XOUT_L = 0x44
MPU9250_RA_GYRO_YOUT_H = 0x45
MPU9250_RA_GYRO_YOUT_L = 0x46
MPU9250_RA_GYRO_ZOUT_H = 0x47
MPU9250_RA_GYRO_ZOUT_L = 0x48
MPU9250_RA_EXT_SENS_DATA_00 = 0x49
MPU9250_RA_EXT_SENS_DATA_01 = 0x4A
MPU9250_RA_EXT_SENS_DATA_02 = 0x4B
MPU9250_RA_EXT_SENS_DATA_03 = 0x4C
MPU9250_RA_EXT_SENS_DATA_04 = 0x4D
MPU9250_RA_EXT_SENS_DATA_05 = 0x4E
MPU9250_RA_EXT_SENS_DATA_06 = 0x4F
MPU9250_RA_EXT_SENS_DATA_07 = 0x50
MPU9250_RA_EXT_SENS_DATA_08 = 0x51
MPU9250_RA_EXT_SENS_DATA_09 = 0x52
MPU9250_RA_EXT_SENS_DATA_10 = 0x53
MPU9250_RA_EXT_SENS_DATA_11 = 0x54
MPU9250_RA_EXT_SENS_DATA_12 = 0x55
MPU9250_RA_EXT_SENS_DATA_13 = 0x56
MPU9250_RA_EXT_SENS_DATA_14 = 0x57
MPU9250_RA_EXT_SENS_DATA_15 = 0x58
MPU9250_RA_EXT_SENS_DATA_16 = 0x59
MPU9250_RA_EXT_SENS_DATA_17 = 0x5A
MPU9250_RA_EXT_SENS_DATA_18 = 0x5B
MPU9250_RA_EXT_SENS_DATA_19 = 0x5C
MPU9250_RA_EXT_SENS_DATA_20 = 0x5D
MPU9250_RA_EXT_SENS_DATA_21 = 0x5E
MPU9250_RA_EXT_SENS_DATA_22 = 0x5F
MPU9250_RA_EXT_SENS_DATA_23 = 0x60
MPU9250_RA_MOT_DETECT_STATUS = 0x61
MPU9250_RA_I2C_SLV0_DO = 0x63
MPU9250_RA_I2C_SLV1_DO = 0x64
MPU9250_RA_I2C_SLV2_DO = 0x65
MPU9250_RA_I2C_SLV3_DO = 0x66
MPU9250_RA_I2C_MST_DELAY_CTRL = 0x67
MPU9250_RA_SIGNAL_PATH_RESET = 0x68
MPU9250_RA_MOT_DETECT_CTRL = 0x69
MPU9250_RA_USER_CTRL = 0x6A
MPU9250_RA_PWR_MGMT_1 = 0x6B
MPU9250_RA_PWR_MGMT_2 = 0x6C
MPU9250_RA_BANK_SEL = 0x6D
MPU9250_RA_MEM_START_ADDR = 0x6E
MPU9250_RA_MEM_R_W = 0x6F
MPU9250_RA_DMP_CFG_1 = 0x70
MPU9250_RA_DMP_CFG_2 = 0x71
MPU9250_RA_FIFO_COUNTH = 0x72
MPU9250_RA_FIFO_COUNTL = 0x73
MPU9250_RA_FIFO_R_W = 0x74
MPU9250_RA_WHO_AM_I = 0x75

MPU9250_TC_PWR_MODE_BIT = 7
MPU9250_TC_OFFSET_BIT = 6
MPU9250_TC_OFFSET_LENGTH = 6
MPU9250_TC_OTP_BNK_VLD_BIT = 0

MPU9250_VDDIO_LEVEL_VLOGIC = 0
MPU9250_VDDIO_LEVEL_VDD = 1

MPU9250_CFG_EXT_SYNC_SET_BIT = 5
MPU9250_CFG_EXT_SYNC_SET_LENGTH = 3
MPU9250_CFG_DLPF_CFG_BIT = 2
MPU9250_CFG_DLPF_CFG_LENGTH = 3

MPU9250_EXT_SYNC_DISABLED = 0x0
MPU9250_EXT_SYNC_TEMP_OUT_L = 0x1
MPU9250_EXT_SYNC_GYRO_XOUT_L = 0x2
MPU9250_EXT_SYNC_GYRO_YOUT_L = 0x3
MPU9250_EXT_SYNC_GYRO_ZOUT_L = 0x4
MPU9250_EXT_SYNC_ACCEL_XOUT_L = 0x5
MPU9250_EXT_SYNC_ACCEL_YOUT_L = 0x6
MPU9250_EXT_SYNC_ACCEL_ZOUT_L = 0x7

MPU9250_DLPF_BW_256 = 0x00
MPU9250_DLPF_BW_188 = 0x01
MPU9250_DLPF_BW_98 = 0x02
MPU9250_DLPF_BW_42 = 0x03
MPU9250_DLPF_BW_20 = 0x04
MPU9250_DLPF_BW_10 = 0x05
MPU9250_DLPF_BW_5 = 0x06

MPU9250_GCONFIG_FS_SEL_BIT = 4
MPU9250_GCONFIG_FS_SEL_LENGTH = 2

MPU9250_GYRO_FS_250 = 0x00
MPU9250_GYRO_FS_500 = 0x01
MPU9250_GYRO_FS_1000 = 0x02
MPU9250_GYRO_FS_2000 = 0x03

MPU9250_ACONFIG_XA_ST_BIT = 7
MPU9250_ACONFIG_YA_ST_BIT = 6
MPU9250_ACONFIG_ZA_ST_BIT = 5
MPU9250_ACONFIG_AFS_SEL_BIT = 4
MPU9250_ACONFIG_AFS_SEL_LENGTH = 2
MPU9250_ACONFIG_ACCEL_HPF_BIT = 2
MPU9250_ACONFIG_ACCEL_HPF_LENGTH = 3

MPU9250_ACCEL_FS_2 = 0x00
MPU9250_ACCEL_FS_4 = 0x01
MPU9250_ACCEL_FS_8 = 0x02
MPU9250_ACCEL_FS_16 = 0x03

MPU9250_DHPF_RESET = 0x00
MPU9250_DHPF_5 = 0x01
MPU9250_DHPF_2P5 = 0x02
MPU9250_DHPF_1P25 = 0x03
MPU9250_DHPF_0P63 = 0x04
MPU9250_DHPF_HOLD = 0x07

MPU9250_TEMP_FIFO_EN_BIT = 7
MPU9250_XG_FIFO_EN_BIT = 6
MPU9250_YG_FIFO_EN_BIT = 5
MPU9250_ZG_FIFO_EN_BIT = 4
MPU9250_ACCEL_FIFO_EN_BIT = 3
MPU9250_SLV2_FIFO_EN_BIT = 2
MPU9250_SLV1_FIFO_EN_BIT = 1
MPU9250_SLV0_FIFO_EN_BIT = 0

MPU9250_MULT_MST_EN_BIT = 7
MPU9250_WAIT_FOR_ES_BIT = 6
MPU9250_SLV_3_FIFO_EN_BIT = 5
MPU9250_I2C_MST_P_NSR_BIT = 4
MPU9250_I2C_MST_CLK_BIT = 3
MPU9250_I2C_MST_CLK_LENGTH = 4

MPU9250_CLOCK_DIV_348 = 0x0
MPU9250_CLOCK_DIV_333 = 0x1
MPU9250_CLOCK_DIV_320 = 0x2
MPU9250_CLOCK_DIV_308 = 0x3
MPU9250_CLOCK_DIV_296 = 0x4
MPU9250_CLOCK_DIV_286 = 0x5
MPU9250_CLOCK_DIV_276 = 0x6
MPU9250_CLOCK_DIV_267 = 0x7
MPU9250_CLOCK_DIV_258 = 0x8
MPU9250_CLOCK_DIV_500 = 0x9
MPU9250_CLOCK_DIV_471 = 0xA
MPU9250_CLOCK_DIV_444 = 0xB
MPU9250_CLOCK_DIV_421 = 0xC
MPU9250_CLOCK_DIV_400 = 0xD
MPU9250_CLOCK_DIV_381 = 0xE
MPU9250_CLOCK_DIV_364 = 0xF

MPU9250_I2C_SLV_RW_BIT = 7
MPU9250_I2C_SLV_ADDR_BIT = 6
MPU9250_I2C_SLV_ADDR_LENGTH = 7
MPU9250_I2C_SLV_EN_BIT = 7
MPU9250_I2C_SLV_BYTE_SW_BIT = 6
MPU9250_I2C_SLV_REG_DIS_BIT = 5
MPU9250_I2C_SLV_GRP_BIT = 4
MPU9250_I2C_SLV_LEN_BIT = 3
MPU9250_I2C_SLV_LEN_LENGTH = 4

MPU9250_I2C_SLV4_RW_BIT = 7
MPU9250_I2C_SLV4_ADDR_BIT = 6
MPU9250_I2C_SLV4_ADDR_LENGTH = 7
MPU9250_I2C_SLV4_EN_BIT = 7
MPU9250_I2C_SLV4_INT_EN_BIT = 6
MPU9250_I2C_SLV4_REG_DIS_BIT = 5
MPU9250_I2C_SLV4_MST_DLY_BIT = 4
MPU9250_I2C_SLV4_MST_DLY_LENGTH = 5

MPU9250_MST_PASS_THROUGH_BIT = 7
MPU9250_MST_I2C_SLV4_DONE_BIT = 6
MPU9250_MST_I2C_LOST_ARB_BIT = 5
MPU9250_MST_I2C_SLV4_NACK_BIT = 4
MPU9250_MST_I2C_SLV3_NACK_BIT = 3
MPU9250_MST_I2C_SLV2_NACK_BIT = 2
MPU9250_MST_I2C_SLV1_NACK_BIT = 1
MPU9250_MST_I2C_SLV0_NACK_BIT = 0

MPU9250_INTCFG_INT_LEVEL_BIT = 7
MPU9250_INTCFG_INT_OPEN_BIT = 6
MPU9250_INTCFG_LATCH_INT_EN_BIT = 5
MPU9250_INTCFG_INT_RD_CLEAR_BIT = 4
MPU9250_INTCFG_FSYNC_INT_LEVEL_BIT = 3
MPU9250_INTCFG_FSYNC_INT_EN_BIT = 2
MPU9250_INTCFG_I2C_BYPASS_EN_BIT = 1
MPU9250_INTCFG_CLKOUT_EN_BIT = 0

MPU9250_INTMODE_ACTIVEHIGH = 0x00
MPU9250_INTMODE_ACTIVELOW = 0x01

MPU9250_INTDRV_PUSHPULL = 0x00
MPU9250_INTDRV_OPENDRAIN = 0x01

MPU9250_INTLATCH_50USPULSE = 0x00
MPU9250_INTLATCH_WAITCLEAR = 0x01

MPU9250_INTCLEAR_STATUSREAD = 0x00
MPU9250_INTCLEAR_ANYREAD = 0x01

MPU9250_INTERRUPT_FF_BIT = 7
MPU9250_INTERRUPT_MOT_BIT = 6
MPU9250_INTERRUPT_ZMOT_BIT = 5
MPU9250_INTERRUPT_FIFO_OFLOW_BIT = 4
MPU9250_INTERRUPT_I2C_MST_INT_BIT = 3
MPU9250_INTERRUPT_PLL_RDY_INT_BIT = 2
MPU9250_INTERRUPT_DMP_INT_BIT = 1
MPU9250_INTERRUPT_DATA_RDY_BIT = 0

# TODO: figure out what these actually do
# UMPL source code is not very obivous
MPU9250_DMPINT_5_BIT = 5
MPU9250_DMPINT_4_BIT = 4
MPU9250_DMPINT_3_BIT = 3
MPU9250_DMPINT_2_BIT = 2
MPU9250_DMPINT_1_BIT = 1
MPU9250_DMPINT_0_BIT = 0

MPU9250_MOTION_MOT_XNEG_BIT = 7
MPU9250_MOTION_MOT_XPOS_BIT = 6
MPU9250_MOTION_MOT_YNEG_BIT = 5
MPU9250_MOTION_MOT_YPOS_BIT = 4
MPU9250_MOTION_MOT_ZNEG_BIT = 3
MPU9250_MOTION_MOT_ZPOS_BIT = 2
MPU9250_MOTION_MOT_ZRMOT_BIT = 0

MPU9250_DELAYCTRL_DELAY_ES_SHADOW_BIT = 7
MPU9250_DELAYCTRL_I2C_SLV4_DLY_EN_BIT = 4
MPU9250_DELAYCTRL_I2C_SLV3_DLY_EN_BIT = 3
MPU9250_DELAYCTRL_I2C_SLV2_DLY_EN_BIT = 2
MPU9250_DELAYCTRL_I2C_SLV1_DLY_EN_BIT = 1
MPU9250_DELAYCTRL_I2C_SLV0_DLY_EN_BIT = 0

MPU9250_PATHRESET_GYRO_RESET_BIT = 2
MPU9250_PATHRESET_ACCEL_RESET_BIT = 1
MPU9250_PATHRESET_TEMP_RESET_BIT = 0

MPU9250_DETECT_ACCEL_ON_DELAY_BIT = 5
MPU9250_DETECT_ACCEL_ON_DELAY_LENGTH = 2
MPU9250_DETECT_FF_COUNT_BIT = 3
MPU9250_DETECT_FF_COUNT_LENGTH = 2
MPU9250_DETECT_MOT_COUNT_BIT = 1
MPU9250_DETECT_MOT_COUNT_LENGTH = 2

MPU9250_DETECT_DECREMENT_RESET = 0x0
MPU9250_DETECT_DECREMENT_1 = 0x1
MPU9250_DETECT_DECREMENT_2 = 0x2
MPU9250_DETECT_DECREMENT_4 = 0x3

MPU9250_USERCTRL_DMP_EN_BIT = 7
MPU9250_USERCTRL_FIFO_EN_BIT = 6
MPU9250_USERCTRL_I2C_MST_EN_BIT = 5
MPU9250_USERCTRL_I2C_IF_DIS_BIT = 4
MPU9250_USERCTRL_DMP_RESET_BIT = 3
MPU9250_USERCTRL_FIFO_RESET_BIT = 2
MPU9250_USERCTRL_I2C_MST_RESET_BIT = 1
MPU9250_USERCTRL_SIG_COND_RESET_BIT = 0

MPU9250_PWR1_DEVICE_RESET_BIT = 7
MPU9250_PWR1_SLEEP_BIT = 6
MPU9250_PWR1_CYCLE_BIT = 5
MPU9250_PWR1_TEMP_DIS_BIT = 3
MPU9250_PWR1_CLKSEL_BIT = 2
MPU9250_PWR1_CLKSEL_LENGTH = 3

MPU9250_CLOCK_INTERNAL = 0x00
MPU9250_CLOCK_PLL_XGYRO = 0x01
MPU9250_CLOCK_PLL_YGYRO = 0x02
MPU9250_CLOCK_PLL_ZGYRO = 0x03
MPU9250_CLOCK_PLL_EXT32K = 0x04
MPU9250_CLOCK_PLL_EXT19M = 0x05
MPU9250_CLOCK_KEEP_RESET = 0x07

MPU9250_PWR2_LP_WAKE_CTRL_BIT = 7
MPU9250_PWR2_LP_WAKE_CTRL_LENGTH = 2
MPU9250_PWR2_STBY_XA_BIT = 5
MPU9250_PWR2_STBY_YA_BIT = 4
MPU9250_PWR2_STBY_ZA_BIT = 3
MPU9250_PWR2_STBY_XG_BIT = 2
MPU9250_PWR2_STBY_YG_BIT = 1
MPU9250_PWR2_STBY_ZG_BIT = 0

MPU9250_WAKE_FREQ_1P25 = 0x0
MPU9250_WAKE_FREQ_2P5 = 0x1
MPU9250_WAKE_FREQ_5 = 0x2
MPU9250_WAKE_FREQ_10 = 0x3

MPU9250_BANKSEL_PRFTCH_EN_BIT = 6
MPU9250_BANKSEL_CFG_USER_BANK_BIT = 5
MPU9250_BANKSEL_MEM_SEL_BIT = 4
MPU9250_BANKSEL_MEM_SEL_LENGTH = 5

MPU9250_WHO_AM_I_BIT = 6
MPU9250_WHO_AM_I_LENGTH = 8

MPU9250_DMP_MEMORY_BANKS = 8
MPU9250_DMP_MEMORY_BANK_SIZE = 256
MPU9250_DMP_MEMORY_CHUNK_SIZE = 16

class MPU9250:
	## Specific address constructor.
	# @param i2c_manager instance of I2CWrapper that connected to I2C
	# @param address I2C address
	# @see MPU9250_DEFAULT_ADDRESS
	# @see MPU9250_ADDRESS_AD0_LOW
	# @see MPU9250_ADDRESS_AD0_HIGH
	def __init__(self, i2c_manager, address = MPU9250_DEFAULT_ADDRESS):
		if not isinstance(i2c_manager, I2CWrapper):
			raise TypeError("i2c_manager must be instance of 'I2CWrapper' class");

		self.__devAddr = address;
		self.__i2cWrapper = i2c_manager;

	## Power on and prepare for general usage.
	# This will activate the device and take it out of sleep mode (which must be done
	# after start-up). This function also sets both the accelerometer and the gyroscope
	# to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
	# the clock source to use the X Gyro for reference, which is slightly better than
	# the default internal clock source.
	def initialize(self):
		self.setClockSource(MPU9250_CLOCK_PLL_XGYRO);
		self.setFullScaleGyroRange(MPU9250_GYRO_FS_250);
		self.setFullScaleAccelRange(MPU9250_ACCEL_FS_2);
		self.setSleepEnabled(False);


	## Verify the I2C connection.
	# Make sure the device is connected and responds as expected.
	# @return True if connection is valid, False otherwise
	def testConnection(self):
		return self.getDeviceID() == 0x71;

	## AUX_VDDIO register
	# Get the auxiliary I2C supply voltage level.
	# When set to 1, the auxiliary I2C bus high logic level is VDD. When cleared to
	# 0, the auxiliary I2C bus high logic level is VLOGIC. This does not apply to
	# the MPU-6000, which does not have a VLOGIC pin.
	# @return I2C supply voltage level (0 = VLOGIC, 1 = VDD)
	def getAuxVDDIOLevel(self):
		return self.__i2cWrapper.readBits(self.__devAddr, MPU9250_RA_YG_OFFS_TC, MPU9250_TC_PWR_MODE_BIT, 1)[0];

	# Set the auxiliary I2C supply voltage level.
	# When set to 1, the auxiliary I2C bus high logic level is VDD. When cleared to
	# 0, the auxiliary I2C bus high logic level is VLOGIC. This does not apply to
	# the MPU-6000, which does not have a VLOGIC pin.
	# @param level I2C supply voltage level (0 = VLOGIC, 1 = VDD)
	def setAuxVDDIOLevel(self, level):
		self.__i2cWrapper.writeBit(self.__devAddr, MPU9250_RA_YG_OFFS_TC, MPU9250_TC_PWR_MODE_BIT, level);

	## SMPLRT_DIV register
	# Get gyroscope output rate divider.
	# The sensor register output, FIFO output, DMP sampling, Motion detection, Zero
	# Motion detection, and Free Fall detection are all based on the Sample Rate.
	# The Sample Rate is generated by dividing the gyroscope output rate by
	# SMPLRT_DIV:
	#
	# Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	#
	# where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or
	# 7), and 1kHz when the DLPF is enabled (see Register 26).
	#
	# Note: The accelerometer output rate is 1kHz. This means that for a Sample
	# Rate greater than 1kHz, the same accelerometer sample may be output to the
	# FIFO, DMP, and sensor registers more than once.
	#
	# For a diagram of the gyroscope and accelerometer signal paths, see Section 8
	# of the MPU-6000/MPU-9250 Product Specification document.
	#
	# @return Current sample rate
	# @see MPU9250_RA_SMPLRT_DIV
	def getRate(self):
		return self.__i2cWrapper.readBytes(self.__devAddr, MPU9250_RA_SMPLRT_DIV, 1)[0];

	# Set gyroscope sample rate divider.
	# @param rate New sample rate divider
	# @see getRate()
	# @see MPU9250_RA_SMPLRT_DIV
	def setRate(self, rate):
		self.__i2cWrapper.writeByte(self.__devAddr, MPU9250_RA_SMPLRT_DIV, rate);

	## CONFIG register
	# Get external FSYNC configuration.
	# Configures the external Frame Synchronization (FSYNC) pin sampling. An
	# external signal connected to the FSYNC pin can be sampled by configuring
	# EXT_SYNC_SET. Signal changes to the FSYNC pin are latched so that short
	# strobes may be captured. The latched FSYNC signal will be sampled at the
	# Sampling Rate, as defined in register 25. After sampling, the latch will
	# reset to the current FSYNC signal state.
	#
	# The sampled value will be reported in place of the least significant bit in
	# a sensor data register determined by the value of EXT_SYNC_SET according to
	# the following table.
	#
	# <pre>
	# EXT_SYNC_SET | bit | FSYNC Bit Location
	# -------------+-------------------
	# 0            | 000 | Input disabled
	# 1            | 001 | TEMP_OUT_L[0]
	# 2            | 010 | GYRO_XOUT_L[0]
	# 3            | 011 | GYRO_YOUT_L[0]
	# 4            | 100 | GYRO_ZOUT_L[0]
	# 5            | 101 | ACCEL_XOUT_L[0]
	# 6            | 110 | ACCEL_YOUT_L[0]
	# 7            | 111 | ACCEL_ZOUT_L[0]
	# </pre>
	#
	# @return FSYNC configuration value
	def getExternalFrameSync(self):
		return self.__i2cWrapper.readBits(self.__devAddr, MPU9250_RA_CONFIG, MPU9250_CFG_EXT_SYNC_SET_BIT, MPU9250_CFG_EXT_SYNC_SET_LENGTH);

	# Set external FSYNC configuration.
	# @see getExternalFrameSync()
	# @see MPU9250_RA_CONFIG
	# @param sync New FSYNC configuration value
	def setExternalFrameSync(self, sync):
		if len(sync) != MPU9250_CFG_EXT_SYNC_SET_LENGTH:
			raise ValueError("MPU9250_CFG_EXT_SYNC_SET_LENGTH must be " + str(MPU9250_CFG_EXT_SYNC_SET_LENGTH));
		self.__i2cWrapper.writeBits(self.__devAddr, MPU9250_RA_CONFIG, MPU9250_CFG_EXT_SYNC_SET_BIT, sync);

	## Get digital low-pass filter configuration.
	# The DLPF_CFG parameter sets the digital low pass filter configuration. It
	# also determines the internal sampling rate used by the device as shown in
	# the table below.
	#
	# Note: The accelerometer output rate is 1kHz. This means that for a Sample
	# Rate greater than 1kHz, the same accelerometer sample may be output to the
	# FIFO, DMP, and sensor registers more than once.
	#
	# <pre>
	#          |   ACCELEROMETER    |           GYROSCOPE
	# DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
	# ---------+-----------+--------+-----------+--------+-------------
	# 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
	# 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
	# 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
	# 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
	# 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
	# 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
	# 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
	# 7        |   -- Reserved --   |   -- Reserved --   | Reserved
	# </pre>
	#
	# @return DLFP configuration
	# @see MPU9250_RA_CONFIG
	# @see MPU9250_CFG_DLPF_CFG_BIT
	# @see MPU9250_CFG_DLPF_CFG_LENGTH
	def getDLPFMode(self):
		return self.__i2cWrapper.readBits(self.__devAddr, MPU9250_RA_CONFIG, MPU9250_CFG_DLPF_CFG_BIT, MPU9250_CFG_DLPF_CFG_LENGTH);

	# Set digital low-pass filter configuration.
	# @param mode New DLFP configuration setting
	# @see getDLPFBandwidth()
	# @see MPU9250_DLPF_BW_256
	# @see MPU9250_RA_CONFIG
	# @see MPU9250_CFG_DLPF_CFG_BIT
	# @see MPU9250_CFG_DLPF_CFG_LENGTH
	def setDLPFMode(self, bandwidth):
		if len(bandwidth) != MPU9250_CFG_DLPF_CFG_LENGTH:
			raise ValueError("MPU9250_CFG_DLPF_CFG_LENGTH must be " + str(MPU9250_CFG_DLPF_CFG_LENGTH));
		self.__i2cWrapper.writeBits(self.__devAddr, MPU9250_RA_CONFIG, MPU9250_CFG_DLPF_CFG_BIT, sync);

	## GYRO_CONFIG register
	# Get full-scale gyroscope range.
	# The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
	# as described in the table below.
	#
	# <pre>
	# 0 = +/- 250 degrees/sec
	# 1 = +/- 500 degrees/sec
	# 2 = +/- 1000 degrees/sec
	# 3 = +/- 2000 degrees/sec
	# </pre>
	#
	# @return Current full-scale gyroscope range setting
	# @see MPU9250_GYRO_FS_250
	# @see MPU9250_RA_GYRO_CONFIG
	# @see MPU9250_GCONFIG_FS_SEL_BIT
	# @see MPU9250_GCONFIG_FS_SEL_LENGTH
	def getFullScaleGyroRange(self):
    	return self.__i2cWrapper.readBits(self.__devAddr, MPU9250_RA_GYRO_CONFIG, MPU9250_GCONFIG_FS_SEL_BIT, MPU9250_GCONFIG_FS_SEL_LENGTH);

	# Set full-scale gyroscope range.
	# @param range New full-scale gyroscope range value
	# @see getFullScaleRange()
	# @see MPU9250_GYRO_FS_250
	# @see MPU9250_RA_GYRO_CONFIG
	# @see MPU9250_GCONFIG_FS_SEL_BIT
	# @see MPU9250_GCONFIG_FS_SEL_LENGTH
	def setFullScaleGyroRange(self, range):
		if len(range) != MPU9250_GCONFIG_FS_SEL_LENGTH:
			raise ValueError("MPU9250_GCONFIG_FS_SEL_LENGTH must be " + str(MPU9250_GCONFIG_FS_SEL_LENGTH));
		self.__i2cWrapper.writeBits(self.__devAddr, MPU9250_RA_GYRO_CONFIG, MPU9250_GCONFIG_FS_SEL_BIT, range);

	## ACCEL_CONFIG register
	def getAccelXSelfTest(self):
		pass
	def setAccelXSelfTest(self, enabled):
		pass
	def getAccelYSelfTest(self):
		pass
	def setAccelYSelfTest(self, enabled):
		pass
	def getAccelZSelfTest(self):
		pass
	def setAccelZSelfTest(self, enabled):
		pass
	def getFullScaleAccelRange(self):
		pass
	def setFullScaleAccelRange(self, range):
		pass
	def getDHPFMode(self):
		pass
	def setDHPFMode(self, mode):
		pass

	# FF_THR register
	def getFreefallDetectionThreshold(self):
		pass
	def setFreefallDetectionThreshold(self, threshold):
		pass

	# FF_DUR register
	def getFreefallDetectionDuration(self):
		pass
	def setFreefallDetectionDuration(self, duration):
		pass

	# MOT_THR register
	def getMotionDetectionThreshold(self):
		pass
	def setMotionDetectionThreshold(self, threshold):
		pass

	# MOT_DUR register
	def getMotionDetectionDuration(self):
		pass
	def setMotionDetectionDuration(self, duration):
		pass

	# ZRMOT_THR register
	def getZeroMotionDetectionThreshold(self):
		pass
	def setZeroMotionDetectionThreshold(self, threshold):
		pass

	# ZRMOT_DUR register
	def getZeroMotionDetectionDuration(self):
		pass
	def setZeroMotionDetectionDuration(self, duration):
		pass

	# FIFO_EN register
	def getTempFIFOEnabled(self):
		pass
	def setTempFIFOEnabled(self, enabled):
		pass
	def getXGyroFIFOEnabled(self):
		pass
	def setXGyroFIFOEnabled(self, enabled):
		pass
	def getYGyroFIFOEnabled(self):
		pass
	def setYGyroFIFOEnabled(self, enabled):
		pass
	def getZGyroFIFOEnabled(self):
		pass
	def setZGyroFIFOEnabled(self, enabled):
		pass
	def getAccelFIFOEnabled(self):
		pass
	def setAccelFIFOEnabled(self, enabled):
		pass
	def getSlave2FIFOEnabled(self):
		pass
	def setSlave2FIFOEnabled(self, enabled):
		pass
	def getSlave1FIFOEnabled(self):
		pass
	def setSlave1FIFOEnabled(self, enabled):
		pass
	def getSlave0FIFOEnabled(self):
		pass
	def setSlave0FIFOEnabled(self, enabled):
		pass

	# I2C_MST_CTRL register
	def getMultiMasterEnabled(self):
		pass
	def setMultiMasterEnabled(self, enabled):
		pass
	def getWaitForExternalSensorEnabled(self):
		pass
	def setWaitForExternalSensorEnabled(self, enabled):
		pass
	def getSlave3FIFOEnabled(self):
		pass
	def setSlave3FIFOEnabled(self, enabled):
		pass
	def getSlaveReadWriteTransitionEnabled(self):
		pass
	def setSlaveReadWriteTransitionEnabled(self, enabled):
		pass
	def getMasterClockSpeed(self):
		pass
	def setMasterClockSpeed(self, speed):
		pass

	# I2C_SLV* registers (Slave 0-3)
	def getSlaveAddress(self, num):
		pass
	def setSlaveAddress(self, num, address):
		pass
	def getSlaveRegister(self, num):
		pass
	def setSlaveRegister(self, num, reg):
		pass
	def getSlaveEnabled(self, num):
		pass
	def setSlaveEnabled(self, num, enabled):
		pass
	def getSlaveWordByteSwap(self, num):
		pass
	def setSlaveWordByteSwap(self, num, enabled):
		pass
	def getSlaveWriteMode(self, num):
		pass
	def setSlaveWriteMode(self, num, mode):
		pass
	def getSlaveWordGroupOffset(self, num):
		pass
	def setSlaveWordGroupOffset(self, num, enabled):
		pass
	def getSlaveDataLength(self, num):
		pass
	def setSlaveDataLength(self, num, length):
		pass

	# I2C_SLV* registers (Slave 4)
	def getSlave4Address(self):
		pass
	def setSlave4Address(self, address):
		pass
	def getSlave4Register(self):
		pass
	def setSlave4Register(self, reg):
		pass
	def setSlave4OutputByte(self, data):
		pass
	def getSlave4Enabled(self):
		pass
	def setSlave4Enabled(self, enabled):
		pass
	def getSlave4InterruptEnabled(self):
		pass
	def setSlave4InterruptEnabled(self, enabled):
		pass
	def getSlave4WriteMode(self):
		pass
	def setSlave4WriteMode(self, mode):
		pass
	def getSlave4MasterDelay(self):
		pass
	def setSlave4MasterDelay(self, delay):
		pass
	def getSlate4InputByte(self):
		pass

	# I2C_MST_STATUS register
	def getPassthroughStatus(self):
		pass
	def getSlave4IsDone(self):
		pass
	def getLostArbitration(self):
		pass
	def getSlave4Nack(self):
		pass
	def getSlave3Nack(self):
		pass
	def getSlave2Nack(self):
		pass
	def getSlave1Nack(self):
		pass
	def getSlave0Nack(self):
		pass

	# INT_PIN_CFG register
	def getInterruptMode(self):
		pass
	def setInterruptMode(self, mode):
		pass
	def getInterruptDrive(self):
		pass
	def setInterruptDrive(self, drive):
		pass
	def getInterruptLatch(self):
		pass
	def setInterruptLatch(self, latch):
		pass
	def getInterruptLatchClear(self):
		pass
	def setInterruptLatchClear(self, clear):
		pass
	def getFSyncInterruptLevel(self):
		pass
	def setFSyncInterruptLevel(self, level):
		pass
	def getFSyncInterruptEnabled(self):
		pass
	def setFSyncInterruptEnabled(self, enabled):
		pass
	def getI2CBypassEnabled(self):
		pass
	def setI2CBypassEnabled(self, enabled):
		pass
	def getClockOutputEnabled(self):
		pass
	def setClockOutputEnabled(self, enabled):
		pass

	# INT_ENABLE register
	def getIntEnabled(self):
		pass
	def setIntEnabled(self, enabled):
		pass
	def getIntFreefallEnabled(self):
		pass
	def setIntFreefallEnabled(self, enabled):
		pass
	def getIntMotionEnabled(self):
		pass
	def setIntMotionEnabled(self, enabled):
		pass
	def getIntZeroMotionEnabled(self):
		pass
	def setIntZeroMotionEnabled(self, enabled):
		pass
	def getIntFIFOBufferOverflowEnabled(self):
		pass
	def setIntFIFOBufferOverflowEnabled(self, enabled):
		pass
	def getIntI2CMasterEnabled(self):
		pass
	def setIntI2CMasterEnabled(self, enabled):
		pass
	def getIntDataReadyEnabled(self):
		pass
	def setIntDataReadyEnabled(self, enabled):
		pass

	# INT_STATUS register
	def getIntStatus(self):
		pass
	def getIntFreefallStatus(self):
		pass
	def getIntMotionStatus(self):
		pass
	def getIntZeroMotionStatus(self):
		pass
	def getIntFIFOBufferOverflowStatus(self):
		pass
	def getIntI2CMasterStatus(self):
		pass
	def getIntDataReadyStatus(self):
		pass

	# ACCEL_*OUT_* registers
	def getMotion9(self, ax, ay, az, gx, gy, gz, mx, my, mz):
		pass
	def getMotion6(self, ax, ay, az, gx, gy, gz):
		pass
	def getAcceleration(self, x, y, z):
		pass
	def getAccelerationX(self):
		pass
	def getAccelerationY(self):
		pass
	def getAccelerationZ(self):
		pass

	# TEMP_OUT_* registers
	def getTemperature(self):
		pass

	# GYRO_*OUT_* registers
	def getRotation(self, x, y, z):
		pass
	def getRotationX(self):
		pass
	def getRotationY(self):
		pass
	def getRotationZ(self):
		pass

	# EXT_SENS_DATA_* registers
	def getExternalSensorByte(self, position):
		pass
	def getExternalSensorWord(self, position):
		pass
	def getExternalSensorDWord(self, position):
		pass

	# MOT_DETECT_STATUS register
	def getXNegMotionDetected(self):
		pass
	def getXPosMotionDetected(self):
		pass
	def getYNegMotionDetected(self):
		pass
	def getYPosMotionDetected(self):
		pass
	def getZNegMotionDetected(self):
		pass
	def getZPosMotionDetected(self):
		pass
	def getZeroMotionDetected(self):
		pass

	# I2C_SLV*_DO register
	def setSlaveOutputByte(self, num, data):
		pass

	# I2C_MST_DELAY_CTRL register
	def getExternalShadowDelayEnabled(self):
		pass
	def setExternalShadowDelayEnabled(self, enabled):
		pass
	def getSlaveDelayEnabled(self, num):
		pass
	def setSlaveDelayEnabled(self, num, enabled):
		pass

	# SIGNAL_PATH_RESET register
	def resetGyroscopePath(self):
		pass
	def resetAccelerometerPath(self):
		pass
	def resetTemperaturePath(self):
		pass

	# MOT_DETECT_CTRL register
	def getAccelerometerPowerOnDelay(self):
		pass
	def setAccelerometerPowerOnDelay(self, delay):
		pass
	def getFreefallDetectionCounterDecrement(self):
		pass
	def setFreefallDetectionCounterDecrement(self, decrement):
		pass
	def getMotionDetectionCounterDecrement(self):
		pass
	def setMotionDetectionCounterDecrement(self, decrement):
		pass

	# USER_CTRL register
	def getFIFOEnabled(self):
		pass
	def setFIFOEnabled(self, enabled):
		pass
	def getI2CMasterModeEnabled(self):
		pass
	def setI2CMasterModeEnabled(self, enabled):
		pass
	def switchSPIEnabled(self, enabled):
		pass
	def resetFIFO(self):
		pass
	def resetI2CMaster(self):
		pass
	def resetSensors(self):
		pass

	# PWR_MGMT_1 register
	def reset(self):
		pass
	def getSleepEnabled(self):
		pass
	def setSleepEnabled(self, enabled):
		pass
	def getWakeCycleEnabled(self):
		pass
	def setWakeCycleEnabled(self, enabled):
		pass
	def getTempSensorEnabled(self):
		pass
	def setTempSensorEnabled(self, enabled):
		pass
	def getClockSource(self):
		pass
	def setClockSource(self, source):
		pass

	# PWR_MGMT_2 register
	def getWakeFrequency(self):
		pass
	def setWakeFrequency(self, frequency):
		pass
	def getStandbyXAccelEnabled(self):
		pass
	def setStandbyXAccelEnabled(self, enabled):
		pass
	def getStandbyYAccelEnabled(self):
		pass
	def setStandbyYAccelEnabled(self, enabled):
		pass
	def getStandbyZAccelEnabled(self):
		pass
	def setStandbyZAccelEnabled(self, enabled):
		pass
	def getStandbyXGyroEnabled(self):
		pass
	def setStandbyXGyroEnabled(self, enabled):
		pass
	def getStandbyYGyroEnabled(self):
		pass
	def setStandbyYGyroEnabled(self, enabled):
		pass
	def getStandbyZGyroEnabled(self):
		pass
	def setStandbyZGyroEnabled(self, enabled):
		pass

	# FIFO_COUNT_* registers
	def getFIFOCount(self):
		pass

	# FIFO_R_W register
	def getFIFOByte(self):
		pass
	def setFIFOByte(self, data):
		pass
	def getFIFOBytes(self, data, length):
		pass

	# WHO_AM_I register
	def getDeviceID(self):
		pass
	def setDeviceID(self, id):
		pass

	# ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========

	# XG_OFFS_TC register
	def getOTPBankValid(self):
		pass
	def setOTPBankValid(self, enabled):
		pass
	def getXGyroOffset(self):
		pass
	def setXGyroOffset(self, offset):
		pass

	# YG_OFFS_TC register
	def getYGyroOffset(self):
		pass
	def setYGyroOffset(self, offset):
		pass

	# ZG_OFFS_TC register
	def getZGyroOffset(self):
		pass
	def setZGyroOffset(self, offset):
		pass

	# X_FINE_GAIN register
	def getXFineGain(self):
		pass
	def setXFineGain(self, gain):
		pass

	# Y_FINE_GAIN register
	def getYFineGain(self):
		pass
	def setYFineGain(self, gain):
		pass

	# Z_FINE_GAIN register
	def getZFineGain(self):
		pass
	def setZFineGain(self, gain):
		pass

	# XA_OFFS_* registers
	def getXAccelOffset(self):
		pass
	def setXAccelOffset(self, offset):
		pass

	# YA_OFFS_* register
	def getYAccelOffset(self):
		pass
	def setYAccelOffset(self, offset):
		pass

	# ZA_OFFS_* register
	def getZAccelOffset(self):
		pass
	def setZAccelOffset(self, offset):
		pass

	# XG_OFFS_USR* registers
	def getXGyroOffsetUser(self):
		pass
	def setXGyroOffsetUser(self, offset):
		pass

	# YG_OFFS_USR* register
	def getYGyroOffsetUser(self):
		pass
	def setYGyroOffsetUser(self, offset):
		pass

	# ZG_OFFS_USR* register
	def getZGyroOffsetUser(self):
		pass
	def setZGyroOffsetUser(self, offset):
		pass

	# INT_ENABLE register (DMP functions)
	def getIntPLLReadyEnabled(self):
		pass
	def setIntPLLReadyEnabled(self, enabled):
		pass
	def getIntDMPEnabled(self):
		pass
	def setIntDMPEnabled(self, enabled):
		pass

	# DMP_INT_STATUS
	def getDMPInt5Status(self):
		pass
	def getDMPInt4Status(self):
		pass
	def getDMPInt3Status(self):
		pass
	def getDMPInt2Status(self):
		pass
	def getDMPInt1Status(self):
		pass
	def getDMPInt0Status(self):
		pass

	# INT_STATUS register (DMP functions)
	def getIntPLLReadyStatus(self):
		pass
	def getIntDMPStatus(self):
		pass

	# USER_CTRL register (DMP functions)
	def getDMPEnabled(self):
		pass
	def setDMPEnabled(self, enabled):
		pass
	def resetDMP(self):
		pass

	# BANK_SEL register
	def setMemoryBank(self, bank, prefetchEnabled = False, userBank = False):
		pass

	# MEM_START_ADDR register
	def setMemoryStartAddress(self, address):
		pass

	# MEM_R_W register
	def readMemoryByte(self):
		pass
	def writeMemoryByte(self, data):
		pass
	def readMemoryBlock(self, data, dataSize, bank = 0, address = 0):
		pass
	def writeMemoryBlock(self, data, dataSize, bank = 0, address = 0, verify = True, useProgMem = False):
		pass
	def writeProgMemoryBlock(self, data, dataSize, bank = 0, address = 0, verify = True):
		pass

	def writeDMPConfigurationSet(self, data, dataSize, useProgMem = False):
		pass
	def writeProgDMPConfigurationSet(self, data, dataSize):
		pass

	# DMP_CFG_1 register
	def getDMPConfig1(self):
		pass
	def setDMPConfig1(self, config):
		pass

	# DMP_CFG_2 register
	def getDMPConfig2(self):
		pass
	def setDMPConfig2(self, config):
		pass

	## DMP functions
	# special methods for MotionApps 2.0 implementation
	#ifdef MPU9250_INCLUDE_DMP_MOTIONAPPS20
	#def *dmpPacketBuffer;
	#def dmpPacketSize;

	#def dmpInitialize(self):
	#	pass
	#def dmpPacketAvailable(self):
	#	pass

	#def dmpSetFIFORate(self, fifoRate):
	#	pass
	#def dmpGetFIFORate(self):
	#	pass
	#def dmpGetSampleStepSizeMS(self):
	#	pass
	#def dmpGetSampleFrequency(self):
	#	pass
	#def dmpDecodeTemperature(self, tempReg):
	#	pass

	## Register callbacks after a packet of FIFO data is processed
	##dmpRegisterFIFORateProcess(inv_obj_func func, priority):
	#	pass
	##dmpUnregisterFIFORateProcess(inv_obj_func func):
	#	pass
	#def dmpRunFIFORateProcesses(self):
	#	pass

	## Setup FIFO for various output
	#def dmpSendQuaternion(self, accuracy):
	#	pass
	#def dmpSendGyro(self, elements, accuracy):
	#	pass
	#def dmpSendAccel(self, elements, accuracy):
	#	pass
	#def dmpSendLinearAccel(self, elements, accuracy):
	#	pass
	#def dmpSendLinearAccelInWorld(self, elements, accuracy):
	#	pass
	#def dmpSendControlData(self, elements, accuracy):
	#	pass
	#def dmpSendSensorData(self, elements, accuracy):
	#	pass
	#def dmpSendExternalSensorData(self, elements, accuracy):
	#	pass
	#def dmpSendGravity(self, elements, accuracy):
	#	pass
	#def dmpSendPacketNumber(self, accuracy):
	#	pass
	#def dmpSendQuantizedAccel(self, elements, accuracy):
	#	pass
	#def dmpSendEIS(self, elements, accuracy):
	#	pass

	## Get Fixed Point data from FIFO
	#def dmpGetAccel(self, data, packet = 0):
	#	pass
	#def dmpGetAccel(self, data, packet = 0):
	#	pass
	#def dmpGetAccel(self, v, packet = 0):
	#	pass
	#def dmpGetQuaternion(self, data, packet = 0):
	#	pass
	#def dmpGetQuaternion(self, data, packet = 0):
	#	pass
	#def dmpGetQuaternion(self, q, packet = 0):
	#	pass
	#def dmpGet6AxisQuaternion(self, data, packet = 0):
	#	pass
	#def dmpGet6AxisQuaternion(self, data, packet = 0):
	#	pass
	#def dmpGet6AxisQuaternion(self, q, packet = 0):
	#	pass
	#def dmpGetRelativeQuaternion(self, data, packet = 0):
	#	pass
	#def dmpGetRelativeQuaternion(self, data, packet = 0):
	#	pass
	#def dmpGetRelativeQuaternion(self, data, packet = 0):
	#	pass
	#def dmpGetGyro(self, data, packet = 0):
	#	pass
	#def dmpGetGyro(self, data, packet = 0):
	#	pass
	#def dmpGetGyro(self, v, packet = 0):
	#	pass
	#def dmpSetLinearAccelFilterCoefficient(self, coef):
	#	pass
	#def dmpGetLinearAccel(self, data, packet = 0):
	#	pass
	#def dmpGetLinearAccel(self, data, packet = 0):
	#	pass
	#def dmpGetLinearAccel(self, v, packet = 0):
	#	pass
	#def dmpGetLinearAccel(self, v, vRaw, gravity):
	#	pass
	#def dmpGetLinearAccelInWorld(self, data, packet = 0):
	#	pass
	#def dmpGetLinearAccelInWorld(self, data, packet = 0):
	#	pass
	#def dmpGetLinearAccelInWorld(self, v, packet = 0):
	#	pass
	#def dmpGetLinearAccelInWorld(self, v, vReal, q):
	#	pass
	#def dmpGetGyroAndAccelSensor(self, data, packet = 0):
	#	pass
	#def dmpGetGyroAndAccelSensor(self, data, packet = 0):
	#	pass
	#def dmpGetGyroAndAccelSensor(self, g, a, packet = 0):
	#	pass
	#def dmpGetGyroSensor(self, data, packet = 0):
	#	pass
	#def dmpGetGyroSensor(self, data, packet = 0):
	#	pass
	#def dmpGetGyroSensor(self, v, packet = 0):
	#	pass
	#def dmpGetControlData(self, data, packet = 0):
	#	pass
	#def dmpGetTemperature(self, data, packet = 0):
	#	pass
	#def dmpGetGravity(self, data, packet = 0):
	#	pass
	#def dmpGetGravity(self, data, packet = 0):
	#	pass
	#def dmpGetGravity(self, v, packet = 0):
	#	pass
	#def dmpGetGravity(self, v, q):
	#	pass
	#def dmpGetUnquantizedAccel(self, data, packet = 0):
	#	pass
	#def dmpGetUnquantizedAccel(self, data, packet = 0):
	#	pass
	#def dmpGetUnquantizedAccel(self, v, packet = 0):
	#	pass
	#def dmpGetQuantizedAccel(self, data, packet = 0):
	#	pass
	#def dmpGetQuantizedAccel(self, data, packet = 0):
	#	pass
	#def dmpGetQuantizedAccel(self, v, packet = 0):
	#	pass
	#def dmpGetExternalSensorData(self, data, size, packet = 0):
	#	pass
	#def dmpGetEIS(self, data, packet = 0):
	#	pass

	#def dmpGetEuler(self, data, q):
	#	pass
	#def dmpGetYawPitchRoll(self, data, q, gravity):
	#	pass

	## Get Floating Point data from FIFO
	#def dmpGetAccelFloat(self, data, packet = 0):
	#	pass
	#def dmpGetQuaternionFloat(self, data, packet = 0):
	#	pass

	#def dmpProcessFIFOPacket(self, dmpData):
	#	pass
	#def dmpReadAndProcessFIFOPacket(self, numPackets, processed = NULL):
	#	pass

	#def dmpSetFIFOProcessedCallback(self, void (*func) (void)):
	#	pass

	#def dmpInitFIFOParam(self):
	#	pass
	#def dmpCloseFIFO(self):
	#	pass
	#def dmpSetGyroDataSource(self, source):
	#	pass
	#def dmpDecodeQuantizedAccel(self):
	#	pass
	#def dmpGetGyroSumOfSquare(self):
	#	pass
	#def dmpGetAccelSumOfSquare(self):
	#	pass
	#def dmpOverrideQuaternion(self, q):
	#	pass
	#def dmpGetFIFOPacketSize(self):
	#	pass
	#endif

	# special methods for MotionApps 4.1 implementation
	#ifdef MPU9250_INCLUDE_DMP_MOTIONAPPS41
    #def dmpInitialize(self):
    #	pass
    #def dmpPacketAvailable(self):
    #	pass

    #def dmpSetFIFORate(self, fifoRate):
    #	pass
    #def dmpGetFIFORate(self):
    #	pass
    #def dmpGetSampleStepSizeMS(self):
    #	pass
    #def dmpGetSampleFrequency(self):
    #	pass
    #def dmpDecodeTemperature(self, tempReg):
    #	pass

    ## Register callbacks after a packet of FIFO data is processed
    ##dmpRegisterFIFORateProcess(inv_obj_func func, priority):
    #	pass
    ##dmpUnregisterFIFORateProcess(inv_obj_func func):
    #	pass
    #def dmpRunFIFORateProcesses(self):
    #	pass

    ## Setup FIFO for various output
    #def dmpSendQuaternion(self, accuracy):
    #	pass
    #def dmpSendGyro(self, elements, accuracy):
    #	pass
    #def dmpSendAccel(self, elements, accuracy):
    #	pass
    #def dmpSendLinearAccel(self, elements, accuracy):
    #	pass
    #def dmpSendLinearAccelInWorld(self, elements, accuracy):
    #	pass
    #def dmpSendControlData(self, elements, accuracy):
    #	pass
    #def dmpSendSensorData(self, elements, accuracy):
    #	pass
    #def dmpSendExternalSensorData(self, elements, accuracy):
    #	pass
    #def dmpSendGravity(self, elements, accuracy):
    #	pass
    #def dmpSendPacketNumber(self, accuracy):
    #	pass
    #def dmpSendQuantizedAccel(self, elements, accuracy):
    #	pass
    #def dmpSendEIS(self, elements, accuracy):
    #	pass

    ## Get Fixed Point data from FIFO
    #def dmpGetAccel(self, data, packet = 0):
    #	pass
    #def dmpGetAccel(self, data, packet = 0):
    #	pass
    #def dmpGetAccel(self, v, packet = 0):
    #	pass
    #def dmpGetQuaternion(self, data, packet = 0):
    #	pass
    #def dmpGetQuaternion(self, data, packet = 0):
    #	pass
    #def dmpGetQuaternion(self, q, packet = 0):
    #	pass
    #def dmpGet6AxisQuaternion(self, data, packet = 0):
    #	pass
    #def dmpGet6AxisQuaternion(self, data, packet = 0):
    #	pass
    #def dmpGet6AxisQuaternion(self, q, packet = 0):
    #	pass
    #def dmpGetRelativeQuaternion(self, data, packet = 0):
    #	pass
    #def dmpGetRelativeQuaternion(self, data, packet = 0):
    #	pass
    #def dmpGetRelativeQuaternion(self, data, packet = 0):
    #	pass
    #def dmpGetGyro(self, data, packet = 0):
    #	pass
    #def dmpGetGyro(self, data, packet = 0):
    #	pass
    #def dmpGetGyro(self, v, packet = 0):
    #	pass
    #def dmpGetMag(self, data, packet = 0):
    #	pass
    #def dmpSetLinearAccelFilterCoefficient(self, coef):
    #	pass
    #def dmpGetLinearAccel(self, data, packet = 0):
    #	pass
    #def dmpGetLinearAccel(self, data, packet = 0):
    #	pass
    #def dmpGetLinearAccel(self, v, packet = 0):
    #	pass
    #def dmpGetLinearAccel(self, v, vRaw, gravity):
    #	pass
    #def dmpGetLinearAccelInWorld(self, data, packet = 0):
    #	pass
    #def dmpGetLinearAccelInWorld(self, data, packet = 0):
    #	pass
    #def dmpGetLinearAccelInWorld(self, v, packet = 0):
    #	pass
    #def dmpGetLinearAccelInWorld(self, v, vReal, q):
    #	pass
    #def dmpGetGyroAndAccelSensor(self, data, packet = 0):
    #	pass
    #def dmpGetGyroAndAccelSensor(self, data, packet = 0):
    #	pass
    #def dmpGetGyroAndAccelSensor(self, g, a, packet = 0):
    #	pass
    #def dmpGetGyroSensor(self, data, packet = 0):
    #	pass
    #def dmpGetGyroSensor(self, data, packet = 0):
    #	pass
    #def dmpGetGyroSensor(self, v, packet = 0):
    #	pass
    #def dmpGetControlData(self, data, packet = 0):
    #	pass
    #def dmpGetTemperature(self, data, packet = 0):
    #	pass
    #def dmpGetGravity(self, data, packet = 0):
    #	pass
    #def dmpGetGravity(self, data, packet = 0):
    #	pass
    #def dmpGetGravity(self, v, packet = 0):
    #	pass
    #def dmpGetGravity(self, v, q):
    #	pass
    #def dmpGetUnquantizedAccel(self, data, packet = 0):
    #	pass
    #def dmpGetUnquantizedAccel(self, data, packet = 0):
    #	pass
    #def dmpGetUnquantizedAccel(self, v, packet = 0):
    #	pass
    #def dmpGetQuantizedAccel(self, data, packet = 0):
    #	pass
    #def dmpGetQuantizedAccel(self, data, packet = 0):
    #	pass
    #def dmpGetQuantizedAccel(self, v, packet = 0):
    #	pass
    #def dmpGetExternalSensorData(self, data, size, packet = 0):
    #	pass
    #def dmpGetEIS(self, data, packet = 0):
    #	pass

    #def dmpGetEuler(self, data, q):
    #	pass
    #def dmpGetYawPitchRoll(self, data, q, gravity):
    #	pass

    ## Get Floating Point data from FIFO
    #def dmpGetAccelFloat(self, data, packet = 0):
    #	pass
    #def dmpGetQuaternionFloat(self, data, packet = 0):
    #	pass

    #def dmpProcessFIFOPacket(self, dmpData):
    #	pass
    #def dmpReadAndProcessFIFOPacket(self, numPackets, processed = NULL):
    #	pass

    #def dmpSetFIFOProcessedCallback(self, void (*func) (void)):
    #	pass

    #def dmpInitFIFOParam(self):
    #	pass
    #def dmpCloseFIFO(self):
    #	pass
    #def dmpSetGyroDataSource(self, source):
    #	pass
    #def dmpDecodeQuantizedAccel(self):
    #	pass
    #def dmpGetGyroSumOfSquare(self):
    #	pass
    #def dmpGetAccelSumOfSquare(self):
    #	pass
    #def dmpOverrideQuaternion(self, q):
    #	pass
    #def dmpGetFIFOPacketSize(self):
	#endif