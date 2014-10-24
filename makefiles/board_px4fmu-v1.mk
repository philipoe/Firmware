#
# Board-specific definitions for the PX4FMU
#

#
# Configure the toolchain
#
CONFIG_ARCH			= CORTEXM4F
CONFIG_BOARD		= PX4FMU_V1
CONFIG_IMU			= ADIS16448

include $(PX4_MK_DIR)/toolchain_gnu-arm-eabi.mk
