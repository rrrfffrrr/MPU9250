# I2C wrapper class

from abc import *
class I2CWrapper(metaclass=ABCMeta):
	## read
	# @param device device address
	# @param addr register address
	# @param offset offset of start bit
	# @param size size of bits
	# @return list of bits([0, 0, 1, 1, ...])
	@abstractmethod
	def readBits(self, device, addr, offset, size):
		pass

	# @param device device address
	# @param addr register address
	# @param size size of bytes
	# @return list of bytes([104, 2, 0, 42, ...])
	@abstractmethod
	def readBytes(self, device, addr, size):
		pass

	## write
	# @param device device address
	# @param addr register address
	# @param offset offset of start bit
	# @param bit value to write(1, 0)
	# @return success
	@abstractmethod
	def writeBit(self, device, addr, offset, bit):
		pass

	# @param device device address
	# @param addr register address
	# @param offset offset of start bit
	# @param bits list of values([1, 0, 1, ...])
	# @return success
	@abstractmethod
	def writeBits(self, device, addr, offset, bits):
		pass

	# @param device device address
	# @param addr register address
	# @param byte value to write(104)
	# @return success
	@abstractmethod
	def writeByte(self, device, addr, byte):
		pass

	# @param device device address
	# @param addr register address
	# @param bytes list of bytes([104, 2, 0, 42, ...])
	# @return success
	@abstractmethod
	def writeBytes(self, device, addr, bytes):
		pass

	# @param ms millisecond to delay
	@abstractmethod
	def delay(self, ms):
		pass