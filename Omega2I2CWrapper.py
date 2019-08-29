from OmegaExpansion import onionI2C
from I2CWrapper import I2CWrapper
from math import ceil
from time import sleep

class Omega2I2CWrapper(I2CWrapper):
	def __init__(self, optimize = False):
		super().__init__();
		self.i2c = onionI2C.OnionI2C();
		self.optimize = optimize;

	def readBits(self, device, addr, offset, size):
		values = self.i2c.readBytes(device, addr, ceil(size/8));
		bits = []
		for e in values:
			bits.extend(bin(e)[2:].zfill(8));
		bits = list(map(int, bits));

		for i in range(offset):
			del bits[0];
		if self.optimize != True:
			for i in range(max(len(bits) - size, 0)):
				del bits[len(bits) - 1];
		return bits

	def readBytes(self, device, addr, size):
		return self.i2c.readBytes(device, addr, size);

	def writeBit(self, device, addr, offset, bit):
		values = self.readBits(device, addr, 0, 7);
		values[offset] = bit;
		return self.writeByte(device, addr, int("".join(map(str, values)), 2));

	def writeBits(self, device, addr, offset, bits):
		values = self.readBits(device, addr, 0, 7);
		values[offset:offset+len(bits)] = bits;
		return self.writeByte(device, addr, int("".join(map(str, values)), 2));

	def writeByte(self, device, addr, byte):
		return self.i2c.writeByte(device, addr, byte);

	def writeBytes(self, device, addr, bytes):
		return self.i2c.writeBytes(device, addr, bytes);

	def delay(self, ms):
		sleep(ms/1000.0);