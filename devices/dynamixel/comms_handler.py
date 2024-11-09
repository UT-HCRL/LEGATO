import dynamixel_sdk as sdk

from . import dictionary as dict
from .utils import twos_comp_forward, twos_comp_backward, DXL_LOWORD, DXL_HIWORD, DXL_LOBYTE, DXL_HIBYTE


class CommsHandler():

	def __init__(self,  params):
		'''
		Describe this stuff
		'''
		self.params = params

		#Connect to the computer's port
		self.PortHandler = sdk.PortHandler(self.params['PORTNAME'])
		self.PortHandler.openPort()
		self.PortHandler.setBaudRate(self.params['BAUDRATE'])

		#Build handler that constructs packets to be sent
		self.PacketHandler = sdk.PacketHandler(self.params['PROTOCOL_VERSION'])

		self.ADDR_DICT = dict.X_Series_Address
		self.LEN_DICT = dict.X_Series_Address_Length

		self.SyncWriteEncoder = sdk.GroupSyncWrite(self.PortHandler, self.PacketHandler, self.ADDR_DICT['ADDR_GOAL_POSITION'], self.LEN_DICT['LEN_GOAL_POSITION'])

		self.SyncReadEncoder = sdk.GroupSyncRead(self.PortHandler, self.PacketHandler, self.ADDR_DICT['ADDR_PRESENT_POSITION'], self.LEN_DICT['LEN_PRESENT_POSITION'])
		self.SyncReadTorque = sdk.GroupSyncRead(self.PortHandler, self.PacketHandler, self.ADDR_DICT['ADDR_PRESENT_CURRENT'], self.LEN_DICT['LEN_PRESENT_CURRENT'])



	def get_porthandler(self):
		return self.PortHandler

	def get_packethandler(self):
		return self.PacketHandler


	def sync_read_present_encoder(self, ids):

		for id in ids:
			_ = self.SyncReadEncoder.addParam(id)

		_ = self.SyncReadEncoder.txRxPacket()

		enc = []
		for id in ids:
			e = self.SyncReadEncoder.getData(id, self.ADDR_DICT['ADDR_PRESENT_POSITION'], self.LEN_DICT['LEN_PRESENT_POSITION'])
			enc.append(twos_comp_backward(e,32))

		self.SyncReadEncoder.clearParam()

		return enc

	def sync_read_present_torque(self, ids):

		for id in ids:
			_ = self.SyncReadTorque.addParam(id)

		_ = self.SyncReadTorque.txRxPacket()

		tor = []
		for id in ids:
			t = self.SyncReadTorque.getData(id, self.ADDR_DICT['ADDR_PRESENT_CURRENT'], self.LEN_DICT['LEN_PRESENT_CURRENT'])
			tor.append(twos_comp_backward(t,16))

		self.SyncReadTorque.clearParam()

		return tor

	def sync_write_target_encoder(self, ids, enc):

		for i in range(len(ids)):
			e =  twos_comp_forward(enc[i],32)
			target = [DXL_LOBYTE(DXL_LOWORD(e)), DXL_HIBYTE(DXL_LOWORD(e)), DXL_LOBYTE(DXL_HIWORD(e)), DXL_HIBYTE(DXL_HIWORD(e))]
			_ = self.SyncWriteEncoder.addParam(ids[i], target)

		error = self.SyncWriteEncoder.txPacket()

		self.SyncWriteEncoder.clearParam()

		return error
