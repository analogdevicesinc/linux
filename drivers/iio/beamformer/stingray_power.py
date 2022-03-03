import os
import sys
from time import sleep

class IIO:
	def write_reg(self, dev_name, reg, val):
		# print('iio_reg ' + dev_name + ' ' + str(reg) + ' ' + str(val))
		value = os.popen('iio_reg ' + dev_name + ' ' + str(reg) + ' ' + str(val)).read()
		if value == '':
			return ''
		return int(value, 16)

	def read_reg(self, dev_name, reg):
		value = os.popen('iio_reg ' + dev_name + ' ' + str(reg)).read()
		if value == '':
			return ''
		return int(value, 16)

	def write_dev_attr(self, dev_name, attr, val):
		# print('iio_attr -d ' + dev_name + ' ' + str(attr) + ' ' + str(val))
		value = os.popen('iio_attr -d ' + dev_name + ' ' + str(attr) + ' ' + str(val)).read()
		if value == '':
			return ''
		return int(value, 10)

	def read_dev_attr(self, dev_name, attr):
		value = os.popen('iio_attr -d ' + dev_name + ' ' + str(attr)).read()
		if value == '':
			return ''
		return int(value, 10)

	def write_dev_ch_attr(self, dev_name, channel, attr, val, dir = ''):
		value = os.popen('iio_attr -c ' + dir + ' ' + dev_name + ' ' + str(channel) + ' ' + str(attr) + ' ' + str(val)).read()
		if value == '':
			return ''
		try:
			return int(value, 10)
		except ValueError:
			return value

	def read_dev_ch_attr(self, dev_name, channel, attr):
		value = os.popen('iio_attr -c ' + dev_name + ' ' + str(channel) + ' ' + str(attr)).read()
		if value == '':
			return ''
		return int(value, 10)

class Ltc2992:
	_dev_name='ltc2992'
	_gpio_name=''

	def __init__(self):
		self.export_gpios()
		self.get_gpio()

	def export_gpios(self):
		device = ''
		devices = os.popen('grep "" /sys/class/gpio/gpiochip*/device/name').read()
		devices = devices.split(sep="\n")
		for line in devices:
			if self._dev_name in line:
				device = line
				break
		if device == '':
			return
		device = device.split(sep="/")
		for line in device:
			if 'gpiochip' in line:
				device = line
				break
		if device == '':
			return
		device = device.replace('gpiochip', '')
		gpio = int(device, 10)
		os.popen('echo ' + str(gpio + 0) + ' > /sys/class/gpio/export').read()
		os.popen('echo ' + str(gpio + 1) + ' > /sys/class/gpio/export').read()
		os.popen('echo ' + str(gpio + 2) + ' > /sys/class/gpio/export').read()
		os.popen('echo ' + str(gpio + 3) + ' > /sys/class/gpio/export').read()

	def get_gpio(self):
		device = ''
		devices = os.popen('ls /sys/class/gpio').read()
		devices = devices.split(sep="\n")
		for line in devices:
			if (self._dev_name in line) & ('GPIO1' in line):
				device = line
				break
		device = device.replace('GPIO1', 'GPIO')
		if device == '':
			raise SystemError('Stingray power: Cant get GPIOs')
		self._gpio_name = device

	def power_sequencer_enable(self):
		value = os.popen('cat /sys/class/gpio/' + self._gpio_name + '1/value').read()
		if value == '':
			return 0
		val = int(value, 10)
		return val == 0

	def p5v_enable(self):
		value = os.popen('cat /sys/class/gpio/' + self._gpio_name + '2/value').read()
		if value == '':
			return 0
		val = int(value, 10)
		return val == 0

	def power_sequencer_power_good(self):
		value = os.popen('cat /sys/class/gpio/' + self._gpio_name + '3/value').read()
		if value == '':
			return 0
		val = int(value, 10)
		return val == 0

	def p5v_power_good(self):
		value = os.popen('cat /sys/class/gpio/' + self._gpio_name + '4/value').read()
		if value == '':
			return 0
		val = int(value, 10)
		return val == 0

class GPIOS:
	P5V_CTRL_PIN = 4
	PWR_UP_DOWN_PIN = 5

	def get_device(self, dev_label):
		devices = os.popen('grep "" /sys/bus/iio/devices/iio\:device*/label').read()
		devices = devices.split(sep="\n")
		for line in devices:
			if dev_label in line:
				device = line
				break

		device = device.split(sep="/")
		for line in device:
			if 'iio:device' in line:
				device = line
				break

		return device

	def gpio_pulse(self, iio, ch):
		device = self.get_device(dev_label='stingray_control')
		iio.write_dev_ch_attr(dev_name = device, channel = 'voltage' + str(ch), attr = 'raw', val = 1)
		iio.write_dev_ch_attr(dev_name = device, channel = 'voltage' + str(ch), attr = 'raw', val = 0)

class Stingray:
	""" Class for Stingray board control. """
	_revision = 'B'
	_POWER_DELAY = 0.2
	_fully_powered = False
	_devices = list()

	def __init__(self):
		self._iio = IIO()
		self._ltc = Ltc2992()
		self._gpio = GPIOS()

	class Adar1000:
		_channels = 4
		_BIAS_CODE_TO_VOLTAGE_SCALE = -0.018824

		INTERFACE_CONFIG_A_REG = 0x0000
		SCRATCHPAD_REG = 0x000A
		LD_WRK_REGS_REG = 0x0028
		PA1_BIAS_ON_REG = 0x0029
		PA2_BIAS_ON_REG = 0x002A
		PA3_BIAS_ON_REG = 0x002B
		PA4_BIAS_ON_REG = 0x002C
		LNA_BIAS_ON_REG = 0x002D
		RX_ENABLES_REG = 0x002E
		TX_ENABLES_REG = 0x002F
		MISC_ENABLES_REG = 0x0030
		SW_CTRL_REG = 0x0031
		ADC_CTRL_REG = 0x0032
		ADC_OUTPUT_REG = 0x0033
		BIAS_CURRENT_RX_LNA_REG = 0x0034
		BIAS_CURRENT_RX_REG = 0x0035
		BIAS_CURRENT_TX_REG = 0x0036
		BIAS_CURRENT_TX_DRV_REG = 0x0037
		MEM_CTRL_REG = 0x0038
		RX_BEAM_COMMON_REG = 0x0039
		TX_BEAM_COMMON_REG = 0x003A
		PA1_BIAS_OFF_REG = 0x0046
		PA2_BIAS_OFF_REG = 0x0047
		PA3_BIAS_OFF_REG = 0x0048
		PA4_BIAS_OFF_REG = 0x0049
		LNA_BIAS_OFF_REG = 0x004A
		TX_TO_RX_DELAY_REG = 0x004B
		RX_TO_TX_DELAY_REG = 0x004C
		TX_BEAM_STEP_START_REG = 0x004D
		TX_BEAM_STEP_STOP_REG = 0x004E
		RX_BEAM_STEP_START_REG = 0x004F
		RX_BEAM_STEP_STOP_REG = 0x0050
		RX_BIAS_RAM_CTL_REG = 0x0051
		TX_BIAS_RAM_CTL_REG = 0x0052

		def __init__(self, name):
			self._name = name

		def initialize(self, iio, pa_off=-2.5, pa_on=-2.5, lna_off=-2, lna_on=-2):
			# print('Initialize device: ' + self._name)
			# Set the bias currents
			iio.write_reg(self._name, self.BIAS_CURRENT_RX_LNA_REG, 0x08)
			iio.write_reg(self._name, self.BIAS_CURRENT_RX_REG, 0x55)
			iio.write_reg(self._name, self.BIAS_CURRENT_TX_REG, 0x2D)
			iio.write_reg(self._name, self.BIAS_CURRENT_TX_DRV_REG, 0x06)

			# Disable RAM control
			iio.write_dev_attr(self._name, 'beam_mem_enable', 0)
			iio.write_dev_attr(self._name, 'bias_mem_enable', 0)
			iio.write_dev_attr(self._name, 'common_mem_enable', 0)

			# Enable all internal amplifiers
			iio.write_dev_attr(self._name, 'rx_vga_enable', 1)
			iio.write_dev_attr(self._name, 'rx_vm_enable', 1)
			iio.write_dev_attr(self._name, 'rx_lna_enable', 1)
			iio.write_dev_attr(self._name, 'tx_vga_enable', 1)
			iio.write_dev_attr(self._name, 'tx_vm_enable', 1)
			iio.write_dev_attr(self._name, 'tx_drv_enable', 1)

			# Disable Tx/Rx paths
			iio.write_dev_attr(self._name, 'tx_en', 0)
			iio.write_dev_attr(self._name, 'rx_en', 0)
			iio.write_dev_attr(self._name, 'tr_source', 0)

			# Enable the PA/LNA bias DACs
			iio.write_dev_attr(self._name, 'bias_enable', 1)
			iio.write_dev_attr(self._name, 'lna_bias_out_enable', 1)
			iio.write_dev_attr(self._name, 'bias_ctrl', 1)

			# Configure the external switch control
			iio.write_dev_attr(self._name, 'sw_drv_tr_state', 1)
			iio.write_dev_attr(self._name, 'sw_drv_en_tr', 1)

			# Set the default LNA bias
			dac_code = int(lna_on / self._BIAS_CODE_TO_VOLTAGE_SCALE)
			val = iio.write_dev_attr(self._name, 'lna_bias_on', dac_code)
			if val != dac_code:
				raise SystemError('Cant write device attribute')

			dac_code = int(lna_off / self._BIAS_CODE_TO_VOLTAGE_SCALE)
			val = iio.write_dev_attr(self._name, 'lna_bias_off', dac_code)
			if val != dac_code:
				raise SystemError('Stingray power: Cant write device attribute')

			# Settings for each channel
			for i in range(self._channels):
				# Default channel enable
				val = iio.write_dev_ch_attr(self._name, 'voltage' + str(i), 'powerdown', 1, '-i') #RX
				if val != 1:
					raise SystemError('Stingray power: Cant write device attribute')
				val = iio.write_dev_ch_attr(self._name, 'voltage' + str(i), 'powerdown', 1, '-o') #TX
				if val != 1:
					raise SystemError('Stingray power: Cant write device attribute')

				# Default PA bias
				dac_code = int(pa_on / self._BIAS_CODE_TO_VOLTAGE_SCALE)
				val = iio.write_dev_ch_attr(self._name, 'voltage' + str(i), 'pa_bias_on', dac_code)
				if val != dac_code:
					raise SystemError('Stingray power: Cant write device attribute')

				dac_code = int(pa_off / self._BIAS_CODE_TO_VOLTAGE_SCALE)
				val = iio.write_dev_ch_attr(self._name, 'voltage' + str(i), 'pa_bias_off', dac_code)
				if val != dac_code:
					raise SystemError('Stingray power: Cant write device attribute')


				iio.write_dev_ch_attr(self._name, 'voltage' + str(i), 'hardwaregain', -0.127, '-i') #RX
				val = iio.write_dev_ch_attr(self._name, 'voltage' + str(i), 'phase', 0, '-i') #RX

				val = iio.write_dev_ch_attr(self._name, 'voltage' + str(i), 'hardwaregain', -0.127, '-o') #TX
				val = iio.write_dev_ch_attr(self._name, 'voltage' + str(i), 'phase', 0, '-o') #TX

	def pulse_power_pin(self, which):
		if which.lower() == 'pwr_up_down':
			self._gpio.gpio_pulse(self._iio, self._gpio.PWR_UP_DOWN_PIN)
		elif which.lower() == '5v_ctrl':
			self._gpio.gpio_pulse(self._iio, self._gpio.P5V_CTRL_PIN)
		else:
			raise ValueError("Can't pulse the pin:" + which.lower())

	def partially_powered(self):
		""" Status of the board's power tree connected to the ADM1186 """

		# If Rev.A, there's no way to check on the rails directly, we have to rely on SPI readback
		if self._revision == 'A':
			devices = os.popen('grep "" /sys/bus/iio/devices/iio\:device*/name').read()
			devices = devices.split(sep="\n")
			for line in devices:
				if 'adar1000' in line:
					return True

			return False
		else:
			# If Rev.B, we can directly read the sequencer's EN and PG status.
			return bool(self._ltc.power_sequencer_enable() & self._ltc.power_sequencer_power_good())

	def fully_powered(self):
		""" Status of the board's full power tree """

		# If Rev.A, we have to rely on the flags
		if self._revision == 'A':
			return bool(self.partially_powered() & self._fully_powered)

		# If Rev.B, we can directly read the EN and status signals.
		else:
			return bool(self.partially_powered() & self._ltc.p5v_enable() & self._ltc.p5v_power_good())

	def get_devices(self):
		devices = os.popen('grep "" /sys/bus/iio/devices/iio\:device*/name').read()
		devices = devices.split(sep="\n")
		for line in devices:
			if 'adar1000' in line:
				device = line
				device = device.split(sep="/")
				for line in device:
					if 'iio:device' in line:
						a = self.Adar1000(line)
						self._devices.append(a)

	def initialize_devices(self, pa_off=-2.5, pa_on=-2.5, lna_off=-2, lna_on=-2):
		""" Initialize the devices to allow for safe powerup of the board

		Args:
			pa_off (float): Voltage to set the PA_BIAS_OFF values to during initialization
			pa_on (float): Voltage to set the PA_BIAS_ON values to during initialization
			lna_off (float): Voltage to set the LNA_BIAS_OFF values to during initialization
			lna_on (float): Voltage to set the LNA_BIAS_ON values to during initialization
		"""
		for adar in self._devices:
			adar.initialize(self._iio, pa_off, pa_on, lna_off, lna_on)

	def powerdown(self):
		self.get_devices()
		print('Stingray power: Found ' + str(len(self._devices)) + ' ADAR1000 devices')

		# If the +5V rail is up
		if self.fully_powered():
			# Turn on a single PA to help bring this down faster
			self._iio.write_dev_attr(self._devices[1]._name, 'rx_en', 0)
			self._iio.write_dev_attr(self._devices[1]._name, 'tr_source', 0)
			self._iio.write_dev_attr(self._devices[1]._name, 'tx_en', 1)

			dac_code = int(-1.1 / self._devices[1]._BIAS_CODE_TO_VOLTAGE_SCALE)
			self._iio.write_dev_ch_attr(self._devices[1]._name, 'voltage' + str(0), 'pa_bias_on', dac_code)

			# Send a signal to power down the +5V rail
			self.pulse_power_pin('5v_ctrl')
			# Wait for the +5V rail to come down
			if self._revision == 'A':
				sleep(self._POWER_DELAY / 4)
			else:
				while self.fully_powered() > 0.5:
					sleep(0.01)
		else:
			print("Stingray power: Board not powered")

		# If the board is partially powered up
		if self.partially_powered():
			# Turn on a single cell's LNAs to help bring this down faster
			self._iio.write_dev_attr(self._devices[1]._name, 'lna_bias_out_enable', 0)

			# Send a signal to power down the Stingray board's sequenced rails
			self.pulse_power_pin('pwr_up_down')

			# Wait for the rest of the rails to come down
			if self._revision == 'A':
				sleep(self._POWER_DELAY)
			else:
				while self.partially_powered():
					sleep(0.01)

			print("Stingray power: Power down sequence succeeded")

	def bind_devices(self):
		os.popen('echo -n "spi1.1" > /sys/bus/spi/drivers/adar1000/unbind').read()
		os.popen('echo -n "spi1.1" > /sys/bus/spi/drivers/adar1000/bind').read()

		os.popen('echo -n "spi1.2" > /sys/bus/spi/drivers/adar1000/unbind').read()
		os.popen('echo -n "spi1.2" > /sys/bus/spi/drivers/adar1000/bind').read()

		os.popen('echo -n "spi1.3" > /sys/bus/spi/drivers/adar1000/unbind').read()
		os.popen('echo -n "spi1.3" > /sys/bus/spi/drivers/adar1000/bind').read()

		os.popen('echo -n "spi1.4" > /sys/bus/spi/drivers/adar1000/unbind').read()
		os.popen('echo -n "spi1.4" > /sys/bus/spi/drivers/adar1000/bind').read()

	def powerup(self, enable_5v=True, **kwargs):
		# Get any keywords that were given
		pa_off = kwargs.get('pa_off', -2.5)
		pa_on = kwargs.get('pa_on', -2.5)
		lna_off = kwargs.get('lna_off', -2)
		lna_on = kwargs.get('lna_on', -2)

		if self.fully_powered():
			return

		# If the board is powered down
		if not self.partially_powered():
			self.pulse_power_pin('pwr_up_down')
		
			# Wait for supplies to settle
			if self._revision == 'A':
				sleep(self._POWER_DELAY)
			else:
				loops = 0
				while not self._ltc.power_sequencer_power_good():
					sleep(0.01)
					loops += 1

					if loops > 50:
						raise SystemError("Stingray power: Power sequencer PG pin never went high, something's wrong")

			self.bind_devices()

			if not self.partially_powered:
				raise SystemError("Stingray power: Board didn't power up!")

		self.get_devices()
		if len(self._devices) == 0:
			raise SystemError("Stingray power: No ADAR1000 devices found")
		else:
			print('Stingray power: Found ' + str(len(self._devices)) + ' ADAR1000 devices')

		# Initialize all the ADAR1000s
		self.initialize_devices(pa_off=pa_off, pa_on=pa_on, lna_off=lna_off, lna_on=lna_on)

		# Send a signal to power up the +5V rail
		if enable_5v and not self.fully_powered():
			self.pulse_power_pin('5v_ctrl')
			if not self.fully_powered():
				print("Stingray power: Power up sequence failed")
			else:
				print("Stingray power: Power up sequence succeeded")

if __name__ == '__main__':
	try:
		if sys.argv[1] == 'up':
			print("Stingray power: Start power up sequence")
			stingray = Stingray()
			stingray.powerup(enable_5v=True)
		elif sys.argv[1] == 'partial':
			print("Stingray power: Start partial power up sequence")
			stingray = Stingray()
			stingray.powerup(enable_5v=False)
		elif sys.argv[1] == 'down':
			print("Stingray power: Start power down sequence")
			stingray = Stingray()
			stingray.powerdown()
		else:
			print('Stingray power: Please specify "up", "partial" or "down" option')
	except IndexError:
		print('Stingray power: Please specify "up" or "down" option')
