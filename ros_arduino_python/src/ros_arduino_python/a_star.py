# Copyright Pololu Corporation.  For more information, see https://www.pololu.com/
import smbus
import struct

class AStar(object):
  def __init__(self, port = 1):
    self.bus = smbus.SMBus(port)

  def close():
    self.bus.close()

  # Catch IO exception error:
  def try_io(self, my_call, tries=10):
    assert tries > 0
    error = None
    result = None

    while tries:
        try:
            result = my_call()
        except IOError as e:
            print tries, error
            from subprocess import call
            call(["i2cdetect", "-y","1"])
            error = e
            tries -= 1
        else:
            break

    if not tries:
        raise error

    return result

  def read_unpack(self, address, size, format):
    # Ideally we could do this:
    #    byte_list = self.bus.read_i2c_block_data(20, address, size)
    # But the AVR's TWI module can't handle a quick write->read transition,
    # since the STOP interrupt will occasionally happen after the START
    # condition, and the TWI module is disabled until the interrupt can
    # be processed.

    #self.bus.write_byte(20,address)
    self.try_io(lambda: self.bus.write_byte(20,address))
    byte_list = []
    for n in range(0,size):
      #byte_list.append(self.bus.read_byte(20))
      bl = self.try_io(lambda: self.bus.read_byte(20))
      byte_list.append(bl)
    return struct.unpack(format,bytes(bytearray(byte_list)))

  def write_pack(self, address, format, *data):
    data_array = map(ord, list(struct.pack(format, *data)))
    #self.bus.write_i2c_block_data(20, address, data_array)
    self.try_io(lambda: self.bus.write_i2c_block_data(20, address, data_array))

  def leds(self, red, yellow, green):
    self.write_pack(0, 'BBB', red, yellow, green)

  def read_buttons(self):
    return self.read_unpack(3, 3, "???")

  def motors(self, left, right):
    self.write_pack(6, 'hh', left, right)

  def read_battery_millivolts(self):
    return self.read_unpack(10, 2, "H")[0]

  def read_analog(self):
    return self.read_unpack(12, 12, "HHHHHH")

  # Not sure if this is necessary in I2C:
  def send_command(self,cmd):
    self.write_pack(24,'c',cmd)

  # Add quadrature encoders
  def get_encoder_counts(self):
    return self.read_unpack(25, 8, "ll")

  def reset_encoders(self):
    self.write_pack(25, 'll', 0, 0)

  def update_pid(self, Kp, Kd, Ki, Ko):
    self.write_pack(33,"hhhh", Kp, Kd, Ki, Ko)

  def write_servos(self, left_srv, right_srv):
     self.write_pack(41, 'BB', left_srv, right_srv)

  def write_servo_left(self, left_srv):
      self.write_pack(41, 'B', left_srv)

  def write_servo_right(self, right_srv):
      self.write_pack(42, 'B', right_srv)

  def read_servos(self):
    return self.read_unpack(43, 2, 'BB')

  def play_notes(self, notes):
    self.write_pack(45, 'B15s', 1, notes.encode("ascii"))

  def test_read8(self):
    self.read_unpack(0, 8, 'cccccccc')

  def test_write8(self):
    self.bus.write_i2c_block_data(20, 0, [0,0,0,0,0,0,0,0])
