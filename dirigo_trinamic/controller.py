import struct
import threading

import serial

from dirigo_trinamic.enumerations import BaudRates, StatusCodes



class TrinamicController:
    """Controller for command composition, send, and receive via Serial port."""
    def __init__(self, 
                 com_port: int, 
                 module_address: int = 1, 
                 baud_rate = BaudRates.BAUD_9600):
        if not isinstance(module_address, int) or module_address < 1:
            raise ValueError("Module address must be an integer 1 or greater.")
        self._module_address = module_address
        self._motor_number = 0

        if not isinstance(com_port, int) or com_port < 1:
            raise ValueError("Com port must be an integer 1 or greater.")
        self._com_port = com_port

        if not isinstance(baud_rate, BaudRates):
            raise ValueError("Baud rate must be set with one of the available BaudRate enumerations.")
        self._baud_rate = baud_rate

        self._lock = threading.Lock()

        self.open_serial_port()

    def open_serial_port(self):
        com_str = "COM" + str(self._com_port)

        self._serial_port = serial.Serial(
            port=com_str, 
            baudrate=self._baud_rate.rate, 
            timeout=3, 
            write_timeout=None
        )

    def make_command(self, instruction, type=0, operand=0):
        cmd = bytearray()

        # byte0: Module address
        cmd.append(self._module_address) 
        # byte1: Command number (one of the enumerations)
        cmd.append(instruction) 
        # byte2: Type number (like a sub-command, also an enumeration)
        cmd.append(type) 
        # byte3: Motor or bank number
        cmd.append(self._motor_number) 
        # bytes4-7: value (operand) ("MSB first!")
        cmd.extend(struct.pack('>i', operand)) 
        # byte8: checksum
        cmd.append(sum(cmd) & 0xFF) 

        return cmd
    
    def send_receive(self, cmd: bytearray):
        """Send a command, validate response"""
        with self._lock:
            self._serial_port.reset_input_buffer()
            self._serial_port.reset_output_buffer()
            self._serial_port.write(cmd)
            self._serial_port.flush()
            response = self._serial_port.read(9)
            
        return self._interpret_response(bytearray(response))
    
    def _interpret_response(self, response: bytearray):
        if response:
            checksum = sum(response[:-1]) & 0xFF
            if not checksum == response[-1]:
                raise ConnectionError(
                    f"Checksum Error, attached checksum: {response[-1]}, "
                    f"calculated checksum: {checksum}"
                )

            if not response[2] == StatusCodes.SUCCESSFULLY_EXECUTED:
                status = StatusCodes(response[2]).name
                raise ConnectionError(f"Command unsuccessful: {status}")
            
            # bytes4-7: value
            value = struct.unpack('>i', response[4:8])[0]

            return value
