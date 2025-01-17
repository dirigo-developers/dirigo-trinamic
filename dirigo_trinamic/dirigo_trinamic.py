import struct
import threading

import serial
from dirigo import units
from dirigo.hw_interfaces.scanner import ObjectiveZScanner

from enumerations import *




class TrinamicController:
    """Controller for command composition, send, and receive via Serial port."""
    def __init__(self, com_port: int, module_address: int = 1, baud_rate = BaudRates.BAUD_9600):
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

        self.open_seial_port()

    def open_seial_port(self):
        com_str = "COM" + str(self._com_port)

        self._serial_port = serial.Serial(
            com_str, self._baud_rate, timeout=3, write_timeout=None
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
        self._serial_port.reset_input_buffer()
        self._serial_port.reset_output_buffer()
        self._serial_port.write(cmd)
        self._serial_port.flush()
        response = self._serial_port.read(9)
        
        return self._interpret_response(response)
    
    def _interpret_response(self, response: bytearray):
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



class TrinamicObjectiveZScanner(ObjectiveZScanner):

    def __init__(self, com_port: int, step_angle: str, 
                 travel_per_rev: str, 
                 module_address: int = 1, **kwargs):
        super().__init__(**kwargs)

        self._controller = TrinamicController(com_port, module_address)

        self._step_angle = units.Angle(step_angle)
        self._travel_per_rev = units.Position(travel_per_rev)

    @property
    def _microstep_resolution(self) -> int:
        """Returns the number of microsteps per full step."""
        cmd = self._controller.make_command(
            instruction=ParameterCommands.GET_AXIS_PARAMETER, 
            type=AdvancedAxisParameters.MICROSTEP_RESOLUTION
        )
        return_code = self._controller.send_receive(cmd)

        if return_code == 1:
            return 0.5 
        else:
            return 2**return_code # increasing powers of 2
        
    @property
    def _distance_per_microstep(self) -> units.Position:
        """The linear travel per microstep."""
        normalized_step_angle = self._step_angle / units.Angle("360 deg")
        travel_per_step = self._travel_per_rev * normalized_step_angle
        
        return travel_per_step / self._microstep_resolution

    @property
    def position(self):
        """The current (actual) position."""
        cmd = self._controller.make_command(
            instruction=ParameterCommands.GET_AXIS_PARAMETER, 
            type=AxisParameters.ACTUAL_POSITION
        )
        value = self._controller.send_receive(cmd)
        return value * self._distance_per_microstep
    
    @property
    def max_velocity(self) -> units.Velocity:
        """
        Return the maximum velocity used in move operations.

        Note that this is the imposed velocity limit for moves. It is not
        necessarily the maximum attainable velocity for this stage.
        """
        cmd = self._controller.make_command(
            instruction=ParameterCommands.GET_AXIS_PARAMETER, 
            type=AxisParameters.MAX_POSITIONING_SPEED
        )
        velo_trinamic = self._controller.send_receive(cmd)

        velo_microsteps = velo_trinamic * self._velocity_factor
        return units.Velocity(velo_microsteps * float(self._distance_per_microstep))
    
    @max_velocity.setter
    def max_velocity(self, velocity: units.Velocity):
        """Sets the maximum velocity."""
        if not isinstance(velocity, units.Velocity):
            raise ValueError("`max_velocity` must be set with a Velocity object.")
        velo_microsteps = float(velocity) / float(self._distance_per_microstep)
        velo_trinamic = round(velo_microsteps / self._velocity_factor)
        if not (0 < velo_trinamic <= 2047):
            raise ValueError(f"Attempted to set stepper velocity outside of "
                             f"allowable range (0-2047). Got: {velo_trinamic}")

        cmd = self._controller.make_command(
            instruction=ParameterCommands.SET_AXIS_PARAMETER, 
            type=AxisParameters.MAX_POSITIONING_SPEED,
            operand=velo_trinamic
        )
        value = self._controller.send_receive(cmd)

    @property
    def _velocity_factor(self):
        """Ratio of microsteps per second to internal Trinamic velocity (0-2047).
        """
        cmd = self._controller.make_command(
            instruction=ParameterCommands.GET_AXIS_PARAMETER, 
            type=AdvancedAxisParameters.PULSE_DIVISOR
        )
        value = self._controller.send_receive(cmd)
        return 16e6 / (2**value * 2048 * 32)

    @property
    def acceleration(self) -> units.Acceleration:
        """
        Return the acceleration used during ramp up/down phase of move.
        """
        pass

    @acceleration.setter
    def acceleration(self, value: units.Acceleration):
        pass

    @property
    def device_info(self) -> None:
        """Returns an object describing permanent properties of the stage."""
        pass


    @property
    def position_limits(self) -> units.RangeWithUnits:
        """Returns an object describing the stage movement limits."""
        pass

    @property
    def moving(self) -> bool:   
        """Return True if the stage axis is currently moving."""
        pass

    def move_to(self, position: units.Position, blocking: bool = False):
        """
        Initiate move to specified spatial position.

        Choose whether to return immediately (blocking=False, default) or to
        wait until finished moving (blocking=True).
        """
        raise NotImplementedError("Only for homed devices")

    def move_relative(self, move_distance: units.Position):
        """Moves a distance relative the current position."""
        cmd = self._controller.make_command(
            instruction=MotionCommands.MOVE_TO_POSITION, 
            type=MoveTypes.RELATIVE, 
            operand=round(move_distance / self._distance_per_microstep)
        )
        value = self._controller.send_receive(cmd)
        return value
    
    def move_velocity(self, velocity: units.Velocity):
        """"
        Initiate movement at velocity until stopped.
        """
        if not isinstance(velocity, units.Velocity):
            raise ValueError("`move_velocity()` argument must of be type Velocity.")
        
        instruction = MotionCommands.ROTATE_RIGHT if velocity > 0 \
            else MotionCommands.ROTATE_LEFT
        
        velocity_microsteps = abs(float(velocity)) / self._distance_per_microstep
        velocity_trinamic = round(velocity_microsteps / self._velocity_factor)

        if not (0 < velocity_trinamic <= 2047):
            raise ValueError(f"Attempted to set stepper velocity outside of "
                             f"allowable range (0-2047). Got: {velocity_trinamic}")
        
        cmd = self._controller.make_command(
            instruction=instruction, 
            operand=velocity_trinamic
        )
        value = self._controller.send_receive(cmd)

    def stop(self):
        """Halts motion."""
        cmd = self._controller.make_command(
            instruction=MotionCommands.MOTOR_STOP 
        )
        value = self._controller.send_receive(cmd)
        return value

    def home(self, blocking: bool = False):
        """
        Initiate homing. 
        
        Choose whether to return immediately (blocking=False, default) or to
        wait until finished homing (blocking=True).
        """
        pass

    @property
    def homed(self) -> bool:
        """Return whether the stage has been home."""
        pass
        


if __name__ == "__main__":
    import time
    config = {
        "com_port": 5,
        "step_angle": "1.8 deg",
        "travel_per_rev": "0.1 mm",
    }

    tri = TrinamicObjectiveZScanner(axis="z", **config)


    tri.move_velocity(units.Velocity('0.5 mm/s'))
    time.sleep(2)
    tri.stop()

    None

