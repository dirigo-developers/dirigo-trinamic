from functools import cached_property
from typing import Literal, ClassVar, Type

from pydantic import BaseModel, Field, field_validator
from dirigo import units
from dirigo.hw_interfaces.scanner import ObjectiveZScanner

from dirigo_trinamic.enumerations import *
from dirigo_trinamic.controller import TrinamicController



class TrinamicObjectiveZScannerConfig(BaseModel):
    axis: Literal["z"] = Field(
        default     = "z", 
        description = "Which logical axis this actuator controls"
    )
    com_port: int = Field(
        ge          = 1, 
        description = "Windows COM port number (e.g. 5 for COM5)")

    # Controller settings
    module_address: int = Field(
        default     = 1, 
        ge          = 1, 
        description = "Module address (always 1 unless multiple daisy-chained axes)")
    baud_rate: BaudRates = Field(
        default = BaudRates.BAUD_9600
    )

    # Mechanics
    step_angle: units.Angle = Field(
        description = 'Stepper full-step angle, e.g. "1.8 deg".'
    )
    travel_per_rev: units.Position = Field(
        description='Linear travel per motor rev, e.g. "100 um".'
    )

    @field_validator("step_angle", mode="before")
    @classmethod
    def _parse_step_angle(cls, v):
        return v if isinstance(v, units.Angle) else units.Angle(v)

    @field_validator("travel_per_rev", mode="before")
    @classmethod
    def _parse_travel_per_rev(cls, v):
        return v if isinstance(v, units.Position) else units.Position(v)


class TrinamicObjectiveZScanner(ObjectiveZScanner):
    config_model = TrinamicObjectiveZScannerConfig

    MAX_TMC_VELOCITY = 2047
    MAX_TMC_ACCELERATION = 2047

    def __init__(self, 
                 com_port: int, 
                 step_angle: str, 
                 travel_per_rev: str, 
                 module_address: int = 1, 
                 **kwargs):
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

        if return_code is None:
            raise RuntimeError("Could not get microstep resolution code")
        return 2 ** return_code # increasing powers of 2
        
    @cached_property
    def _distance_per_microstep(self) -> units.Position:
        """The linear travel per microstep."""
        normalized_step_angle = self._step_angle / units.Angle("360 deg")
        travel_per_step = self._travel_per_rev * normalized_step_angle
        
        return travel_per_step / self._microstep_resolution

    @property
    def position(self):
        """The current (actual) position."""
        value = self._controller.send_receive(self._position_cmd)
        return value * self._distance_per_microstep
    
    @cached_property
    def _position_cmd(self) -> bytearray:
        cmd = self._controller.make_command(
            instruction=ParameterCommands.GET_AXIS_PARAMETER, 
            type=AxisParameters.ACTUAL_POSITION
        )
        return cmd
    
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
        if not (0 < velo_trinamic <= self.MAX_TMC_VELOCITY):
            raise ValueError(f"Attempted to set stepper velocity outside of "
                             f"allowable range (0-{self.MAX_TMC_VELOCITY}). Got: {velo_trinamic}")

        cmd = self._controller.make_command(
            instruction=ParameterCommands.SET_AXIS_PARAMETER, 
            type=AxisParameters.MAX_POSITIONING_SPEED,
            operand=velo_trinamic
        )
        self._controller.send_receive(cmd)

    @cached_property
    def _velocity_factor(self):
        """Ratio of microsteps per second to internal Trinamic velocity.
        """
        cmd = self._controller.make_command(
            instruction=ParameterCommands.GET_AXIS_PARAMETER, 
            type=AdvancedAxisParameters.PULSE_DIVISOR
        )
        pulse_divisor = self._controller.send_receive(cmd)
        if pulse_divisor is None:
            raise RuntimeError("Could not get pulse divisor code")

        # Source: TMCM-140-42-SE Hardware Manual V1.05, page 20
        return 16e6 / (2 ** pulse_divisor * 2048 * 32)

    @property
    def acceleration(self) -> units.Acceleration:
        """
        Acceleration used during ramp up/down phase of move.
        """
        cmd = self._controller.make_command(
            instruction=ParameterCommands.GET_AXIS_PARAMETER, 
            type=AxisParameters.MAX_ACCELERATION
        )
        accel_trinamic = self._controller.send_receive(cmd)
        if accel_trinamic is None:
            raise RuntimeError("Could not get acceleration parameter")

        accel_trinamic = accel_trinamic * self._acceleration_factor
        return units.Acceleration(accel_trinamic * float(self._distance_per_microstep))

    @acceleration.setter
    def acceleration(self, value: units.Acceleration):
        if not isinstance(value, units.Acceleration):
            raise ValueError("`acceleration` must be set with an Acceleration object.")
        accel_microsteps = float(value) / float(self._distance_per_microstep)
        accel_trinamic = round(accel_microsteps / self._acceleration_factor)
        if not (0 < accel_trinamic <= self.MAX_TMC_ACCELERATION):
            raise ValueError(f"Attempted to set stepper velocity outside of "
                             f"allowable range (0-{self.MAX_TMC_ACCELERATION}). Got: {accel_trinamic}")
        
        cmd = self._controller.make_command(
            instruction=ParameterCommands.SET_AXIS_PARAMETER, 
            type=AxisParameters.MAX_ACCELERATION,
            operand=accel_trinamic
        )
        self._controller.send_receive(cmd)
    
    @cached_property
    def _acceleration_factor(self) -> int:
        cmd = self._controller.make_command(
            instruction=ParameterCommands.GET_AXIS_PARAMETER, 
            type=AdvancedAxisParameters.RAMP_DIVISOR
        )
        ramp_divisor = self._controller.send_receive(cmd)
        cmd = self._controller.make_command(
            instruction=ParameterCommands.GET_AXIS_PARAMETER, 
            type=AdvancedAxisParameters.PULSE_DIVISOR
        )
        pulse_divisor = self._controller.send_receive(cmd)
        if pulse_divisor is None:
            raise RuntimeError("Could not get pulse divisor")

        # Source: TMCM-140-42-SE Hardware Manual V1.05, page 20
        return (16e6)**2 / 2**(ramp_divisor + pulse_divisor + 29)
    
    @property
    def device_info(self) -> None:
        """Returns an object describing permanent properties of the stage."""
        pass

    @property
    def position_limits(self) -> units.RangeWithUnits:
        """Returns an object describing the stage movement limits."""
        raise NotImplementedError

    @property
    def moving(self) -> bool:   
        """Return True if the stage axis is currently moving."""
        cmd = self._controller.make_command(
            instruction=ParameterCommands.GET_AXIS_PARAMETER, 
            type=AxisParameters.ACTUAL_SPEED
        )
        actual_speed = self._controller.send_receive(cmd)
        return bool(actual_speed)

    def move_to(self, position: units.Position, blocking: bool = False) -> None:
        """
        Initiate move to specified spatial position.

        Choose whether to return immediately (blocking=False, default) or to
        wait until finished moving (blocking=True).
        """
        cmd = self._controller.make_command(
            instruction=MotionCommands.MOVE_TO_POSITION, 
            type=MoveTypes.ABSOLUTE, 
            operand=round(position / self._distance_per_microstep)
        )
        self._controller.send_receive(cmd)

    def move_relative(self, move_distance: units.Position) -> None:
        """Moves a distance relative the current position."""
        cmd = self._controller.make_command(
            instruction=MotionCommands.MOVE_TO_POSITION, 
            type=MoveTypes.RELATIVE, 
            operand=round(move_distance / self._distance_per_microstep)
        )
        self._controller.send_receive(cmd)
    
    def move_velocity(self, velocity: units.Velocity) -> None:
        """"
        Initiate movement at velocity until stopped.
        """
        if not isinstance(velocity, units.Velocity):
            raise ValueError("`move_velocity()` argument must of be type Velocity.")
        
        instruction = MotionCommands.ROTATE_RIGHT if velocity > 0 \
            else MotionCommands.ROTATE_LEFT
        
        velocity_microsteps = abs(float(velocity)) / self._distance_per_microstep
        velocity_trinamic = round(velocity_microsteps / self._velocity_factor)

        if not (0 < velocity_trinamic <= self.MAX_TMC_VELOCITY):
            raise ValueError(f"Attempted to set stepper velocity outside of "
                             f"allowable range (0-{self.MAX_TMC_VELOCITY}). Got: {velocity_trinamic}")
        
        cmd = self._controller.make_command(
            instruction=instruction, 
            operand=velocity_trinamic
        )
        self._controller.send_receive(cmd)

    def stop(self) -> None:
        """Halts motion."""
        cmd = self._controller.make_command(
            instruction=MotionCommands.MOTOR_STOP 
        )
        self._controller.send_receive(cmd)

    def home(self, blocking: bool = False):
        """
        Initiate homing. 
        
        Choose whether to return immediately (blocking=False, default) or to
        wait until finished homing (blocking=True).
        """
        raise NotImplementedError

    @property
    def homed(self) -> bool:
        """Return whether the stage has been home."""
        return False # not currently possible
        

