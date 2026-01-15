# Design Document: Motor Simulation System

## Overview

The motor simulation system models a realistic servo motor with torque control, encoder feedback, and friction effects. The design uses a time-stepped simulation approach where all components update synchronously at 1000Hz (1ms time steps). The system maintains separate state for commanded vs. applied torque to model the 1ms actuation delay, and quantizes continuous position to simulate realistic encoder behavior.

## Architecture

The system follows a modular architecture with four main components:

1. **Motor** - Central class that orchestrates all components and maintains motor state
2. **TorqueController** - Manages torque commands with 1ms delay buffering
3. **Encoder** - Quantizes continuous position to 16-bit discrete values
4. **FrictionModel** - Calculates friction torque (pluggable interface)

The Motor class acts as the coordinator, calling each component in sequence during each simulation step:
1. Update torque controller (apply delayed command)
2. Calculate friction torque
3. Compute net torque and integrate dynamics
4. Update encoder with new position

## Components and Interfaces

### Motor Class

The main simulation class that maintains motor state and coordinates updates.

**State Variables:**
- `position: float` - Current angular position in radians (continuous)
- `velocity: float` - Current angular velocity in rad/s
- `inertia: float` - Rotational inertia in kg⋅m²
- `time: float` - Current simulation time in seconds
- `dt: float` - Time step (0.001s for 1000Hz)

**Methods:**
```python
__init__(inertia: float, friction_model: FrictionModel = None)
  # Initialize motor with given inertia and optional friction model
  
set_torque_command(torque: float) -> None
  # Queue a torque command to be applied after 1ms delay
  
step() -> None
  # Advance simulation by one time step (1ms)
  # Updates torque, friction, dynamics, and encoder
  
get_position() -> float
  # Returns continuous position in radians
  
get_encoder_position() -> int
  # Returns quantized encoder reading (0-65535)
  
get_velocity() -> float
  # Returns current velocity in rad/s
  
get_applied_torque() -> float
  # Returns the torque currently being applied (after delay)
```

### TorqueController Class

Manages torque commands with realistic 1ms delay.

**State Variables:**
- `current_torque: float` - Torque currently being applied
- `pending_torque: float` - Torque command waiting to be applied
- `delay_timer: float` - Time remaining until pending command is applied

**Methods:**
```python
__init__()
  # Initialize with zero torque
  
command(torque: float) -> None
  # Queue new torque command with 1ms delay
  
update(dt: float) -> float
  # Advance delay timer and return current applied torque
```

### Encoder Class

Quantizes continuous position to 16-bit discrete values.

**Constants:**
- `RESOLUTION: int = 65536` - 16-bit resolution (2^16)
- `COUNTS_PER_RADIAN: float = RESOLUTION / (2 * π)`

**Methods:**
```python
quantize(position: float) -> int
  # Convert continuous position to quantized encoder count
  # Maps [0, 2π) to [0, 65535], wraps around for multiple rotations
  
to_radians(counts: int) -> float
  # Convert encoder counts back to approximate radians
```

### FrictionModel Interface

Abstract interface for friction calculation, allowing different friction models to be plugged in.

**Methods:**
```python
calculate_friction(velocity: float, position: float) -> float
  # Calculate friction torque based on motor state
  # Returns torque in N⋅m (opposing motion)
```

### DefaultFrictionModel Class

Simple viscous + Coulomb friction model used as placeholder.

**Parameters:**
- `viscous_coefficient: float` - Viscous friction coefficient (N⋅m⋅s/rad)
- `coulomb_torque: float` - Coulomb (static) friction torque (N⋅m)

**Implementation:**
```python
calculate_friction(velocity: float, position: float) -> float
  # friction = viscous_coefficient * velocity + sign(velocity) * coulomb_torque
  # Returns torque opposing motion
```

## Data Models

### Motor State

The motor maintains continuous state that evolves according to rigid body dynamics:

```
angular_acceleration = (applied_torque - friction_torque) / inertia
velocity = velocity + angular_acceleration * dt
position = position + velocity * dt
```

### Encoder Quantization

The encoder maps continuous position to discrete counts:

```
normalized_position = position % (2π)  # Wrap to [0, 2π)
encoder_counts = floor(normalized_position * COUNTS_PER_RADIAN)
encoder_counts = encoder_counts % 65536  # Ensure 16-bit range
```

### Torque Delay Buffer

The torque controller maintains a simple delay mechanism:

```
When new command received:
  pending_torque = commanded_torque
  delay_timer = 0.001  # 1ms

Each update:
  delay_timer -= dt
  if delay_timer <= 0:
    current_torque = pending_torque
    delay_timer = 0
```


## Correctness Properties

A property is a characteristic or behavior that should hold true across all valid executions of a system—essentially, a formal statement about what the system should do. Properties serve as the bridge between human-readable specifications and machine-verifiable correctness guarantees.

### Property 1: Torque Command Acceptance

*For any* floating-point torque value, the Motor_System should accept the command without error and store it for delayed application.

**Validates: Requirements 1.1**

### Property 2: Torque Delay Timing

*For any* torque command value, after commanding the torque and stepping the simulation exactly once (1ms), the applied torque should equal the commanded torque.

**Validates: Requirements 1.2**

### Property 3: Command Overwriting

*For any* sequence of torque commands issued before stepping the simulation, only the most recent command should be applied after the 1ms delay.

**Validates: Requirements 1.4**

### Property 4: Encoder Quantization Validity

*For any* continuous position value, the encoder should return a quantized value in the valid 16-bit range [0, 65535], and nearby continuous positions should map to nearby encoder values (within quantization error).

**Validates: Requirements 2.1, 2.3**

### Property 5: Encoder Scaling

*For any* position value, the encoder should map a full rotation (2π radians) to the full 16-bit range, such that position 0 maps near count 0, and position 2π maps near count 0 (wrapping).

**Validates: Requirements 2.4**

### Property 6: Encoder Read Consistency

*For any* motor state, calling get_encoder_position multiple times without stepping should return the same encoder value.

**Validates: Requirements 2.5**

### Property 7: Friction Opposition

*For any* motor state with non-zero velocity, the friction torque should oppose the direction of motion (negative friction when velocity is positive, positive friction when velocity is negative).

**Validates: Requirements 3.2, 3.4**

### Property 8: Torque Combination

*For any* motor state during a simulation step, the net torque used for acceleration should equal the applied torque minus the friction torque.

**Validates: Requirements 4.2**

### Property 9: Physics Integration

*For any* motor state and applied torques, after one simulation step:
- The change in velocity should equal (net_torque / inertia) * dt
- The change in position should equal velocity * dt (using appropriate integration method)

**Validates: Requirements 4.4**

### Property 10: Update Sequence Correctness

*For any* initial motor state and torque command, after stepping the simulation, all components (torque controller, friction model, dynamics, encoder) should reflect a consistent updated state where the encoder reading corresponds to the integrated position.

**Validates: Requirements 5.2**

### Property 11: Delay Independence

*For any* torque command, the 1ms delay should be tracked independently such that stepping the simulation once after commanding applies the torque, regardless of the motor's current state.

**Validates: Requirements 5.3**

## Error Handling

The motor simulation system should handle several error conditions gracefully:

1. **Invalid Inertia**: If inertia ≤ 0 is provided during initialization, raise ValueError
2. **Numerical Stability**: If velocity or position values become NaN or infinite, raise RuntimeError
3. **Friction Model Errors**: If a custom friction model raises an exception, propagate it with context about the motor state
4. **Encoder Overflow**: Handle position values outside [0, 2π) by wrapping using modulo operation

The system should validate inputs at construction time and maintain numerical stability during integration. Since this is a simulation (not safety-critical control), errors should fail fast with clear messages rather than attempting recovery.

## Testing Strategy

The motor simulation system will be tested using a dual approach combining unit tests and property-based tests.

### Unit Testing

Unit tests will verify specific examples and edge cases:

- **Initialization**: Test motor creation with valid and invalid parameters
- **Zero Motion**: Test that a motor with no torque command remains stationary
- **Single Step**: Test that one simulation step produces expected state changes
- **Encoder Wrapping**: Test encoder behavior at position boundaries (0, 2π, 4π)
- **Friction Edge Cases**: Test friction at zero velocity and direction changes
- **Default Friction Model**: Test that motor works without explicit friction model (Requirement 3.5)
- **Dual Position Access**: Test that both continuous and encoder positions are accessible (Requirement 4.5)

### Property-Based Testing

Property-based tests will verify universal properties across many randomly generated inputs using the **Hypothesis** library for Python. Each test will run a minimum of 100 iterations.

The following properties from the Correctness Properties section will be implemented as property-based tests:

- **Property 1**: Torque Command Acceptance - Generate random torque values
- **Property 2**: Torque Delay Timing - Generate random torques and verify timing
- **Property 3**: Command Overwriting - Generate sequences of commands
- **Property 4**: Encoder Quantization Validity - Generate random positions
- **Property 5**: Encoder Scaling - Generate positions across multiple rotations
- **Property 6**: Encoder Read Consistency - Generate random motor states
- **Property 7**: Friction Opposition - Generate random velocities
- **Property 8**: Torque Combination - Generate random torque and friction values
- **Property 9**: Physics Integration - Generate random initial states and torques
- **Property 10**: Update Sequence Correctness - Generate random states and commands
- **Property 11**: Delay Independence - Generate random commands and states

Each property test will be tagged with a comment in the format:
```python
# Feature: motor-control, Property N: [property description]
```

### Test Configuration

- Property tests: Minimum 100 iterations per test
- Test framework: pytest for unit tests, Hypothesis for property-based tests
- Coverage target: >90% line coverage for core motor logic
- Integration tests: Test motor with different friction models to verify pluggability

The combination of unit tests (for specific examples) and property tests (for comprehensive input coverage) ensures both concrete correctness and general behavioral guarantees.
