# pxt-smartmotor

Power Smart Motor Hub extension for MakeCode micro:bit.

The hub uses I2C address `0x66` and responds after it has entered its normal working state. In the simulator or when the hub is not connected, read blocks return safe default values such as `0` or `V 0.0.0`.

## API

| Category | Block | TypeScript |
| --- | --- | --- |
| Motor | 电机 X1 方向为 X2，速度为 X3 启动 | `smartMotor.motorStart(motor, direction, speed)` |
| Motor | 电机 X1 停止 | `smartMotor.motorStop(motor)` |
| Position | 电机 X1 位置归零 | `smartMotor.motorReset(motor)` |
| Position | 电机 X1 转动到绝对角度 X2，速度 X3 | `smartMotor.motorMoveAbsolute(motor, angle, speed)` |
| Position | 电机 X1 转动角度为 X2，速度 X3 | `smartMotor.motorMoveRelative(motor, angle, speed)` |
| Robot | 机器人车轮直径为 X1 毫米 | `smartMotor.robotSetWheelDiameter(diameter)` |
| Robot | 机器人左轮 X1 和右轮 X2 | `smartMotor.robotSetMotors(leftMotor, rightMotor)` |
| Robot | 机器人转向 X1 度，速度 X2，加速度 X3 | `smartMotor.robotTurn(angle, speed, accel)` |
| Robot | 机器人朝 X1 直行 X2 X3，加速度 X4 | `smartMotor.robotDriveStraight(direction, value, mode, accel)` |
| Robot | 机器人停止 | `smartMotor.robotStop()` |
| Readings | 电机 X1 当前速度（度/秒） | `smartMotor.motorGetSpeed(motor)` |
| Readings | 电机 X1 绝对角度 | `smartMotor.motorGetAbsoluteAngle(motor)` |
| Readings | 电机 X1 相对角度 | `smartMotor.motorGetRelativeAngle(motor)` |
| Gyroscope | 陀螺仪重置 | `smartMotor.resetGyroAngle()` |
| Gyroscope | 陀螺仪 X1 角速度（°/s） | `smartMotor.readGyroAngularSpeed(axis)` |
| Gyroscope | 陀螺仪 X1 角（°） | `smartMotor.readGyroAngle(axis)` |
| Information | 固件版本号 | `smartMotor.readVersion()` |

## Parameters

- Motor ports are `M5`, `M6`, `M7`, and `M8`.
- Motor direction is clockwise or counterclockwise.
- Motor speed is an integer from `-100` to `100`.
- Absolute and relative motor angle blocks accept `0` to `360` degrees.
- Wheel diameter accepts `0` to `10000` millimeters.
- Robot turn angle accepts `-360` to `360` degrees, and robot turn speed accepts `0` to `100`.
- Robot straight mode is millimeters, seconds, or degrees. Acceleration is slow, medium, or fast.
- Gyroscope axis is pitch, yaw, or roll. Angular speed is estimated from consecutive angle samples.

## Example

```typescript
smartMotor.motorStart(smartMotor.MotorPort.M5, smartMotor.MotorDirection.Clockwise, 50)
basic.pause(1000)
smartMotor.motorStop(smartMotor.MotorPort.M5)

smartMotor.motorReset(smartMotor.MotorPort.M5)
smartMotor.motorMoveRelative(smartMotor.MotorPort.M5, 90, 50)
smartMotor.motorMoveAbsolute(smartMotor.MotorPort.M5, 180, 50)

smartMotor.robotSetWheelDiameter(62)
smartMotor.robotSetMotors(smartMotor.MotorPort.M5, smartMotor.MotorPort.M6)
smartMotor.robotTurn(90, 50, smartMotor.AccelLevel.Medium)
smartMotor.robotDriveStraight(smartMotor.DriveDirection.Forward, 200, smartMotor.DriveMode.Millimeters, smartMotor.AccelLevel.Medium)
smartMotor.robotStop()

let speed = smartMotor.motorGetSpeed(smartMotor.MotorPort.M5)
let absoluteAngle = smartMotor.motorGetAbsoluteAngle(smartMotor.MotorPort.M5)
let relativeAngle = smartMotor.motorGetRelativeAngle(smartMotor.MotorPort.M5)
let yaw = smartMotor.readGyroAngle(smartMotor.GyroAxis.Yaw)
let yawSpeed = smartMotor.readGyroAngularSpeed(smartMotor.GyroAxis.Yaw)
let version = smartMotor.readVersion()
```

## Test

Run `pxt build` in this extension directory. A pass means `main.ts`, `test.ts`, and localized block metadata compile for the micro:bit target.

## Use as Extension

Import this repository in MakeCode micro:bit:

```text
https://github.com/zy2516/pxt-smartmotor
```
