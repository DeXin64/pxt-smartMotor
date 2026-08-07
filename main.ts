//% color=#ff0011 icon="\uf1b9" block="Smart Motor"
namespace smartMotor {
    const I2C_ADDRESS = 0x66
    const I2C_REGISTER_PREPARE_DELAY_MS = 2
    const MOTOR_DATA_REFRESH_DELAY_MS = 10
    const COMMAND_REGISTER_READ = 0x01
    const COMMAND_MOTOR_DATA_REFRESH = 0x02
    const COMMAND_VERSION = 0x10
    const COMMAND_SET_SPEED = 0x20
    const COMMAND_STOP = 0x21
    const COMMAND_MOVE = 0x22
    const COMMAND_MOVE_ABSOLUTE = 0x23
    const COMMAND_RESET_PHYSICAL = 0x24
    const COMMAND_RESET_RELATIVE = 0x25
    const COMMAND_ROBOT_SET_SPEED = 0x26
    const COMMAND_ROBOT_MOVE = 0x27
    const REGISTER_GYRO_ANGLE_START = 0x03
    const REGISTER_ACCELERATION_START = 0x0F
    const REGISTER_MOTOR_ERROR_START = 0x15
    const MOTOR_DATA_RECORD_LENGTH = 13
    const MOTOR_DATA_ANGLE_VALID = 0x01
    const MOTOR_DATA_SPEED_VALID = 0x02
    const MOTOR_DATA_RELATIVE_ANGLE_OFFSET = 1
    const MOTOR_DATA_ABSOLUTE_ANGLE_OFFSET = 5
    const MOTOR_DATA_SPEED_OFFSET = 9
    const MOTOR_DATA_ANGLE_SEQUENCE_OFFSET = 11
    const MOTOR_DATA_SPEED_SEQUENCE_OFFSET = 12
    const MOTOR_DATA_REFRESH_ANGLE = 0x01
    const MOTOR_DATA_REFRESH_SPEED = 0x02
    const MOTION_START_DELAY_MS = 100
    const MOTOR_WAIT_POLL_INTERVAL_MS = 10
    const MOTION_POLL_INTERVAL_MS = 20
    const MOTION_TARGET_TOLERANCE_X10 = 10
    const MOTION_START_ANGLE_DELTA_X10 = 5
    const MOTION_STOP_SAMPLE_COUNT = 2
    const MOTION_MIN_TIMEOUT_MS = 2000
    const MOTION_MAX_TIMEOUT_MS = 60000
    const RESET_WAIT_TIMEOUT_MS = 5000
    const ROBOT_TURN_TOLERANCE_DEGREES = 1
    const ROBOT_DEFAULT_WHEEL_DIAMETER_MM = 62

    /** 电机接口位置。 */
    export enum MotorPort {
        //% block="M1"
        M1 = 1,
        //% block="M2"
        M2 = 2,
        //% block="M3"
        M3 = 3,
        //% block="M4"
        M4 = 4
    }

    /** 单电机转动方向。 */
    export enum TurnDirection {
        //% block="clockwise"
        CW = 1,
        //% block="counterclockwise"
        CCW = 2
    }

    /** 绝对角度运动方向。 */
    export enum TurnDirectionEx {
        //% block="clockwise"
        CW = 2,
        //% block="counterclockwise"
        CCW = 3,
        //% block="shortest path"
        ShortestPath = 1
    }

    /** 单电机相对运动单位。 */
    export enum TurnMode {
        //% block="seconds"
        Second = 3,
        //% block="degrees"
        Degree = 2,
        //% block="turns"
        Circle = 1
    }

    /** 机器人直行方向。 */
    export enum DriveDirection {
        //% block="forward"
        Forward = 0,
        //% block="backward"
        Backward = 1
    }

    /** 机器人直行距离或时间单位。 */
    export enum DriveMode {
        //% block="seconds"
        Second = 0,
        //% block="millimeters"
        Millimeter = 1,
        //% block="centimeters"
        Centimeter = 2
    }

    /** 是否在积木返回前等待电机反馈表明运动已经完成。 */
    export enum WaitMode {
        //% block="do not wait"
        NoWait = 0,
        //% block="wait until done"
        Wait = 1
    }

    /** 单路电机当前通信错误状态。 */
    export enum MotorErrorCode {
        None = 0,
        HeartbeatTimeout = 1,
        CommandResponseTimeout = 2,
        TransmitTimeout = 3,
        Unknown = 255
    }

    /** 陀螺仪和加速度数据使用的三轴方向。 */
    export enum SensorAxis {
        //% block="X"
        X = 0,
        //% block="Y"
        Y = 1,
        //% block="Z"
        Z = 2
    }

    let robotLeftMotor = MotorPort.M1
    let robotRightMotor = MotorPort.M2
    let robotWheelDiameterMm = ROBOT_DEFAULT_WHEEL_DIAMETER_MM
    let robotMotionId = 0
    let robotTurnActive = false

    /** 从小端字节流读取有符号16位值。 */
    function readI16Le(buffer: Buffer, offset: number): number {
        let value = buffer[offset] | (buffer[offset + 1] << 8)
        return value >= 0x8000 ? value - 0x10000 : value
    }

    /** 从小端字节流读取有符号32位值。 */
    function readI32Le(buffer: Buffer, offset: number): number {
        return (buffer[offset])
            | (buffer[offset + 1] << 8)
            | (buffer[offset + 2] << 16)
            | (buffer[offset + 3] << 24)
    }

    /** 将速度百分比限制到指定范围并换算为整数。 */
    function clamp(value: number, minimum: number, maximum: number): number {
        return value < minimum ? minimum : value > maximum ? maximum : value
    }

    /** 按参考工程方式忙等待短暂的I2C从机响应准备时间。 */
    function delayMs(ms: number): void {
        let endTime = input.runningTime() + ms
        while (endTime > input.runningTime()) {
        }
    }

    /** 按Cutebot Pro协议发送一条I2C指令，并忙等待接收或查询数据准备。 */
    function i2cCommandSend(command: number, data: number[], delay: number = 1): void {
        let frame = pins.createBuffer(data.length + 4)
        frame[0] = 0xFF
        frame[1] = 0xF9
        frame[2] = command
        frame[3] = data.length
        for (let index = 0; index < data.length; index++) {
            frame[index + 4] = data[index]
        }
        pins.i2cWriteBuffer(I2C_ADDRESS, frame)
        delayMs(delay)
    }

    /** 通知下位机按需刷新请求范围，等待2ms后直接读取原始寄存器。 */
    function readRegisters(startAddress: number, length: number): Buffer {
        let requestLength = clamp(Math.round(length), 1, 24)
        i2cCommandSend(COMMAND_REGISTER_READ, [startAddress, requestLength], I2C_REGISTER_PREPARE_DELAY_MS)
        return pins.i2cReadBuffer(I2C_ADDRESS, requestLength)
    }

    /** 请求下位机刷新单路电机数据，等待UART回复后直接读取最新记录。 */
    function refreshMotorData(motor: MotorPort, dataMask: number): Buffer {
        i2cCommandSend(COMMAND_MOTOR_DATA_REFRESH, [motor, dataMask], MOTOR_DATA_REFRESH_DELAY_MS)
        let motorData = pins.i2cReadBuffer(I2C_ADDRESS, MOTOR_DATA_RECORD_LENGTH)
        if ((motorData[0] & dataMask) != dataMask) {
            delayMs(5)
            motorData = pins.i2cReadBuffer(I2C_ADDRESS, MOTOR_DATA_RECORD_LENGTH)
        }
        if ((motorData[0] & dataMask) != dataMask) {
            delayMs(5)
            motorData = pins.i2cReadBuffer(I2C_ADDRESS, MOTOR_DATA_RECORD_LENGTH)
        }
        return motorData
    }

    /** 将任意0.1度角度归一化到0～3599。 */
    function normalizeAngleX10(angleX10: number): number {
        let normalized = angleX10 % 3600
        return normalized < 0 ? normalized + 3600 : normalized
    }

    /** 按下位机相同的方向规则计算绝对角度命令预计经过的角度。 */
    function absoluteTravelX10(currentX10: number, targetX10: number, turnMode: TurnDirectionEx): number {
        let current = normalizeAngleX10(currentX10)
        let target = normalizeAngleX10(targetX10)
        let clockwise = (target + 3600 - current) % 3600
        let counterClockwise = (current + 3600 - target) % 3600
        if (turnMode == TurnDirectionEx.CW) {
            return clockwise
        }
        if (turnMode == TurnDirectionEx.CCW) {
            return counterClockwise
        }
        return Math.min(clockwise, counterClockwise)
    }

    /** 根据运动量和速度生成带余量且有上下限的反馈等待超时。 */
    function motionTimeoutMs(valueX10: number, mode: TurnMode, speedValue: number): number {
        let estimatedMs = 0
        if (mode == TurnMode.Second) {
            estimatedMs = Math.abs(valueX10) * 100
        } else {
            estimatedMs = Math.abs(valueX10) * 100 / clamp(speedValue, 1, 900)
        }
        return clamp(Math.round(estimatedMs * 2 + 2000), MOTION_MIN_TIMEOUT_MS, MOTION_MAX_TIMEOUT_MS)
    }

    /** 等待角度目标完成且连续两份新速度为零，避免下一条命令提前覆盖。 */
    function waitForMotorFeedback(motor: MotorPort, startData: Buffer, mode: TurnMode, commandValueX10: number, speedValue: number, absoluteTargetX10: number, turnMode: TurnDirectionEx): void {
        let startAngleValid = false
        let startAngleX10 = 0
        let lastAngleSequence = -1
        let lastSpeedSequence = -1
        if (startData.length == MOTOR_DATA_RECORD_LENGTH) {
            startAngleValid = (startData[0] & MOTOR_DATA_ANGLE_VALID) != 0
            if (startAngleValid) {
                startAngleX10 = readI32Le(startData,
                    absoluteTargetX10 >= 0
                        ? MOTOR_DATA_ABSOLUTE_ANGLE_OFFSET
                        : MOTOR_DATA_RELATIVE_ANGLE_OFFSET)
            }
            lastAngleSequence = startData[MOTOR_DATA_ANGLE_SEQUENCE_OFFSET]
            lastSpeedSequence = startData[MOTOR_DATA_SPEED_SEQUENCE_OFFSET]
        }
        let absoluteTarget = absoluteTargetX10 >= 0
        let absoluteTravelMode = turnMode
        let expectedValueX10 = Math.abs(commandValueX10)
        let timeoutMode = mode
        if (absoluteTarget) {
            timeoutMode = TurnMode.Degree
            if (startAngleValid) {
                let targetAngleX10 = normalizeAngleX10(absoluteTargetX10)
                if (turnMode == TurnDirectionEx.ShortestPath) {
                    absoluteTravelMode = absoluteTravelX10(startAngleX10,
                        targetAngleX10, TurnDirectionEx.CW)
                        <= absoluteTravelX10(startAngleX10,
                            targetAngleX10, TurnDirectionEx.CCW)
                        ? TurnDirectionEx.CW : TurnDirectionEx.CCW
                }
                expectedValueX10 = absoluteTravelX10(startAngleX10,
                    targetAngleX10, absoluteTravelMode)
            } else {
                expectedValueX10 = 3600
            }
        }
        let timeoutMs = motionTimeoutMs(expectedValueX10, timeoutMode, speedValue)
        let waitStartMs = input.runningTime()
        let referenceAngleValid = startAngleValid
        let referenceAngleX10 = startAngleX10
        let targetReached = absoluteTarget && startAngleValid
            && expectedValueX10 <= MOTION_TARGET_TOLERANCE_X10
        let motionObserved = targetReached
        let stoppedSamples = 0
        basic.pause(MOTION_START_DELAY_MS)
        while (input.runningTime() - waitStartMs < timeoutMs) {
            let motorData = refreshMotorData(motor, motionObserved && (mode == TurnMode.Second || targetReached) ? MOTOR_DATA_REFRESH_SPEED : MOTOR_DATA_REFRESH_ANGLE | MOTOR_DATA_REFRESH_SPEED)
            if (motorData.length == MOTOR_DATA_RECORD_LENGTH) {
                let flags = motorData[0]
                let angleSequence = motorData[MOTOR_DATA_ANGLE_SEQUENCE_OFFSET]
                if ((flags & MOTOR_DATA_ANGLE_VALID) != 0
                    && angleSequence != lastAngleSequence) {
                    let currentAngleX10 = readI32Le(motorData,
                        absoluteTarget
                            ? MOTOR_DATA_ABSOLUTE_ANGLE_OFFSET
                            : MOTOR_DATA_RELATIVE_ANGLE_OFFSET)
                    lastAngleSequence = angleSequence
                    if (!referenceAngleValid) {
                        referenceAngleValid = true
                        referenceAngleX10 = currentAngleX10
                    } else if (Math.abs(currentAngleX10 - referenceAngleX10)
                        >= MOTION_START_ANGLE_DELTA_X10) {
                        motionObserved = true
                    }
                    if (!targetReached && startAngleValid
                        && mode != TurnMode.Second) {
                        if (absoluteTarget) {
                            targetReached = absoluteTravelX10(startAngleX10,
                                currentAngleX10, absoluteTravelMode)
                                + MOTION_TARGET_TOLERANCE_X10
                                >= expectedValueX10
                        } else if (commandValueX10 >= 0) {
                            targetReached = currentAngleX10
                                + MOTION_TARGET_TOLERANCE_X10
                                >= startAngleX10 + commandValueX10
                        } else {
                            targetReached = currentAngleX10
                                - MOTION_TARGET_TOLERANCE_X10
                                <= startAngleX10 + commandValueX10
                        }
                    }
                }
                let speedSequence = motorData[MOTOR_DATA_SPEED_SEQUENCE_OFFSET]
                if ((flags & MOTOR_DATA_SPEED_VALID) != 0
                    && speedSequence != lastSpeedSequence) {
                    let currentSpeed = readI16Le(motorData, MOTOR_DATA_SPEED_OFFSET)
                    lastSpeedSequence = speedSequence
                    if (currentSpeed == 0) {
                        if (motionObserved
                            || (mode == TurnMode.Second
                                && input.runningTime() - waitStartMs
                                >= Math.abs(commandValueX10) * 100)) {
                            stoppedSamples++
                        } else {
                            stoppedSamples = 0
                        }
                    } else {
                        stoppedSamples = 0
                        motionObserved = true
                    }
                }
                if (stoppedSamples >= MOTION_STOP_SAMPLE_COUNT&& (mode == TurnMode.Second || targetReached || !startAngleValid)) {
                    return
                }
            }
            basic.pause(MOTOR_WAIT_POLL_INTERVAL_MS)
        }
    }

    /** 取消机器人转向，并按需停止其正在驱动的左右轮。 */
    function cancelRobotMotion(stopActiveMotors: boolean = true): void {
        robotMotionId++
        let shouldStop = robotTurnActive && stopActiveMotors
        robotTurnActive = false
        if (shouldStop) {
            i2cCommandSend(COMMAND_STOP, [robotMotorMask() & 0x0F])
        }
    }

    /** 返回当前左右轮对应的电机端口掩码。 */
    function robotMotorMask(): number {
        return (1 << (robotLeftMotor - 1)) | (1 << (robotRightMotor - 1))
    }

    /** 按固定双电机数据一次性设置机器人左右轮速度。 */
    function sendRobotSpeed(leftSpeed: number, rightSpeed: number): void {
        let leftMotorSpeed = -Math.round(clamp(leftSpeed, -100, 100))
        let rightMotorSpeed = Math.round(clamp(rightSpeed, -100, 100))
        let direction = 0
        if (leftMotorSpeed < 0) {
            direction |= 0x01
        }
        if (rightMotorSpeed < 0) {
            direction |= 0x02
        }
        i2cCommandSend(COMMAND_ROBOT_SET_SPEED, [
            robotLeftMotor,
            robotRightMotor,
            Math.abs(leftMotorSpeed),
            Math.abs(rightMotorSpeed),
            direction
        ])
    }

    /** 在物理归零后等待一份新的接近零度的相对角度样本。 */
    function waitForMotorReset(motor: MotorPort, previousSequence: number): void {
        let startMs = input.runningTime()
        basic.pause(MOTION_START_DELAY_MS)
        while (input.runningTime() - startMs < RESET_WAIT_TIMEOUT_MS) {
            let motorData = refreshMotorData(motor, MOTOR_DATA_REFRESH_ANGLE)
            if (motorData.length == MOTOR_DATA_RECORD_LENGTH
                && (motorData[0] & MOTOR_DATA_ANGLE_VALID) != 0
                && motorData[MOTOR_DATA_ANGLE_SEQUENCE_OFFSET] != previousSequence
                && Math.abs(readI32Le(motorData, MOTOR_DATA_RELATIVE_ANGLE_OFFSET))
                <= MOTION_TARGET_TOLERANCE_X10) {
                return
            }
            basic.pause(MOTION_POLL_INTERVAL_MS)
        }
    }

    /** 使用板载Z轴累计角度执行无PID的机器人相对转向。 */
    function runRobotTurn(angle: number, speed: number, motionId: number): void {
        if (motionId != robotMotionId) {
            return
        }
        let startYaw = readGyroAngle(SensorAxis.Z)
        let targetYaw = startYaw + angle
        let turnSpeed = clamp(Math.abs(speed), 1, 100)
        let positiveDirection = angle > 0
        robotTurnActive = true
        sendRobotSpeed(positiveDirection ? turnSpeed : -turnSpeed,
            positiveDirection ? -turnSpeed : turnSpeed)
        if (motionId != robotMotionId) {
            return
        }
        let startMs = input.runningTime()
        let timeoutMs = clamp(Math.round(Math.abs(angle) * 1000 / turnSpeed + 2000),
            MOTION_MIN_TIMEOUT_MS, MOTION_MAX_TIMEOUT_MS)
        while (input.runningTime() - startMs < timeoutMs) {
            if (motionId != robotMotionId) {
                return
            }
            let error = targetYaw - readGyroAngle(SensorAxis.Z)
            if (Math.abs(error) <= ROBOT_TURN_TOLERANCE_DEGREES
                || (positiveDirection && error < 0)
                || (!positiveDirection && error > 0)) {
                i2cCommandSend(COMMAND_STOP, [robotMotorMask() & 0x0F])
                if (motionId == robotMotionId) {
                    robotTurnActive = false
                }
                return
            }
            basic.pause(MOTION_POLL_INTERVAL_MS)
        }
        if (motionId == robotMotionId) {
            i2cCommandSend(COMMAND_STOP, [robotMotorMask() & 0x0F])
            if (motionId == robotMotionId) {
                robotTurnActive = false
            }
        }
    }

    //% group="Motor"
    //% block="start %motor at %speed\\% %direction"
    //% speed.min=0 speed.max=100 speed.defl=50
    //% weight=100
    /** 按指定方向和速度启动单路电机。 */
    export function motorStart(motor: MotorPort, speed: number, direction: TurnDirection): void {
        cancelRobotMotion()
        let speedPercent = Math.round(clamp(speed, 0, 100))
        i2cCommandSend(COMMAND_SET_SPEED,
            [motor, speedPercent, direction == TurnDirection.CCW ? 1 : 0])
    }

    //% group="Motor"
    //% block="stop %motor"
    //% weight=99
    /** 最高优先级停止指定电机并取消其待执行运动。 */
    export function motorStop(motor: MotorPort): void {
        cancelRobotMotion()
        i2cCommandSend(COMMAND_STOP, [(1 << (motor - 1)) & 0x0F])
    }

    //% group="Motor"
    //% block="reset position of %motor %waitMode"
    //% weight=98
    /** 将电机内部物理位置归零，并可等待新角度样本确认完成。 */
    export function motorReset(motor: MotorPort, waitMode: WaitMode = 0): void {
        cancelRobotMotion()
        let previousSequence = 0
        if (waitMode == WaitMode.Wait) {
            let motorData = refreshMotorData(motor, MOTOR_DATA_REFRESH_ANGLE)
            if (motorData.length == MOTOR_DATA_RECORD_LENGTH) {
                previousSequence = motorData[MOTOR_DATA_ANGLE_SEQUENCE_OFFSET]
            }
        }
        i2cCommandSend(COMMAND_RESET_PHYSICAL, [(1 << (motor - 1)) & 0x0F])
        if (waitMode == WaitMode.Wait) {
            waitForMotorReset(motor, previousSequence)
        }
    }

    //% group="Motor"
    //% block="move %motor %value %mode at %speed\\% %direction %waitMode"
    //% speed.min=1 speed.max=100 speed.defl=50 value.min=0
    //% inlineInputMode=inline
    //% weight=97
    /** 按秒、角度或圈数控制单路电机相对运动。 */
    export function motorMoveRelative(motor: MotorPort, value: number, mode: TurnMode, speed: number, direction: TurnDirection, waitMode: WaitMode = 0): void {
        if (speed <= 0 || value <= 0) {
            return
        }
        cancelRobotMotion()
        let speedPercent = Math.round(clamp(speed, 1, 100))
        let valueX10 = Math.round(value * (mode == TurnMode.Circle ? 3600 : 10))
        let startData = pins.createBuffer(0)
        if (waitMode == WaitMode.Wait) {
            startData = refreshMotorData(motor,
                MOTOR_DATA_REFRESH_ANGLE | MOTOR_DATA_REFRESH_SPEED)
        }
        i2cCommandSend(COMMAND_MOVE, [
            motor,
            mode,
            (valueX10 >> 24) & 0xFF,
            (valueX10 >> 16) & 0xFF,
            (valueX10 >> 8) & 0xFF,
            valueX10 & 0xFF,
            speedPercent,
            direction == TurnDirection.CCW ? 1 : 0
        ])
        if (waitMode == WaitMode.Wait) {
            waitForMotorFeedback(motor, startData, mode,
                direction == TurnDirection.CCW ? -valueX10 : valueX10,
                speedPercent * 9, -1, TurnDirectionEx.ShortestPath)
        }
    }

    //% group="Motor"
    //% block="move %motor to absolute angle %angle degrees at %speed\\% via %direction %waitMode"
    //% angle.min=0 angle.max=359 speed.min=1 speed.max=100 speed.defl=50
    //% inlineInputMode=inline
    //% weight=96
    /** 按指定路径和速度转到单圈绝对角度。 */
    export function motorMoveAbsolute(motor: MotorPort, angle: number, speed: number, direction: TurnDirectionEx, waitMode: WaitMode = 0): void {
        if (speed <= 0) {
            return
        }
        cancelRobotMotion()
        let normalized = normalizeAngleX10(Math.round(angle * 10))
        let speedPercent = Math.round(clamp(speed, 1, 100))
        let startData = pins.createBuffer(0)
        if (waitMode == WaitMode.Wait) {
            startData = refreshMotorData(motor,
                MOTOR_DATA_REFRESH_ANGLE | MOTOR_DATA_REFRESH_SPEED)
        }
        i2cCommandSend(COMMAND_MOVE_ABSOLUTE, [
            motor,
            (normalized >> 8) & 0xFF,
            normalized & 0xFF,
            speedPercent,
            direction == TurnDirectionEx.CW ? 0
                : direction == TurnDirectionEx.CCW ? 1 : 2
        ])
        if (waitMode == WaitMode.Wait) {
            waitForMotorFeedback(motor, startData, TurnMode.Degree, 0,
                speedPercent * 9, normalized, direction)
        }
    }

    //% group="Motor"
    //% block="%motor speed (degrees/s)"
    //% weight=95
    /** 主动刷新并读取单路电机速度。 */
    export function motorGetSpeed(motor: MotorPort): number {
        let motorData = refreshMotorData(motor, MOTOR_DATA_REFRESH_SPEED)
        if (motorData.length != MOTOR_DATA_RECORD_LENGTH
            || (motorData[0] & MOTOR_DATA_SPEED_VALID) == 0) {
            return 0
        }
        return readI16Le(motorData, MOTOR_DATA_SPEED_OFFSET)
    }

    //% group="Motor"
    //% block="%motor relative angle (degrees)"
    //% weight=94
    /** 主动刷新并读取相对物理归零点或下位机相对零点的累计角度。 */
    export function motorGetRelativeAngle(motor: MotorPort): number {
        let motorData = refreshMotorData(motor, MOTOR_DATA_REFRESH_ANGLE)
        if (motorData.length != MOTOR_DATA_RECORD_LENGTH
            || (motorData[0] & MOTOR_DATA_ANGLE_VALID) == 0) {
            return 0
        }
        return readI32Le(motorData, MOTOR_DATA_RELATIVE_ANGLE_OFFSET) / 10
    }

    //% group="Motor"
    //% block="%motor absolute angle (degrees)"
    //% weight=93
    /** 主动刷新并读取归一化到0～359.9度的单圈绝对角度。 */
    export function motorGetAbsoluteAngle(motor: MotorPort): number {
        let motorData = refreshMotorData(motor, MOTOR_DATA_REFRESH_ANGLE)
        if (motorData.length != MOTOR_DATA_RECORD_LENGTH
            || (motorData[0] & MOTOR_DATA_ANGLE_VALID) == 0) {
            return 0
        }
        return normalizeAngleX10(
            readI32Le(motorData, MOTOR_DATA_ABSOLUTE_ANGLE_OFFSET)) / 10
    }

    //% group="Robot"
    //% block="set wheel diameter to %diameter mm"
    //% diameter.min=1 diameter.defl=62
    //% weight=80
    /** 设置机器人距离换算使用的轮子直径，单位为毫米。 */
    export function robotSetWheelDiameter(diameter: number): void {
        if (diameter > 0) {
            robotWheelDiameterMm = diameter
        }
    }

    //% group="Robot"
    //% block="set left motor %leftMotor and right motor %rightMotor"
    //% weight=79
    /** 设置机器人左右轮电机，默认分别为M1和M2。 */
    export function robotSetMotors(leftMotor: MotorPort, rightMotor: MotorPort): void {
        if (leftMotor != rightMotor) {
            cancelRobotMotion()
            robotLeftMotor = leftMotor
            robotRightMotor = rightMotor
        }
    }

    //% group="Robot"
    //% block="set left speed %leftSpeed\\% and right speed %rightSpeed\\%"
    //% leftSpeed.min=-100 leftSpeed.max=100 rightSpeed.min=-100 rightSpeed.max=100
    //% weight=78
    /** 一条固定双电机I2C命令设置机器人左右轮独立速度。 */
    export function robotMove(leftSpeed: number, rightSpeed: number): void {
        cancelRobotMotion()
        sendRobotSpeed(leftSpeed, rightSpeed)
    }

    //% group="Robot"
    //% block="turn robot %angle degrees at %speed\\% %waitMode"
    //% speed.min=1 speed.max=100 speed.defl=50
    //% weight=77
    /** 使用板载Z轴角度控制机器人相对转向，不引入PID调节。 */
    export function robotTurnTo(angle: number, speed: number, waitMode: WaitMode = 0): void {
        if (angle == 0 || speed <= 0) {
            return
        }
        cancelRobotMotion()
        let motionId = robotMotionId
        if (waitMode == WaitMode.Wait) {
            runRobotTurn(angle, speed, motionId)
        } else {
            control.inBackground(function () {
                runRobotTurn(angle, speed, motionId)
            })
        }
    }

    //% group="Robot"
    //% block="drive %direction for %value %mode at %speed\\% %waitMode"
    //% speed.min=1 speed.max=100 speed.defl=50 value.min=0
    //% inlineInputMode=inline
    //% weight=76
    /** 按时间、毫米或厘米发送一条固定双电机直行命令。 */
    export function robotDriveStraight(direction: DriveDirection, value: number, mode: DriveMode, speed: number, waitMode: WaitMode = 0): void {
        if (value <= 0 || speed <= 0 || robotWheelDiameterMm <= 0) {
            return
        }
        cancelRobotMotion()
        let turnMode = TurnMode.Second
        let movementX10 = Math.round(value * 10)
        if (mode != DriveMode.Second) {
            let distanceMm = mode == DriveMode.Centimeter ? value * 10 : value
            movementX10 = Math.round(distanceMm * 3600
                / (robotWheelDiameterMm * Math.PI))
            turnMode = TurnMode.Degree
        }
        if (direction == DriveDirection.Backward) {
            movementX10 = -movementX10
        }
        let leftValue = -movementX10
        let rightValue = movementX10
        let speedPercent = Math.round(clamp(speed, 1, 100))
        let leftStart = pins.createBuffer(0)
        let rightStart = pins.createBuffer(0)
        if (waitMode == WaitMode.Wait) {
            leftStart = refreshMotorData(robotLeftMotor,
                MOTOR_DATA_REFRESH_ANGLE | MOTOR_DATA_REFRESH_SPEED)
            rightStart = refreshMotorData(robotRightMotor,
                MOTOR_DATA_REFRESH_ANGLE | MOTOR_DATA_REFRESH_SPEED)
        }
        let directionMask = 0
        if (leftValue < 0) {
            directionMask |= 0x01
        }
        if (rightValue < 0) {
            directionMask |= 0x02
        }
        let leftValueX10 = Math.abs(leftValue)
        let rightValueX10 = Math.abs(rightValue)
        i2cCommandSend(COMMAND_ROBOT_MOVE, [
            robotLeftMotor,
            robotRightMotor,
            turnMode,
            (leftValueX10 >> 24) & 0xFF,
            (leftValueX10 >> 16) & 0xFF,
            (leftValueX10 >> 8) & 0xFF,
            leftValueX10 & 0xFF,
            (rightValueX10 >> 24) & 0xFF,
            (rightValueX10 >> 16) & 0xFF,
            (rightValueX10 >> 8) & 0xFF,
            rightValueX10 & 0xFF,
            speedPercent,
            directionMask
        ])
        if (waitMode == WaitMode.Wait) {
            waitForMotorFeedback(robotLeftMotor, leftStart, turnMode, leftValue,
                speedPercent * 9, -1, TurnDirectionEx.ShortestPath)
            waitForMotorFeedback(robotRightMotor, rightStart, turnMode, rightValue,
                speedPercent * 9, -1, TurnDirectionEx.ShortestPath)
        }
    }

    //% group="Robot"
    //% block="stop robot"
    //% weight=75
    /** 取消机器人转向并最高优先级停止左右轮。 */
    export function robotStop(): void {
        cancelRobotMotion(false)
        i2cCommandSend(COMMAND_STOP, [robotMotorMask() & 0x0F])
    }

    //% group="Robot"
    //% block="robot yaw angle (degrees)"
    //% weight=74
    /** 返回机器人使用的板载Z轴累计偏航角。 */
    export function robotGetYaw(): number {
        return readGyroAngle(SensorAxis.Z)
    }

    //% group="Sensor"
    //% block="gyroscope %axis accumulated angle (degrees)"
    //% weight=70
    /** 读取板载陀螺仪指定轴的累计角度。 */
    export function readGyroAngle(axis: SensorAxis): number {
        let data = readRegisters(REGISTER_GYRO_ANGLE_START + axis * 4, 4)
        return data.length == 4 ? readI32Le(data, 0) / 10 : 0
    }

    //% group="Sensor"
    //% block="acceleration %axis (mg)"
    //% weight=69
    /** 读取板载加速度计指定轴的加速度，单位为mg。 */
    export function readAcceleration(axis: SensorAxis): number {
        let data = readRegisters(REGISTER_ACCELERATION_START + axis * 2, 2)
        return data.length == 2 ? readI16Le(data, 0) : 0
    }

    //% group="Information"
    //% block="firmware version"
    //% weight=60
    /** 读取下位机固件版本。 */
    export function readVersion(): string {
        i2cCommandSend(COMMAND_VERSION, [])
        let reply = pins.i2cReadBuffer(I2C_ADDRESS, 3)
        return reply.length == 3
            ? "V " + reply[0] + "." + reply[1] + "." + reply[2]
            : "V 0.0.0"
    }

    /** 读取指定电机当前锁存通信错误，不显示为积木。 */
    export function readMotorError(motor: MotorPort): MotorErrorCode {
        let data = readRegisters(REGISTER_MOTOR_ERROR_START + motor - 1, 1)
        return data.length == 1 && data[0] <= MotorErrorCode.TransmitTimeout
            ? data[0]
            : MotorErrorCode.Unknown
    }

    /** 将当前累计角度保存为下位机相对零点，不显示为积木。 */
    export function resetRelativeAngle(motor: MotorPort): void {
        cancelRobotMotion()
        i2cCommandSend(COMMAND_RESET_RELATIVE, [(1 << (motor - 1)) & 0x0F])
    }
}
