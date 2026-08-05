//% color=#ff0011 icon="\uf1b9" block="Smart Motor"
namespace smartMotor {
    const I2C_ADDRESS = 0x66
    const I2C_COMMAND_DELAY_MS = 1
    const I2C_REGISTER_PREPARE_DELAY_MS = 2
    const COMMAND_REGISTER_READ = 0x01
    const COMMAND_VERSION = 0x10
    const COMMAND_SET_SPEED = 0x20
    const COMMAND_STOP = 0x21
    const COMMAND_MOVE = 0x22
    const COMMAND_MOVE_ABSOLUTE = 0x23
    const COMMAND_RESET_PHYSICAL = 0x24
    const COMMAND_RESET_RELATIVE = 0x25
    const REGISTER_GYRO_ANGLE_START = 0x03
    const REGISTER_ACCELERATION_START = 0x0F
    const REGISTER_MOTOR_ERROR_START = 0x15
    const REGISTER_MOTOR_TELEMETRY_START = 0x19
    const MOTOR_TELEMETRY_RECORD_LENGTH = 13
    const MOTOR_TELEMETRY_ANGLE_VALID = 0x01
    const MOTOR_TELEMETRY_SPEED_VALID = 0x02
    const MOTOR_TELEMETRY_RELATIVE_ANGLE_OFFSET = 1
    const MOTOR_TELEMETRY_ABSOLUTE_ANGLE_OFFSET = 5
    const MOTOR_TELEMETRY_SPEED_OFFSET = 9
    const MOTOR_TELEMETRY_ANGLE_SEQUENCE_OFFSET = 11
    const MOTOR_TELEMETRY_SPEED_SEQUENCE_OFFSET = 12
    const MOTION_START_DELAY_MS = 100
    const MOTION_POLL_INTERVAL_MS = 20
    const MOTION_TARGET_TOLERANCE_X10 = 10
    const MOTION_START_ANGLE_DELTA_X10 = 5
    const MOTION_STABLE_ANGLE_DELTA_X10 = 1
    const MOTION_STABLE_ANGLE_SAMPLE_COUNT = 10
    const MOTION_STOP_SAMPLE_COUNT = 2
    const MOTION_MIN_TIMEOUT_MS = 2000
    const MOTION_MAX_TIMEOUT_MS = 60000
    const RESET_WAIT_TIMEOUT_MS = 5000
    const ROBOT_TURN_TOLERANCE_DEGREES = 1
    const ROBOT_DEFAULT_WHEEL_DIAMETER_MM = 62
    const REPLY_STATUS_ACCEPTED = 0

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

    let i2cLocked = false
    let nextTransactionIdValue = 1
    let lastAcceptStatusValue = 0
    let robotLeftMotor = MotorPort.M1
    let robotRightMotor = MotorPort.M2
    let robotWheelDiameterMm = ROBOT_DEFAULT_WHEEL_DIAMETER_MM
    let robotMotionGeneration = 0
    let robotTurnActive = false

    /** 等待并独占本扩展的I2C写读组合，避免MakeCode多个fiber相互插入。 */
    function acquireI2c(): void {
        while (i2cLocked) {
            basic.pause(1)
        }
        i2cLocked = true
    }

    /** 释放本扩展的I2C访问权。 */
    function releaseI2c(): void {
        i2cLocked = false
    }

    /** 返回一个自然回绕且不使用0的8位事务ID。 */
    function nextTransactionId(): number {
        let result = nextTransactionIdValue
        nextTransactionIdValue = (nextTransactionIdValue + 1) & 0xFF
        if (nextTransactionIdValue == 0) {
            nextTransactionIdValue = 1
        }
        return result
    }

    /** 从小端字节流读取无符号16位值。 */
    function readU16Le(buffer: Buffer, offset: number): number {
        return buffer[offset] | (buffer[offset + 1] << 8)
    }

    /** 从小端字节流读取有符号16位值。 */
    function readI16Le(buffer: Buffer, offset: number): number {
        let value = readU16Le(buffer, offset)
        return value >= 0x8000 ? value - 0x10000 : value
    }

    /** 从小端字节流读取有符号32位值。 */
    function readI32Le(buffer: Buffer, offset: number): number {
        return (buffer[offset])
            | (buffer[offset + 1] << 8)
            | (buffer[offset + 2] << 16)
            | (buffer[offset + 3] << 24)
    }

    /** 判断电机号是否处于M1～M4范围。 */
    function validMotor(motor: number): boolean {
        return motor >= MotorPort.M1 && motor <= MotorPort.M4
    }

    /** 将速度百分比限制到指定范围并换算为整数。 */
    function clamp(value: number, minimum: number, maximum: number): number {
        if (value < minimum) {
            return minimum
        }
        if (value > maximum) {
            return maximum
        }
        return value
    }

    /** 按参考工程方式忙等待短暂的I2C从机响应准备时间。 */
    function delayMs(ms: number): void {
        let endTime = input.runningTime() + ms
        while (endTime > input.runningTime()) {
        }
    }

    /** 构造并发送统一I2C命令帧，按当前业务要求等待下位机准备回复。 */
    function i2cCommandSend(command: number, params: number[], prepareDelayMs: number): void {
        let frame = pins.createBuffer(params.length + 4)
        frame[0] = 0xFF
        frame[1] = 0xF9
        frame[2] = command
        frame[3] = params.length
        for (let index = 0; index < params.length; index++) {
            frame[index + 4] = params[index]
        }
        pins.i2cWriteBuffer(I2C_ADDRESS, frame)
        delayMs(prepareDelayMs)
    }

    /** 发送统一I2C帧并读取、校验统一回复帧。 */
    function transact(command: number, payload: number[], maxReplyPayload: number, prepareDelayMs: number = I2C_COMMAND_DELAY_MS): Buffer {
        acquireI2c()
        i2cCommandSend(command, payload, prepareDelayMs)
        let rawReply = pins.i2cReadBuffer(I2C_ADDRESS, maxReplyPayload + 4)
        releaseI2c()
        if (rawReply.length < 4
            || rawReply[0] != 0xFF
            || rawReply[1] != 0xF9
            || rawReply[2] != command
            || rawReply[3] > maxReplyPayload) {
            return pins.createBuffer(0)
        }
        let replyLength = rawReply[3]
        let reply = pins.createBuffer(replyLength)
        for (let index = 0; index < replyLength; index++) {
            reply[index] = rawReply[index + 4]
        }
        return reply
    }

    /** 发送电机控制帧并保存即时接收结果。 */
    function sendControl(command: number, payload: number[], transactionId: number): boolean {
        let reply = transact(command, payload, 2)
        if (reply.length != 2) {
            lastAcceptStatusValue = 0xFF
            return false
        }
        lastAcceptStatusValue = reply[0]
        return reply[0] == REPLY_STATUS_ACCEPTED && reply[1] == transactionId
    }

    /** 一条I2C事务批量设置最多四路PWM速度。 */
    function sendSpeedBatch(motors: number[], pwmValues: number[]): boolean {
        if (motors.length == 0 || motors.length != pwmValues.length || motors.length > 4) {
            return false
        }
        let transactionId = nextTransactionId()
        let payload: number[] = []
        payload[0] = transactionId
        payload[1] = motors.length
        for (let index = 0; index < motors.length; index++) {
            if (!validMotor(motors[index])) {
                return false
            }
            let offset = 2 + index * 3
            let pwmValue = clamp(Math.round(pwmValues[index]), -1000, 1000)
            payload[offset] = motors[index]
            payload[offset + 1] = pwmValue & 0xFF
            payload[offset + 2] = (pwmValue >> 8) & 0xFF
        }
        return sendControl(COMMAND_SET_SPEED, payload, transactionId)
    }

    /** 一条I2C事务批量发送最多两路定量运动。 */
    function sendMoveBatch(motors: number[], modes: number[], valuesX10: number[], speeds: number[]): boolean {
        if (motors.length == 0
            || motors.length > 2
            || motors.length != modes.length
            || motors.length != valuesX10.length
            || motors.length != speeds.length) {
            return false
        }
        let transactionId = nextTransactionId()
        let payload: number[] = []
        payload[0] = transactionId
        payload[1] = motors.length
        for (let index = 0; index < motors.length; index++) {
            if (!validMotor(motors[index])) {
                return false
            }
            let offset = 2 + index * 8
            let movementValue = Math.round(valuesX10[index])
            let speedValue = clamp(Math.round(speeds[index]), 1, 900)
            payload[offset] = motors[index]
            payload[offset + 1] = modes[index]
            payload[offset + 2] = movementValue & 0xFF
            payload[offset + 3] = (movementValue >> 8) & 0xFF
            payload[offset + 4] = (movementValue >> 16) & 0xFF
            payload[offset + 5] = (movementValue >> 24) & 0xFF
            payload[offset + 6] = speedValue & 0xFF
            payload[offset + 7] = (speedValue >> 8) & 0xFF
        }
        return sendControl(COMMAND_MOVE, payload, transactionId)
    }

    /** 一条I2C事务批量发送最多两路绝对角度运动。 */
    function sendAbsoluteBatch(motors: number[], turnModes: number[], targetsX10: number[], speeds: number[]): boolean {
        if (motors.length == 0
            || motors.length > 2
            || motors.length != turnModes.length
            || motors.length != targetsX10.length
            || motors.length != speeds.length) {
            return false
        }
        let transactionId = nextTransactionId()
        let payload: number[] = []
        payload[0] = transactionId
        payload[1] = motors.length
        for (let index = 0; index < motors.length; index++) {
            if (!validMotor(motors[index])) {
                return false
            }
            let offset = 2 + index * 8
            let targetValue = Math.round(targetsX10[index])
            let speedValue = clamp(Math.round(speeds[index]), 1, 900)
            payload[offset] = motors[index]
            payload[offset + 1] = turnModes[index]
            payload[offset + 2] = targetValue & 0xFF
            payload[offset + 3] = (targetValue >> 8) & 0xFF
            payload[offset + 4] = (targetValue >> 16) & 0xFF
            payload[offset + 5] = (targetValue >> 24) & 0xFF
            payload[offset + 6] = speedValue & 0xFF
            payload[offset + 7] = (speedValue >> 8) & 0xFF
        }
        return sendControl(COMMAND_MOVE_ABSOLUTE, payload, transactionId)
    }

    /** 发送带事务ID和电机掩码的停止或归零命令。 */
    function sendMaskCommand(command: number, motorMask: number): boolean {
        let transactionId = nextTransactionId()
        let payload = [transactionId, motorMask & 0x0F]
        return sendControl(command, payload, transactionId)
    }

    /** 通知下位机按需刷新请求范围，等待2ms后读取最多17字节寄存器。 */
    function readRegisters(startAddress: number, length: number): Buffer {
        let requestLength = clamp(Math.round(length), 1, 17)
        let payload = [startAddress, requestLength]
        let reply = transact(COMMAND_REGISTER_READ, payload, requestLength + 3, I2C_REGISTER_PREPARE_DELAY_MS)
        if (reply.length < 3 || reply[1] != startAddress || reply[2] > requestLength) {
            return pins.createBuffer(0)
        }
        let result = pins.createBuffer(reply[2])
        for (let index = 0; index < result.length; index++) {
            result[index] = reply[index + 3]
        }
        return result
    }

    /** 将运动量换算为下位机统一使用的0.1度或0.1秒值。 */
    function movementValueX10(value: number, mode: TurnMode): number {
        if (mode == TurnMode.Circle) {
            return Math.round(value * 3600)
        }
        return Math.round(value * 10)
    }

    /** 读取单路电机的有效标志、相对/绝对角度、速度和更新序列。 */
    function readMotorTelemetry(motor: MotorPort): Buffer {
        if (!validMotor(motor)) {
            return pins.createBuffer(0)
        }
        let startAddress = REGISTER_MOTOR_TELEMETRY_START
            + (motor - MotorPort.M1) * MOTOR_TELEMETRY_RECORD_LENGTH
        return readRegisters(startAddress, MOTOR_TELEMETRY_RECORD_LENGTH)
    }

    /** 将任意0.1度角度归一化到0～3599。 */
    function normalizeAngleX10(angleX10: number): number {
        let normalized = angleX10 % 3600
        if (normalized < 0) {
            normalized += 3600
        }
        return normalized
    }

    /** 返回两个单圈角度之间的最小绝对差，单位为0.1度。 */
    function absoluteAngleErrorX10(currentX10: number, targetX10: number): number {
        let difference = Math.abs(normalizeAngleX10(currentX10) - normalizeAngleX10(targetX10))
        return Math.min(difference, 3600 - difference)
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

    /**
     * 启动后等待100ms并确认运动发生；所有模式都以连续新速度样本归零作为
     * 完成条件，角度模式还要求到达目标或运动后角度稳定，由动态超时兜底。
     */
    function waitForMotorFeedback(motor: MotorPort, startTelemetry: Buffer, mode: TurnMode, commandValueX10: number, speedValue: number, absoluteTargetX10: number, turnMode: TurnDirectionEx): void {
        let startAngleValid = false
        let startAngleX10 = 0
        let lastAngleSequence = -1
        let lastSpeedSequence = -1
        if (startTelemetry.length == MOTOR_TELEMETRY_RECORD_LENGTH) {
            startAngleValid = (startTelemetry[0] & MOTOR_TELEMETRY_ANGLE_VALID) != 0
            if (startAngleValid) {
                startAngleX10 = readI32Le(startTelemetry,
                    absoluteTargetX10 >= 0
                        ? MOTOR_TELEMETRY_ABSOLUTE_ANGLE_OFFSET
                        : MOTOR_TELEMETRY_RELATIVE_ANGLE_OFFSET)
            }
            lastAngleSequence = startTelemetry[MOTOR_TELEMETRY_ANGLE_SEQUENCE_OFFSET]
            lastSpeedSequence = startTelemetry[MOTOR_TELEMETRY_SPEED_SEQUENCE_OFFSET]
        }
        let absoluteTarget = absoluteTargetX10 >= 0
        let targetAngleValid = absoluteTarget || (startAngleValid && mode != TurnMode.Second)
        let targetAngleX10 = absoluteTarget
            ? normalizeAngleX10(absoluteTargetX10)
            : startAngleX10 + commandValueX10
        let expectedValueX10 = Math.abs(commandValueX10)
        let timeoutMode = mode
        if (absoluteTarget) {
            timeoutMode = TurnMode.Degree
            expectedValueX10 = startAngleValid
                ? absoluteTravelX10(startAngleX10, targetAngleX10, turnMode)
                : 3600
        }
        let timeoutMs = motionTimeoutMs(expectedValueX10, timeoutMode, speedValue)
        let waitStartMs = input.runningTime()
        let referenceAngleValid = startAngleValid
        let referenceAngleX10 = startAngleX10
        let previousAngleValid = false
        let previousAngleX10 = 0
        let motionObserved = false
        let targetReached = false
        let stableAngleSamples = 0
        let stoppedSamples = 0
        basic.pause(MOTION_START_DELAY_MS)
        while (input.runningTime() - waitStartMs < timeoutMs) {
            let telemetry = readMotorTelemetry(motor)
            if (telemetry.length == MOTOR_TELEMETRY_RECORD_LENGTH) {
                let flags = telemetry[0]
                let angleSequence = telemetry[MOTOR_TELEMETRY_ANGLE_SEQUENCE_OFFSET]
                if ((flags & MOTOR_TELEMETRY_ANGLE_VALID) != 0
                    && angleSequence != lastAngleSequence) {
                    let currentAngleX10 = readI32Le(telemetry,
                        absoluteTarget
                            ? MOTOR_TELEMETRY_ABSOLUTE_ANGLE_OFFSET
                            : MOTOR_TELEMETRY_RELATIVE_ANGLE_OFFSET)
                    lastAngleSequence = angleSequence
                    if (!referenceAngleValid) {
                        referenceAngleValid = true
                        referenceAngleX10 = currentAngleX10
                    } else if (Math.abs(currentAngleX10 - referenceAngleX10)
                        >= MOTION_START_ANGLE_DELTA_X10) {
                        motionObserved = true
                    }
                    if (previousAngleValid
                        && Math.abs(currentAngleX10 - previousAngleX10)
                            <= MOTION_STABLE_ANGLE_DELTA_X10) {
                        stableAngleSamples++
                    } else {
                        stableAngleSamples = 0
                    }
                    previousAngleValid = true
                    previousAngleX10 = currentAngleX10
                    if (targetAngleValid) {
                        let targetErrorX10 = absoluteTarget
                            ? absoluteAngleErrorX10(currentAngleX10, targetAngleX10)
                            : Math.abs(currentAngleX10 - targetAngleX10)
                        targetReached = targetErrorX10 <= MOTION_TARGET_TOLERANCE_X10
                    }
                }
                let speedSequence = telemetry[MOTOR_TELEMETRY_SPEED_SEQUENCE_OFFSET]
                if ((flags & MOTOR_TELEMETRY_SPEED_VALID) != 0
                    && speedSequence != lastSpeedSequence) {
                    let currentSpeed = readI16Le(telemetry, MOTOR_TELEMETRY_SPEED_OFFSET)
                    lastSpeedSequence = speedSequence
                    if (currentSpeed == 0) {
                        if (motionObserved) {
                            stoppedSamples++
                        } else {
                            stoppedSamples = 0
                        }
                    } else {
                        stoppedSamples = 0
                        motionObserved = true
                    }
                }
                if (mode == TurnMode.Second) {
                    if (stoppedSamples >= MOTION_STOP_SAMPLE_COUNT
                        && (motionObserved
                            || input.runningTime() - waitStartMs
                                >= Math.abs(commandValueX10) * 100)) {
                        return
                    }
                } else if (stoppedSamples >= MOTION_STOP_SAMPLE_COUNT
                    && (targetReached
                        || stableAngleSamples
                            >= MOTION_STABLE_ANGLE_SAMPLE_COUNT)) {
                    return
                }
            }
            basic.pause(MOTION_POLL_INTERVAL_MS)
        }
    }

    /** 使机器人转向任务失效，并按需停止其正在驱动的左右轮。 */
    function cancelRobotMotion(stopActiveMotors: boolean = true): void {
        robotMotionGeneration++
        let shouldStop = robotTurnActive && stopActiveMotors
        robotTurnActive = false
        if (shouldStop) {
            sendMaskCommand(COMMAND_STOP, robotMotorMask())
        }
    }

    /** 返回当前左右轮对应的电机通道掩码。 */
    function robotMotorMask(): number {
        return (1 << (robotLeftMotor - 1)) | (1 << (robotRightMotor - 1))
    }

    /** 按机器人逻辑方向一次性设置左右轮速度。 */
    function sendRobotSpeed(leftSpeed: number, rightSpeed: number): boolean {
        let leftPwm = -Math.round(clamp(leftSpeed, -100, 100) * 10)
        let rightPwm = Math.round(clamp(rightSpeed, -100, 100) * 10)
        return sendSpeedBatch([robotLeftMotor, robotRightMotor], [leftPwm, rightPwm])
    }

    /** 在物理归零后等待一份新的接近零度的相对角度样本。 */
    function waitForMotorReset(motor: MotorPort, previousSequence: number): void {
        let startMs = input.runningTime()
        basic.pause(MOTION_START_DELAY_MS)
        while (input.runningTime() - startMs < RESET_WAIT_TIMEOUT_MS) {
            let telemetry = readMotorTelemetry(motor)
            if (telemetry.length == MOTOR_TELEMETRY_RECORD_LENGTH
                && (telemetry[0] & MOTOR_TELEMETRY_ANGLE_VALID) != 0
                && telemetry[MOTOR_TELEMETRY_ANGLE_SEQUENCE_OFFSET] != previousSequence
                && Math.abs(readI32Le(telemetry, MOTOR_TELEMETRY_RELATIVE_ANGLE_OFFSET))
                    <= MOTION_TARGET_TOLERANCE_X10) {
                return
            }
            basic.pause(MOTION_POLL_INTERVAL_MS)
        }
    }

    /** 使用板载Z轴累计角度执行无PID的机器人相对转向。 */
    function runRobotTurn(angle: number, speed: number, generation: number): void {
        if (generation != robotMotionGeneration) {
            return
        }
        let startYaw = readGyroAngle(SensorAxis.Z)
        let targetYaw = startYaw + angle
        let turnSpeed = clamp(Math.abs(speed), 1, 100)
        let positiveDirection = angle > 0
        robotTurnActive = true
        if (!sendRobotSpeed(positiveDirection ? turnSpeed : -turnSpeed,
            positiveDirection ? -turnSpeed : turnSpeed)) {
            if (generation == robotMotionGeneration) {
                robotTurnActive = false
            }
            return
        }
        if (generation != robotMotionGeneration) {
            return
        }
        let startMs = input.runningTime()
        let timeoutMs = clamp(Math.round(Math.abs(angle) * 1000 / turnSpeed + 2000),
            MOTION_MIN_TIMEOUT_MS, MOTION_MAX_TIMEOUT_MS)
        while (input.runningTime() - startMs < timeoutMs) {
            if (generation != robotMotionGeneration) {
                return
            }
            let error = targetYaw - readGyroAngle(SensorAxis.Z)
            if (Math.abs(error) <= ROBOT_TURN_TOLERANCE_DEGREES
                || (positiveDirection && error < 0)
                || (!positiveDirection && error > 0)) {
                sendMaskCommand(COMMAND_STOP, robotMotorMask())
                if (generation == robotMotionGeneration) {
                    robotTurnActive = false
                }
                return
            }
            basic.pause(MOTION_POLL_INTERVAL_MS)
        }
        if (generation == robotMotionGeneration) {
            sendMaskCommand(COMMAND_STOP, robotMotorMask())
            if (generation == robotMotionGeneration) {
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
        if (!validMotor(motor)) {
            return
        }
        cancelRobotMotion()
        let pwm = Math.round(clamp(speed, 0, 100) * 10)
        sendSpeedBatch([motor], [direction == TurnDirection.CCW ? -pwm : pwm])
    }

    //% group="Motor"
    //% block="stop %motor"
    //% weight=99
    /** 最高优先级停止指定电机并取消其待执行运动。 */
    export function motorStop(motor: MotorPort): void {
        if (validMotor(motor)) {
            cancelRobotMotion()
            sendMaskCommand(COMMAND_STOP, 1 << (motor - 1))
        }
    }

    //% group="Motor"
    //% block="reset position of %motor %waitMode"
    //% weight=98
    /** 将电机内部物理位置归零，并可等待新角度样本确认完成。 */
    export function motorReset(motor: MotorPort, waitMode: WaitMode = WaitMode.NoWait): void {
        if (!validMotor(motor)) {
            return
        }
        cancelRobotMotion()
        let previousSequence = 0
        if (waitMode == WaitMode.Wait) {
            let telemetry = readMotorTelemetry(motor)
            if (telemetry.length == MOTOR_TELEMETRY_RECORD_LENGTH) {
                previousSequence = telemetry[MOTOR_TELEMETRY_ANGLE_SEQUENCE_OFFSET]
            }
        }
        if (sendMaskCommand(COMMAND_RESET_PHYSICAL, 1 << (motor - 1))
            && waitMode == WaitMode.Wait) {
            waitForMotorReset(motor, previousSequence)
        }
    }

    //% group="Motor"
    //% block="move %motor %value %mode at %speed\\% %direction %waitMode"
    //% speed.min=1 speed.max=100 speed.defl=50 value.min=0
    //% inlineInputMode=inline
    //% weight=97
    /** 按秒、角度或圈数控制单路电机相对运动。 */
    export function motorMoveRelative(motor: MotorPort, value: number, mode: TurnMode, speed: number, direction: TurnDirection, waitMode: WaitMode = WaitMode.NoWait): void {
        if (!validMotor(motor) || speed <= 0 || value <= 0) {
            return
        }
        cancelRobotMotion()
        let speedValue = Math.round(clamp(speed, 1, 100) * 9)
        let signedValue = movementValueX10(value, mode)
        if (direction == TurnDirection.CCW) {
            signedValue = -signedValue
        }
        let startTelemetry = pins.createBuffer(0)
        if (waitMode == WaitMode.Wait) {
            startTelemetry = readMotorTelemetry(motor)
        }
        if (sendMoveBatch([motor], [mode], [signedValue], [speedValue])
            && waitMode == WaitMode.Wait) {
            waitForMotorFeedback(motor, startTelemetry, mode, signedValue,
                speedValue, -1, TurnDirectionEx.ShortestPath)
        }
    }

    //% group="Motor"
    //% block="move %motor to absolute angle %angle degrees at %speed\\% via %direction %waitMode"
    //% angle.min=0 angle.max=359 speed.min=1 speed.max=100 speed.defl=50
    //% inlineInputMode=inline
    //% weight=96
    /** 按指定路径和速度转到单圈绝对角度。 */
    export function motorMoveAbsolute(motor: MotorPort, angle: number, speed: number, direction: TurnDirectionEx, waitMode: WaitMode = WaitMode.NoWait): void {
        if (!validMotor(motor) || speed <= 0) {
            return
        }
        cancelRobotMotion()
        let normalized = normalizeAngleX10(Math.round(angle * 10))
        let speedValue = Math.round(clamp(speed, 1, 100) * 9)
        let startTelemetry = pins.createBuffer(0)
        if (waitMode == WaitMode.Wait) {
            startTelemetry = readMotorTelemetry(motor)
        }
        if (sendAbsoluteBatch([motor], [direction], [normalized], [speedValue])
            && waitMode == WaitMode.Wait) {
            waitForMotorFeedback(motor, startTelemetry, TurnMode.Degree, 0,
                speedValue, normalized, direction)
        }
    }

    //% group="Motor"
    //% block="%motor speed (degrees/s)"
    //% weight=95
    /** 读取单路电机最近一次有效速度。 */
    export function motorGetSpeed(motor: MotorPort): number {
        let telemetry = readMotorTelemetry(motor)
        if (telemetry.length != MOTOR_TELEMETRY_RECORD_LENGTH
            || (telemetry[0] & MOTOR_TELEMETRY_SPEED_VALID) == 0) {
            return 0
        }
        return readI16Le(telemetry, MOTOR_TELEMETRY_SPEED_OFFSET)
    }

    //% group="Motor"
    //% block="%motor relative angle (degrees)"
    //% weight=94
    /** 读取相对物理归零点或下位机相对零点的累计角度。 */
    export function motorGetRelativeAngle(motor: MotorPort): number {
        let telemetry = readMotorTelemetry(motor)
        if (telemetry.length != MOTOR_TELEMETRY_RECORD_LENGTH
            || (telemetry[0] & MOTOR_TELEMETRY_ANGLE_VALID) == 0) {
            return 0
        }
        return readI32Le(telemetry, MOTOR_TELEMETRY_RELATIVE_ANGLE_OFFSET) / 10
    }

    //% group="Motor"
    //% block="%motor absolute angle (degrees)"
    //% weight=93
    /** 读取归一化到0～359.9度的单圈绝对角度。 */
    export function motorGetAbsoluteAngle(motor: MotorPort): number {
        let telemetry = readMotorTelemetry(motor)
        if (telemetry.length != MOTOR_TELEMETRY_RECORD_LENGTH
            || (telemetry[0] & MOTOR_TELEMETRY_ANGLE_VALID) == 0) {
            return 0
        }
        return normalizeAngleX10(
            readI32Le(telemetry, MOTOR_TELEMETRY_ABSOLUTE_ANGLE_OFFSET)) / 10
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
    /** 一条I2C批量命令设置机器人左右轮独立速度。 */
    export function robotMove(leftSpeed: number, rightSpeed: number): void {
        cancelRobotMotion()
        sendRobotSpeed(leftSpeed, rightSpeed)
    }

    //% group="Robot"
    //% block="turn robot %angle degrees at %speed\\% %waitMode"
    //% speed.min=1 speed.max=100 speed.defl=50
    //% weight=77
    /** 使用板载Z轴角度控制机器人相对转向，不引入PID调节。 */
    export function robotTurnTo(angle: number, speed: number, waitMode: WaitMode = WaitMode.NoWait): void {
        if (angle == 0 || speed <= 0) {
            return
        }
        cancelRobotMotion()
        let generation = robotMotionGeneration
        if (waitMode == WaitMode.Wait) {
            runRobotTurn(angle, speed, generation)
        } else {
            control.inBackground(function () {
                runRobotTurn(angle, speed, generation)
            })
        }
    }

    //% group="Robot"
    //% block="drive %direction for %value %mode at %speed\\% %waitMode"
    //% speed.min=1 speed.max=100 speed.defl=50 value.min=0
    //% inlineInputMode=inline
    //% weight=76
    /** 按时间、毫米或厘米发送一条左右轮批量直行命令。 */
    export function robotDriveStraight(direction: DriveDirection, value: number, mode: DriveMode, speed: number, waitMode: WaitMode = WaitMode.NoWait): void {
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
        let speedValue = Math.round(clamp(speed, 1, 100) * 9)
        let leftStart = pins.createBuffer(0)
        let rightStart = pins.createBuffer(0)
        if (waitMode == WaitMode.Wait) {
            leftStart = readMotorTelemetry(robotLeftMotor)
            rightStart = readMotorTelemetry(robotRightMotor)
        }
        if (sendMoveBatch([robotLeftMotor, robotRightMotor], [turnMode, turnMode],
            [leftValue, rightValue], [speedValue, speedValue])
            && waitMode == WaitMode.Wait) {
            waitForMotorFeedback(robotLeftMotor, leftStart, turnMode, leftValue,
                speedValue, -1, TurnDirectionEx.ShortestPath)
            waitForMotorFeedback(robotRightMotor, rightStart, turnMode, rightValue,
                speedValue, -1, TurnDirectionEx.ShortestPath)
        }
    }

    //% group="Robot"
    //% block="stop robot"
    //% weight=75
    /** 取消机器人后台转向并最高优先级停止左右轮。 */
    export function robotStop(): void {
        cancelRobotMotion(false)
        sendMaskCommand(COMMAND_STOP, robotMotorMask())
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
        let payload: number[] = []
        let reply = transact(COMMAND_VERSION, payload, 3)
        if (reply.length != 3) {
            return "V 0.0.0"
        }
        return "V " + reply[0] + "." + reply[1] + "." + reply[2]
    }

    /** 返回最近一次控制命令是否被下位机即时接收，不显示为积木。 */
    export function lastCommandAccepted(): boolean {
        return lastAcceptStatusValue == REPLY_STATUS_ACCEPTED
    }

    /** 读取指定电机当前锁存通信错误，不显示为积木。 */
    export function readMotorError(motor: MotorPort): MotorErrorCode {
        let data = readRegisters(REGISTER_MOTOR_ERROR_START + motor - 1, 1)
        if (data.length != 1 || data[0] > MotorErrorCode.TransmitTimeout) {
            return MotorErrorCode.Unknown
        }
        return data[0]
    }

    /** 将当前累计角度保存为下位机相对零点，不显示为积木。 */
    export function resetRelativeAngle(motor: MotorPort): void {
        if (validMotor(motor)) {
            cancelRobotMotion()
            sendMaskCommand(COMMAND_RESET_RELATIVE, 1 << (motor - 1))
        }
    }
}
