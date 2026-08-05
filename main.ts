//% color=#ff0011 icon="\uf1b9" block="Smart Motor"
namespace smartMotor {
    const I2C_ADDRESS = 0x10
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
    const REGISTER_MOTOR_ONLINE_MASK = 0x15
    const REGISTER_MOTOR_ERROR_START = 0x16
    const REPLY_STATUS_ACCEPTED = 0

    /** 电机接口位置。 */
    export enum MotorPosition {
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
    export enum MovementDirection {
        //% block="clockwise"
        CW = 1,
        //% block="counterclockwise"
        CCW = 2
    }

    /** 绝对角度运动方向。 */
    export enum AbsoluteTurnMode {
        //% block="shortest path"
        ShortestPath = 1,
        //% block="clockwise"
        CW = 2,
        //% block="counterclockwise"
        CCW = 3
    }

    /** 定量运动单位。 */
    export enum MoveMode {
        //% block="turns"
        Circle = 1,
        //% block="degrees"
        Degree = 2,
        //% block="seconds"
        Second = 3
    }

    /** 组合电机前进或后退方向。 */
    export enum TravelDirection {
        //% block="forward"
        Forward = 1,
        //% block="backward"
        Backward = 2
    }

    /** 组合运动距离和角度单位。 */
    export enum ComboMoveUnit {
        //% block="degrees"
        Degree = 2,
        //% block="turns"
        Circle = 1,
        //% block="seconds"
        Second = 3,
        //% block="centimeters"
        Centimeter = 4,
        //% block="inches"
        Inch = 5
    }

    /** 长度单位。 */
    export enum LengthUnit {
        //% block="centimeters"
        Centimeter = 1,
        //% block="inches"
        Inch = 2
    }

    /** 是否在积木返回前按运动参数进行估算等待。 */
    export enum DelayMode {
        //% block="wait"
        Wait = 1,
        //% block="do not wait"
        NoWait = 0
    }

    /** 单路电机当前通信错误状态。 */
    export enum MotorErrorCode {
        //% block="no error"
        None = 0,
        //% block="heartbeat timeout"
        HeartbeatTimeout = 1,
        //% block="command response timeout"
        CommandResponseTimeout = 2,
        //% block="UART transmit timeout"
        TransmitTimeout = 3,
        //% block="unknown"
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
    let comboLeftMotor = MotorPosition.M1
    let comboRightMotor = MotorPosition.M2
    let wheelPerimeterCm = 0
    let wheelBaseCm = 0
    let comboRotateCalibration = 1
    let servoSpeedValue = 900

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

    /** 将一个字节写入缓冲区。 */
    function writeU8(buffer: Buffer, offset: number, value: number): void {
        buffer[offset] = value & 0xFF
    }

    /** 将16位值按小端写入缓冲区。 */
    function writeU16Le(buffer: Buffer, offset: number, value: number): void {
        buffer[offset] = value & 0xFF
        buffer[offset + 1] = (value >> 8) & 0xFF
    }

    /** 将32位值按小端写入缓冲区。 */
    function writeI32Le(buffer: Buffer, offset: number, value: number): void {
        let integerValue = Math.round(value)
        buffer[offset] = integerValue & 0xFF
        buffer[offset + 1] = (integerValue >> 8) & 0xFF
        buffer[offset + 2] = (integerValue >> 16) & 0xFF
        buffer[offset + 3] = (integerValue >> 24) & 0xFF
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
        return motor >= MotorPosition.M1 && motor <= MotorPosition.M4
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

    /** 发送统一I2C帧并读取、校验统一回复帧。 */
    function transact(command: number, payload: Buffer, maxReplyPayload: number): Buffer {
        let frame = pins.createBuffer(payload.length + 4)
        frame[0] = 0xFF
        frame[1] = 0xF9
        frame[2] = command
        frame[3] = payload.length
        for (let index = 0; index < payload.length; index++) {
            frame[index + 4] = payload[index]
        }
        acquireI2c()
        pins.i2cWriteBuffer(I2C_ADDRESS, frame)
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
    function sendControl(command: number, payload: Buffer, transactionId: number): boolean {
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
        let payload = pins.createBuffer(2 + motors.length * 3)
        payload[0] = transactionId
        payload[1] = motors.length
        for (let index = 0; index < motors.length; index++) {
            if (!validMotor(motors[index])) {
                return false
            }
            let offset = 2 + index * 3
            payload[offset] = motors[index]
            writeU16Le(payload, offset + 1, clamp(Math.round(pwmValues[index]), -1000, 1000))
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
        let payload = pins.createBuffer(2 + motors.length * 8)
        payload[0] = transactionId
        payload[1] = motors.length
        for (let index = 0; index < motors.length; index++) {
            if (!validMotor(motors[index])) {
                return false
            }
            let offset = 2 + index * 8
            payload[offset] = motors[index]
            payload[offset + 1] = modes[index]
            writeI32Le(payload, offset + 2, valuesX10[index])
            writeU16Le(payload, offset + 6, clamp(Math.round(speeds[index]), 1, 900))
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
        let payload = pins.createBuffer(2 + motors.length * 8)
        payload[0] = transactionId
        payload[1] = motors.length
        for (let index = 0; index < motors.length; index++) {
            if (!validMotor(motors[index])) {
                return false
            }
            let offset = 2 + index * 8
            payload[offset] = motors[index]
            payload[offset + 1] = turnModes[index]
            writeI32Le(payload, offset + 2, targetsX10[index])
            writeU16Le(payload, offset + 6, clamp(Math.round(speeds[index]), 1, 900))
        }
        return sendControl(COMMAND_MOVE_ABSOLUTE, payload, transactionId)
    }

    /** 发送带事务ID和电机掩码的停止或归零命令。 */
    function sendMaskCommand(command: number, motorMask: number): boolean {
        let transactionId = nextTransactionId()
        let payload = pins.createBuffer(2)
        payload[0] = transactionId
        payload[1] = motorMask & 0x0F
        return sendControl(command, payload, transactionId)
    }

    /** 按地址读取最多17字节下位机寄存器。 */
    function readRegisters(startAddress: number, length: number): Buffer {
        let requestLength = clamp(Math.round(length), 1, 17)
        let payload = pins.createBuffer(2)
        payload[0] = startAddress
        payload[1] = requestLength
        let reply = transact(COMMAND_REGISTER_READ, payload, requestLength + 3)
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
    function movementValueX10(value: number, mode: MoveMode): number {
        if (mode == MoveMode.Circle) {
            return Math.round(value * 3600)
        }
        return Math.round(value * 10)
    }

    /** 按参考工程的速度和运动量估算积木等待时间。 */
    function waitForEstimatedMotion(value: number, mode: MoveMode, speedPercent: number): void {
        let speed = clamp(speedPercent, 1, 100) * 9
        let delayTime = 0
        if (mode == MoveMode.Circle) {
            delayTime = value * 360000 / speed + 500
        } else if (mode == MoveMode.Degree) {
            delayTime = value * 1000 / speed + 500
        } else {
            delayTime = value * 1000
        }
        basic.pause(delayTime)
    }

    //% group="Single motor"
    //% block="set %motor speed to %speed\%"
    //% speed.min=-100 speed.max=100
    //% weight=100
    /** 设置单路电机有符号速度并立即启动。 */
    export function start(motor: MotorPosition, speed: number): void {
        let limitedSpeed = clamp(speed, -100, 100)
        sendSpeedBatch([motor], [Math.round(limitedSpeed * 10)])
    }

    //% group="Single motor"
    //% block="stop %motor"
    //% weight=99
    /** 最高优先级停止指定电机并取消其待执行运动。 */
    export function stop(motor: MotorPosition): void {
        if (validMotor(motor)) {
            sendMaskCommand(COMMAND_STOP, 1 << (motor - 1))
        }
    }

    //% group="Single motor"
    //% block="move %motor at %speed\% %direction for %value %mode || %delayMode"
    //% speed.min=1 speed.max=100 value.min=0
    //% inlineInputMode=inline
    //% weight=98
    /** 按圈、角度或时间控制单路电机运动。 */
    export function move(motor: MotorPosition, speed: number, direction: MovementDirection, value: number, mode: MoveMode, delayMode: DelayMode = DelayMode.Wait): void {
        if (!validMotor(motor) || speed <= 0 || value <= 0) {
            return
        }
        let limitedSpeed = clamp(speed, 1, 100)
        servoSpeedValue = Math.round(limitedSpeed * 9)
        let signedValue = movementValueX10(value, mode)
        if (direction == MovementDirection.CCW) {
            signedValue = -signedValue
        }
        if (sendMoveBatch([motor], [mode], [signedValue], [servoSpeedValue])
            && delayMode == DelayMode.Wait) {
            waitForEstimatedMotion(value, mode, limitedSpeed)
        }
    }

    //% group="Single motor"
    //% block="move %motor to absolute angle %angle degrees via %turnMode || %delayMode"
    //% angle.min=0 angle.max=359
    //% inlineInputMode=inline
    //% weight=97
    /** 查询当前位置后按指定方向转到单圈绝对角度。 */
    export function moveToAbsoluteAngle(motor: MotorPosition, turnMode: AbsoluteTurnMode, angle: number, delayMode: DelayMode = DelayMode.Wait): void {
        if (!validMotor(motor)) {
            return
        }
        let normalized = angle
        while (normalized < 0) {
            normalized += 360
        }
        normalized = normalized % 360
        if (sendAbsoluteBatch([motor], [turnMode], [Math.round(normalized * 10)], [servoSpeedValue])
            && delayMode == DelayMode.Wait) {
            basic.pause(500)
        }
    }

    //% group="Single motor"
    //% block="reset physical position of %motor"
    //% weight=96
    /** 向电机发送物理位置归零命令。 */
    export function resetPhysicalZero(motor: MotorPosition): void {
        if (validMotor(motor) && sendMaskCommand(COMMAND_RESET_PHYSICAL, 1 << (motor - 1))) {
            basic.pause(1000)
        }
    }

    //% group="Single motor"
    //% block="set current angle of %motor as relative zero"
    //% weight=95
    /** 在下位机保存当前累计角度作为相对零点。 */
    export function resetRelativeAngle(motor: MotorPosition): void {
        if (validMotor(motor)) {
            sendMaskCommand(COMMAND_RESET_RELATIVE, 1 << (motor - 1))
        }
    }

    //% group="Data"
    //% block="gyroscope %axis accumulated angle (degrees)"
    //% weight=90
    /** 读取板载陀螺仪指定轴的累计角度。 */
    export function readGyroAngle(axis: SensorAxis): number {
        let data = readRegisters(REGISTER_GYRO_ANGLE_START + axis * 4, 4)
        return data.length == 4 ? readI32Le(data, 0) / 10 : 0
    }

    //% group="Data"
    //% block="acceleration %axis (mg)"
    //% weight=89
    /** 读取板载加速度计指定轴的加速度，单位为mg。 */
    export function readAcceleration(axis: SensorAxis): number {
        let data = readRegisters(REGISTER_ACCELERATION_START + axis * 2, 2)
        return data.length == 2 ? readI16Le(data, 0) : 0
    }

    //% group="Data"
    //% block="is %motor online"
    //% weight=88
    /** 查询下位机首次心跳和在线租期结果。 */
    export function isMotorOnline(motor: MotorPosition): boolean {
        let data = readRegisters(REGISTER_MOTOR_ONLINE_MASK, 1)
        return data.length == 1 && (data[0] & (1 << (motor - 1))) != 0
    }

    //% group="Combined motors"
    //% block="set left motor %leftMotor and right motor %rightMotor"
    //% weight=80
    /** 设置组合运动使用的左右电机，默认分别为M1和M2。 */
    export function setComboMotors(leftMotor: MotorPosition, rightMotor: MotorPosition): void {
        if (leftMotor != rightMotor) {
            comboLeftMotor = leftMotor
            comboRightMotor = rightMotor
        }
    }

    //% group="Combined motors"
    //% block="move %direction at %speed\% speed"
    //% speed.min=0 speed.max=100
    //% weight=79
    /** 一条I2C命令同时启动左右电机前进或后退。 */
    export function comboRun(speed: number, direction: TravelDirection): void {
        let pwm = Math.round(clamp(speed, 0, 100) * 10)
        let leftPwm = direction == TravelDirection.Forward ? -pwm : pwm
        let rightPwm = -leftPwm
        sendSpeedBatch([comboLeftMotor, comboRightMotor], [leftPwm, rightPwm])
    }

    //% group="Combined motors"
    //% block="set left speed %leftSpeed\% and right speed %rightSpeed\%"
    //% leftSpeed.min=-100 leftSpeed.max=100 rightSpeed.min=-100 rightSpeed.max=100
    //% weight=78
    /** 一条I2C命令设置左右轮独立有符号速度。 */
    export function comboStart(leftSpeed: number, rightSpeed: number): void {
        let leftPwm = -Math.round(clamp(leftSpeed, -100, 100) * 10)
        let rightPwm = Math.round(clamp(rightSpeed, -100, 100) * 10)
        sendSpeedBatch([comboLeftMotor, comboRightMotor], [leftPwm, rightPwm])
    }

    //% group="Combined motors"
    //% block="stop combined motors"
    //% weight=77
    /** 一条最高优先级I2C命令同时停止左右电机。 */
    export function comboStop(): void {
        let mask = (1 << (comboLeftMotor - 1)) | (1 << (comboRightMotor - 1))
        sendMaskCommand(COMMAND_STOP, mask)
    }

    //% group="Combined motors"
    //% block="set wheel circumference to %value %unit"
    //% value.min=0
    //% weight=76
    /** 保存组合距离换算使用的车轮周长。 */
    export function setWheelPerimeter(value: number, unit: LengthUnit): void {
        wheelPerimeterCm = unit == LengthUnit.Inch ? value * 2.54 : value
    }

    //% group="Combined motors"
    //% block="set wheelbase to %value %unit"
    //% value.min=0
    //% weight=75
    /** 保存原地旋转换算使用的左右轮中心距离。 */
    export function setWheelBase(value: number, unit: LengthUnit): void {
        wheelBaseCm = unit == LengthUnit.Inch ? value * 2.54 : value
    }

    //% group="Combined motors"
    //% block="set combined rotation calibration to %factor"
    //% factor.min=0.01
    //% weight=74
    /** 设置机械安装误差使用的组合旋转校准系数。 */
    export function setComboRotateCalibration(factor: number): void {
        comboRotateCalibration = factor > 0 ? factor : 1
    }

    //% group="Combined motors"
    //% block="move %direction at %speed\% for %value %unit"
    //% speed.min=1 speed.max=100 value.min=0
    //% inlineInputMode=inline
    //% weight=73
    /** 一条I2C命令同时控制左右轮完成相同运动量。 */
    export function comboMove(speed: number, direction: TravelDirection, value: number, unit: ComboMoveUnit): void {
        if (speed <= 0 || value <= 0) {
            return
        }
        let moveMode = MoveMode.Degree
        let convertedValue = value
        if (unit == ComboMoveUnit.Circle) {
            moveMode = MoveMode.Circle
        } else if (unit == ComboMoveUnit.Second) {
            moveMode = MoveMode.Second
        } else if (unit == ComboMoveUnit.Centimeter) {
            if (wheelPerimeterCm <= 0) {
                return
            }
            convertedValue = 360 * value / wheelPerimeterCm
        } else if (unit == ComboMoveUnit.Inch) {
            if (wheelPerimeterCm <= 0) {
                return
            }
            convertedValue = 360 * value * 2.54 / wheelPerimeterCm
        }
        let speedValue = Math.round(clamp(speed, 1, 100) * 9)
        servoSpeedValue = speedValue
        let movement = movementValueX10(convertedValue, moveMode)
        let leftValue = direction == TravelDirection.Forward ? -movement : movement
        let rightValue = -leftValue
        if (sendMoveBatch([comboLeftMotor, comboRightMotor], [moveMode, moveMode], [leftValue, rightValue], [speedValue, speedValue])) {
            waitForEstimatedMotion(convertedValue, moveMode, speed)
        }
    }

    //% group="Combined motors"
    //% block="rotate combined motors %angle degrees at %speed\% speed"
    //% speed.min=1 speed.max=100
    //% weight=72
    /** 一条I2C命令同时控制左右轮完成原地旋转。 */
    export function comboRotate(angle: number, speed: number): void {
        if (angle == 0 || speed <= 0 || wheelBaseCm <= 0 || wheelPerimeterCm <= 0) {
            return
        }
        let arcDistance = Math.abs(angle) * Math.PI / 180 * (wheelBaseCm / 2)
        let motorDegree = arcDistance * 360 * comboRotateCalibration / wheelPerimeterCm
        let movement = Math.round(motorDegree * 10)
        if (angle > 0) {
            movement = -movement
        }
        let speedValue = Math.round(clamp(speed, 1, 100) * 9)
        servoSpeedValue = speedValue
        if (sendMoveBatch([comboLeftMotor, comboRightMotor], [MoveMode.Degree, MoveMode.Degree], [movement, movement], [speedValue, speedValue])) {
            waitForEstimatedMotion(motorDegree, MoveMode.Degree, speed)
        }
    }

    //% group="Status"
    //% block="was the last control accepted"
    //% weight=60
    /** 返回最近一次控制命令的即时I2C接收结果。 */
    export function lastCommandAccepted(): boolean {
        return lastAcceptStatusValue == REPLY_STATUS_ACCEPTED
    }

    //% group="Status"
    //% block="%motor error status"
    //% weight=59
    /** 读取指定电机当前锁存的通信错误，合法回复恢复后自动清除。 */
    export function readMotorError(motor: MotorPosition): MotorErrorCode {
        let data = readRegisters(REGISTER_MOTOR_ERROR_START + motor - 1, 1)
        if (data.length != 1 || data[0] > MotorErrorCode.TransmitTimeout) {
            return MotorErrorCode.Unknown
        }
        return data[0]
    }

    //% group="Status"
    //% block="firmware version"
    //% weight=58
    /** 读取下位机固件版本。 */
    export function readVersion(): string {
        let payload = pins.createBuffer(0)
        let reply = transact(COMMAND_VERSION, payload, 3)
        if (reply.length != 3) {
            return "V ?.?.?"
        }
        return "V " + reply[0] + "." + reply[1] + "." + reply[2]
    }
}
