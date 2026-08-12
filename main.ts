//% color=#ff0011 icon="\uf1b9" block="Smart Motor" weight=100
//% groups='["Motor","Position","Robot","Readings","Gyroscope","Information"]'
namespace smartMotor {
    const I2C_ADDRESS = 0x66
    const I2C_QUERY_POLL_INTERVAL_MS = 2
    const I2C_QUERY_TIMEOUT_MS = 100
    const GYRO_RESET_CONFIRM_TIMEOUT_MS = 350
    const GYRO_RESET_CONFIRM_POLL_INTERVAL_MS = 10
    const GYRO_RESET_CONFIRM_TOLERANCE_X10 = 1
    const COMMAND_REGISTER_READ = 0x01
    const COMMAND_MOTOR_DATA_REFRESH = 0x02
    const COMMAND_VERSION = 0x10
    const COMMAND_GYRO_RESET = 0x11
    const COMMAND_SET_SPEED = 0x20
    const COMMAND_STOP = 0x21
    const COMMAND_MOVE = 0x22
    const COMMAND_MOVE_ABSOLUTE = 0x23
    const COMMAND_RESET_PHYSICAL = 0x24
    const COMMAND_ROBOT_SET_SPEED = 0x26
    const REGISTER_GYRO_ANGLE_START = 0x03
    const MOTOR_DATA_RECORD_LENGTH = 13
    const MOTOR_DATA_ANGLE_VALID = 0x01
    const MOTOR_DATA_SPEED_VALID = 0x02
    const MOTOR_DATA_RELATIVE_ANGLE_OFFSET = 1
    const MOTOR_DATA_ABSOLUTE_ANGLE_OFFSET = 5
    const MOTOR_DATA_SPEED_OFFSET = 9
    const MOTOR_DATA_REFRESH_ANGLE = 0x01
    const MOTOR_DATA_REFRESH_SPEED = 0x02
    const ROBOT_DRIVE_GYRO_KP = 2
    const ROBOT_DRIVE_GYRO_MAX_CORRECTION = 35
    const ROBOT_INVALID_GYRO_ANGLE = 1000000000
    const ROBOT_DEFAULT_WHEEL_DIAMETER_MM = 62

    /** Motor connector shown on the Power Smart Motor Hub. */
    export enum MotorPort {
        //% block="M5"
        M5 = 1,
        //% block="M6"
        M6 = 2,
        //% block="M7"
        M7 = 3,
        //% block="M8"
        M8 = 4
    }

    /** Motor rotation direction. */
    export enum MotorDirection {
        //% block="clockwise"
        Clockwise = 0,
        //% block="counterclockwise"
        Counterclockwise = 1
    }

    /** Robot straight-drive direction. */
    export enum DriveDirection {
        //% block="forward"
        Forward = 0,
        //% block="backward"
        Backward = 1
    }

    /** Unit or mode used by the robot straight-drive block. */
    export enum DriveMode {
        //% block="millimeters"
        Millimeters = 0,
        //% block="seconds"
        Seconds = 1,
        //% block="degrees"
        Degrees = 2
    }

    /** Robot speed level. */
    export enum AccelLevel {
        //% block="slow"
        Slow = 0,
        //% block="medium"
        Medium = 1,
        //% block="fast"
        Fast = 2
    }

    /** Whether a robot command waits until the motion is complete. */
    export enum WaitMode {
        //% block="do not wait"
        NoWait = 0,
        //% block="wait until done"
        Wait = 1
    }

    /** Gyroscope attitude axis. */
    export enum GyroAxis {
        //% block="pitch"
        Pitch = 0,
        //% block="yaw"
        Yaw = 2,
        //% block="roll"
        Roll = 1
    }

    let robotLeftMotor = MotorPort.M5
    let robotRightMotor = MotorPort.M6
    let robotWheelDiameterMm = ROBOT_DEFAULT_WHEEL_DIAMETER_MM
    let robotMotionId = 0
    let robotTurnActive = false
    let robotDriveActive = false
    let robotWorkerStarted = false
    let robotTurnMotionId = 0
    let robotTurnTargetYaw = 0
    let robotTurnCurrentSpeed = 0
    let robotTurnMaxSpeed = 0
    let robotTurnLastError = 0
    let robotTurnLastTime = 0
    let robotTurnAccel = AccelLevel.Medium
    let lastQueryWasSuccessful = false
    let queryCacheKeys: string[] = []
    let queryCacheData: Buffer[] = []
    let gyroSpeedLastAngle: number[] = [0, 0, 0]
    let gyroSpeedLastTime: number[] = [0, 0, 0]
    let gyroSpeedHasSample: boolean[] = [false, false, false]

    function readI16Le(buffer: Buffer, offset: number): number {
        let value = buffer[offset] | (buffer[offset + 1] << 8)
        return value >= 0x8000 ? value - 0x10000 : value
    }

    function readI32Le(buffer: Buffer, offset: number): number {
        return (buffer[offset])
            | (buffer[offset + 1] << 8)
            | (buffer[offset + 2] << 16)
            | (buffer[offset + 3] << 24)
    }

    function clamp(value: number, minimum: number, maximum: number): number {
        return value < minimum ? minimum : value > maximum ? maximum : value
    }

    function copyBuffer(source: Buffer): Buffer {
        let result = pins.createBuffer(source.length)
        for (let index = 0; index < source.length; index++) {
            result[index] = source[index]
        }
        return result
    }

    function delayMs(ms: number): void {
        let endTime = input.runningTime() + ms
        while (endTime > input.runningTime()) {
        }
    }

    function queryCacheKey(command: number, commandData: number[], dataLength: number): string {
        let key = "" + command
        for (let index = 0; index < commandData.length; index++) {
            key += ":" + commandData[index]
        }
        return key + ":" + dataLength
    }

    function findQueryCache(key: string): number {
        for (let index = 0; index < queryCacheKeys.length; index++) {
            if (queryCacheKeys[index] == key) {
                return index
            }
        }
        return -1
    }

    function readQueryCache(key: string, dataLength: number): Buffer {
        let index = findQueryCache(key)
        return index >= 0 ? copyBuffer(queryCacheData[index]) : pins.createBuffer(dataLength)
    }

    function writeQueryCache(key: string, data: Buffer): void {
        let index = findQueryCache(key)
        if (index < 0) {
            queryCacheKeys.push(key)
            queryCacheData.push(copyBuffer(data))
        } else {
            queryCacheData[index] = copyBuffer(data)
        }
    }

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

    function i2cQueryRead(command: number, commandData: number[], dataLength: number,
        timeoutMs: number = 100): Buffer {
        let key = queryCacheKey(command, commandData, dataLength)
        let cachedData = readQueryCache(key, dataLength)
        lastQueryWasSuccessful = false
        let deadline = input.runningTime() + Math.max(0, timeoutMs)
        i2cCommandSend(command, commandData, 0)
        while (input.runningTime() < deadline) {
            let remainingMs = deadline - input.runningTime()
            delayMs(Math.min(I2C_QUERY_POLL_INTERVAL_MS, remainingMs))
            if (input.runningTime() >= deadline) {
                break
            }
            let reply = pins.i2cReadBuffer(I2C_ADDRESS, dataLength + 1)
            if (reply.length >= dataLength + 1 && reply[0] == 1) {
                let data = pins.createBuffer(dataLength)
                for (let index = 0; index < dataLength; index++) {
                    data[index] = reply[index + 1]
                }
                writeQueryCache(key, data)
                lastQueryWasSuccessful = true
                return data
            }
        }
        return cachedData
    }

    function readRegisters(startAddress: number, length: number,
        timeoutMs: number = 100): Buffer {
        let requestLength = clamp(Math.round(length), 1, 24)
        return i2cQueryRead(COMMAND_REGISTER_READ, [startAddress, requestLength], requestLength,
            timeoutMs)
    }

    function refreshMotorData(motor: MotorPort, dataMask: number): Buffer {
        return i2cQueryRead(COMMAND_MOTOR_DATA_REFRESH, [motor, dataMask], MOTOR_DATA_RECORD_LENGTH)
    }

    function readFreshGyroAngle(axis: GyroAxis): number {
        let data = readRegisters(REGISTER_GYRO_ANGLE_START + axis * 4, 4)
        return lastQueryWasSuccessful && data.length == 4
            ? readI32Le(data, 0) / 10
            : ROBOT_INVALID_GYRO_ANGLE
    }

    function gyroAngleIsValid(angle: number): boolean {
        return angle != ROBOT_INVALID_GYRO_ANGLE
    }

    function refreshFreshMotorData(motor: MotorPort, dataMask: number): Buffer {
        let data = refreshMotorData(motor, dataMask)
        return lastQueryWasSuccessful ? data : pins.createBuffer(0)
    }

    function normalizeAngleX10(angleX10: number): number {
        let normalized = angleX10 % 3600
        return normalized < 0 ? normalized + 3600 : normalized
    }

    function motorMask(motor: MotorPort): number {
        return (1 << (motor - 1)) & 0x0F
    }

    function signedSpeed(speed: number): number {
        return Math.round(clamp(speed, -100, 100))
    }

    function motorDirectionBit(direction: MotorDirection, speed: number): number {
        let reverse = signedSpeed(speed) < 0
        let counterclockwise = direction == MotorDirection.Counterclockwise
        return reverse != counterclockwise ? 1 : 0
    }

    function robotMotorMask(): number {
        return motorMask(robotLeftMotor) | motorMask(robotRightMotor)
    }

    function cancelRobotMotion(stopActiveMotors: boolean = true): void {
        robotMotionId++
        let shouldStop = (robotTurnActive || robotDriveActive) && stopActiveMotors
        robotTurnActive = false
        robotDriveActive = false
        if (shouldStop) {
            i2cCommandSend(COMMAND_STOP, [robotMotorMask()])
        }
    }

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

    function sendMotorRelativeStep(motor: MotorPort, angle: number, speed: number): void {
        if (angle == 0 || speed == 0) {
            return
        }
        let valueX10 = Math.abs(Math.round(clamp(angle, 0, 360) * 10))
        let speedPercent = Math.abs(signedSpeed(speed))
        let reverse = signedSpeed(speed) < 0
        let counterclockwise = angle < 0
        i2cCommandSend(COMMAND_MOVE, [
            motor,
            2,
            (valueX10 >> 24) & 0xFF,
            (valueX10 >> 16) & 0xFF,
            (valueX10 >> 8) & 0xFF,
            valueX10 & 0xFF,
            speedPercent,
            reverse != counterclockwise ? 1 : 0
        ])
    }

    function speedForLevel(accel: AccelLevel): number {
        if (accel == AccelLevel.Slow) {
            return 35
        }
        if (accel == AccelLevel.Fast) {
            return 85
        }
        return 60
    }

    function turnAccelerationForLevel(accel: AccelLevel): number {
        if (accel == AccelLevel.Slow) {
            return 25
        }
        if (accel == AccelLevel.Fast) {
            return 300
        }
        return 60
    }

    function robotStopIfCurrentMotion(motionId: number): void {
        if (motionId == robotMotionId) {
            i2cCommandSend(COMMAND_STOP, [robotMotorMask()])
            robotTurnActive = false
            robotDriveActive = false
        }
    }

    function updateRobotTurn(): void {
        if (!robotTurnActive || robotTurnMotionId != robotMotionId) {
            return
        }

        let now = input.runningTime()
        let elapsed = now - robotTurnLastTime
        if (elapsed <= 0) {
            return
        }
        robotTurnLastTime = now

        let error = robotTurnTargetYaw - readFreshGyroAngle(GyroAxis.Yaw)
        let crossedTarget = (robotTurnLastError > 0 && error <= 0)
            || (robotTurnLastError < 0 && error >= 0)
        if (Math.abs(error) <= 1 || crossedTarget) {
            robotStopIfCurrentMotion(robotTurnMotionId)
            return
        }

        let dt = elapsed / 1000
        let derivative = dt <= 0.2 ? (error - robotTurnLastError) / dt : 0
        robotTurnLastError = error

        let minimumSpeed = Math.min(8, robotTurnMaxSpeed)
        let allowedSpeed = robotTurnMaxSpeed
        if (Math.abs(error) < 30) {
            allowedSpeed = Math.max(minimumSpeed,
                robotTurnMaxSpeed * Math.abs(error) / 30)
        }

        let acceleration = turnAccelerationForLevel(robotTurnAccel)
        if (robotTurnCurrentSpeed < allowedSpeed) {
            robotTurnCurrentSpeed = Math.min(allowedSpeed,
                robotTurnCurrentSpeed + acceleration * dt)
        } else if (robotTurnCurrentSpeed > allowedSpeed) {
            robotTurnCurrentSpeed = Math.max(allowedSpeed,
                robotTurnCurrentSpeed - acceleration * 2 * dt)
        }

        let output = clamp(0.35 * error + 0.015 * derivative,
            -robotTurnCurrentSpeed, robotTurnCurrentSpeed)
        if (output * error <= 0 || Math.abs(output) < minimumSpeed) {
            output = error > 0 ? minimumSpeed : -minimumSpeed
        }
        sendRobotSpeed(output, -output)
    }

    function startRobotWorker(): void {
        if (robotWorkerStarted) {
            return
        }
        robotWorkerStarted = true
        control.inBackground(function () {
            while (true) {
                updateRobotTurn()
                basic.pause(9)
            }
        })
    }

    function runRobotDriveStraight(direction: DriveDirection, movementX10: number, timedDrive: boolean,
        speed: number, motionId: number): void {
        if (motionId != robotMotionId) {
            return
        }
        let speedPercent = Math.round(clamp(speed, 1, 100))
        let baseSpeed = direction == DriveDirection.Backward ? -speedPercent : speedPercent
        let leftStart = pins.createBuffer(0)
        let rightStart = pins.createBuffer(0)
        let leftStartAngle = 0
        let rightStartAngle = 0
        let startMs = input.runningTime()
        if (!timedDrive) {
            leftStart = refreshFreshMotorData(robotLeftMotor, MOTOR_DATA_REFRESH_ANGLE)
            rightStart = refreshFreshMotorData(robotRightMotor, MOTOR_DATA_REFRESH_ANGLE)
            leftStartAngle = readI32Le(leftStart, MOTOR_DATA_RELATIVE_ANGLE_OFFSET)
            rightStartAngle = readI32Le(rightStart, MOTOR_DATA_RELATIVE_ANGLE_OFFSET)
        }
        let targetYaw = readFreshGyroAngle(GyroAxis.Yaw)
        let maxCorrection = clamp(speedPercent - 1, 0, ROBOT_DRIVE_GYRO_MAX_CORRECTION)
        robotDriveActive = true
        while (true) {
            if (motionId != robotMotionId) {
                return
            }
            let currentYaw = readFreshGyroAngle(GyroAxis.Yaw)
            let yawError = targetYaw - currentYaw
            let correction = clamp(Math.round(yawError * ROBOT_DRIVE_GYRO_KP),
                -maxCorrection, maxCorrection)
            sendRobotSpeed(baseSpeed - correction, baseSpeed + correction)
            if (timedDrive) {
                if (input.runningTime() - startMs >= Math.abs(movementX10) * 100) {
                    break
                }
            } else {
                let leftData = refreshFreshMotorData(robotLeftMotor, MOTOR_DATA_REFRESH_ANGLE)
                let rightData = refreshFreshMotorData(robotRightMotor, MOTOR_DATA_REFRESH_ANGLE)
                let leftAngle = readI32Le(leftData, MOTOR_DATA_RELATIVE_ANGLE_OFFSET)
                let rightAngle = readI32Le(rightData, MOTOR_DATA_RELATIVE_ANGLE_OFFSET)
                let leftTravel = Math.abs(leftAngle - leftStartAngle)
                let rightTravel = Math.abs(rightAngle - rightStartAngle)
                if (leftTravel >= Math.abs(movementX10) && rightTravel >= Math.abs(movementX10)) {
                    break
                }
            }
            basic.pause(10)
        }
        robotStopIfCurrentMotion(motionId)
    }

    //% group="Motor"
    //% blockId=smartmotor_motor_start block="motor $motor direction $direction speed $speed start"
    //% motor.defl=smartMotor.MotorPort.M5
    //% speed.min=-100 speed.max=100 speed.defl=50
    //% direction.defl=smartMotor.MotorDirection.Clockwise
    //% weight=100
    /**
     * Start a motor with the selected direction and signed speed.
     * @param motor motor port M5-M8
     * @param direction clockwise or counterclockwise direction
     * @param speed speed from -100 to 100
     */
    export function motorStart(motor: MotorPort, direction: MotorDirection, speed: number): void {
        cancelRobotMotion()
        let speedPercent = Math.abs(signedSpeed(speed))
        i2cCommandSend(COMMAND_SET_SPEED, [motor, speedPercent, motorDirectionBit(direction, speed)])
    }

    //% group="Motor"
    //% blockId=smartmotor_motor_stop block="motor $motor stop"
    //% motor.defl=smartMotor.MotorPort.M5
    //% weight=99
    /**
     * Stop one motor.
     * @param motor motor port M5-M8
     */
    export function motorStop(motor: MotorPort): void {
        cancelRobotMotion()
        i2cCommandSend(COMMAND_STOP, [motorMask(motor)])
    }

    //% group="Position"
    //% blockId=smartmotor_motor_reset block="motor $motor reset position"
    //% motor.defl=smartMotor.MotorPort.M5
    //% weight=90
    /**
     * Reset the current motor position to zero.
     * @param motor motor port M5-M8
     */
    export function motorReset(motor: MotorPort): void {
        cancelRobotMotion()
        i2cCommandSend(COMMAND_RESET_PHYSICAL, [motorMask(motor)])
    }

    //% group="Position"
    //% blockId=smartmotor_motor_move_absolute block="motor $motor rotate to absolute angle $angle speed $speed"
    //% motor.defl=smartMotor.MotorPort.M5
    //% angle.min=0 angle.max=360 angle.defl=90
    //% speed.min=-100 speed.max=100 speed.defl=50
    //% inlineInputMode=inline
    //% weight=89
    /**
     * Rotate a motor to an absolute angle.
     * @param motor motor port M5-M8
     * @param angle target angle in degrees, 0 to 360
     * @param speed speed from -100 to 100
     */
    export function motorMoveAbsolute(motor: MotorPort, angle: number, speed: number): void {
        if (speed == 0) {
            return
        }
        cancelRobotMotion()
        let normalized = normalizeAngleX10(Math.round(angle * 10))
        let speedPercent = Math.abs(signedSpeed(speed))
        i2cCommandSend(COMMAND_MOVE_ABSOLUTE, [
            motor,
            (normalized >> 8) & 0xFF,
            normalized & 0xFF,
            speedPercent,
            2
        ])
    }

    //% group="Position"
    //% blockId=smartmotor_motor_move_relative block="motor $motor rotate angle $angle speed $speed"
    //% motor.defl=smartMotor.MotorPort.M5
    //% angle.min=0 angle.max=360 angle.defl=90
    //% speed.min=-100 speed.max=100 speed.defl=50
    //% inlineInputMode=inline
    //% weight=88
    /**
     * Rotate a motor by a relative angle.
     * @param motor motor port M5-M8
     * @param angle relative angle in degrees, 0 to 360
     * @param speed speed from -100 to 100
     */
    export function motorMoveRelative(motor: MotorPort, angle: number, speed: number): void {
        if (angle == 0 || speed == 0) {
            return
        }
        cancelRobotMotion()
        sendMotorRelativeStep(motor, angle, speed)
    }

    //% group="Robot"
    //% blockId=smartmotor_robot_set_wheel_diameter block="robot wheel diameter $diameter mm"
    //% diameter.min=0 diameter.max=10000 diameter.defl=62
    //% weight=80
    /**
     * Set the robot wheel diameter.
     * @param diameter wheel diameter in millimeters, 0 to 10000
     */
    export function robotSetWheelDiameter(diameter: number): void {
        robotWheelDiameterMm = Math.round(clamp(diameter, 0, 10000))
    }

    //% group="Robot"
    //% blockId=smartmotor_robot_set_motors block="robot left wheel $leftMotor and right wheel $rightMotor"
    //% leftMotor.defl=smartMotor.MotorPort.M5
    //% rightMotor.defl=smartMotor.MotorPort.M6
    //% weight=79
    /**
     * Select the motors used as the robot left and right wheels.
     * @param leftMotor left wheel motor port M5-M8
     * @param rightMotor right wheel motor port M5-M8
     */
    export function robotSetMotors(leftMotor: MotorPort, rightMotor: MotorPort): void {
        if (leftMotor != rightMotor) {
            cancelRobotMotion()
            robotLeftMotor = leftMotor
            robotRightMotor = rightMotor
        }
    }

    //% group="Robot"
    //% blockId=smartmotor_robot_turn block="robot turn $angle degrees speed $speed acceleration $accel $waitMode"
    //% angle.min=-360 angle.max=360 angle.defl=90
    //% speed.min=0 speed.max=100 speed.defl=50
    //% accel.defl=smartMotor.AccelLevel.Medium
    //% waitMode.defl=smartMotor.WaitMode.Wait
    //% inlineInputMode=inline
    //% weight=78
    /**
     * Turn the robot in place using gyroscope feedback.
     * @param angle turn angle in degrees, -360 to 360
     * @param speed speed from 0 to 100
     * @param accel acceleration level used by the turn speed ramp
     * @param waitMode wait for completion or return after starting the background motion
     */
    export function robotTurn(angle: number, speed: number, accel: AccelLevel,
        waitMode: WaitMode = WaitMode.Wait): void {
        cancelRobotMotion()
        let turnAngle = clamp(angle, -360, 360)
        let turnSpeed = clamp(speed, 0, 100)
        if (turnAngle == 0 || turnSpeed <= 0) {
            return
        }

        robotTurnMotionId = robotMotionId
        robotTurnTargetYaw = readFreshGyroAngle(GyroAxis.Yaw) + turnAngle
        robotTurnCurrentSpeed = 8
        robotTurnMaxSpeed = Math.abs(turnSpeed)
        robotTurnLastError = turnAngle
        robotTurnLastTime = input.runningTime()
        robotTurnAccel = accel
        robotTurnActive = true
        startRobotWorker()

        if (waitMode == WaitMode.Wait) {
            while (robotTurnActive && robotTurnMotionId == robotMotionId) {
                basic.pause(10)
            }
        }
    }

    //% group="Robot"
    //% blockId=smartmotor_robot_drive_straight block="robot drive $direction $value $mode speed level $accel"
    //% direction.defl=smartMotor.DriveDirection.Forward
    //% value.min=0 value.max=10000 value.defl=100
    //% mode.defl=smartMotor.DriveMode.Millimeters
    //% accel.defl=smartMotor.AccelLevel.Medium
    //% inlineInputMode=inline
    //% weight=77
    /**
     * Drive the robot straight using a distance, time, or wheel-angle value.
     * @param direction forward or backward
     * @param value distance in millimeters, time in seconds, or wheel-angle value in degrees
     * @param mode millimeters, seconds, or wheel degrees
     * @param accel speed level mapped to 35, 60, or 85 percent
     */
    export function robotDriveStraight(direction: DriveDirection, value: number, mode: DriveMode, accel: AccelLevel): void {
        cancelRobotMotion()
        if (robotWheelDiameterMm <= 0) {
            return
        }
        let driveValue = Math.round(clamp(value, 0, 10000))
        if (driveValue <= 0) {
            return
        }
        let movementX10 = driveValue * 10
        let timedDrive = mode == DriveMode.Seconds
        if (mode == DriveMode.Millimeters) {
            movementX10 = Math.round(driveValue * 3600 / (robotWheelDiameterMm * Math.PI))
        }
        runRobotDriveStraight(direction, movementX10, timedDrive, speedForLevel(accel), robotMotionId)
    }

    //% group="Robot"
    //% blockId=smartmotor_robot_stop block="robot stop"
    //% weight=76
    /**
     * Stop robot motion.
     */
    export function robotStop(): void {
        cancelRobotMotion(false)
        i2cCommandSend(COMMAND_STOP, [robotMotorMask()])
    }

    //% group="Robot"
    //% blockId=smartmotor_robot_is_idle block="robot is idle"
    //% weight=75
    /**
     * Check whether the robot has no active turn or straight-drive motion.
     */
    export function robotIsIdle(): boolean {
        return !robotTurnActive && !robotDriveActive
    }

    //% group="Readings"
    //% blockId=smartmotor_motor_speed block="motor $motor current speed (degrees/s)"
    //% motor.defl=smartMotor.MotorPort.M5
    //% weight=70
    /**
     * Read the current motor speed.
     * @param motor motor port M5-M8
     */
    export function motorGetSpeed(motor: MotorPort): number {
        let motorData = refreshMotorData(motor, MOTOR_DATA_REFRESH_SPEED)
        if (motorData.length != MOTOR_DATA_RECORD_LENGTH
            || (motorData[0] & MOTOR_DATA_SPEED_VALID) == 0) {
            return 0
        }
        return readI16Le(motorData, MOTOR_DATA_SPEED_OFFSET)
    }

    //% group="Readings"
    //% blockId=smartmotor_motor_absolute_angle block="motor $motor absolute angle"
    //% motor.defl=smartMotor.MotorPort.M5
    //% weight=69
    /**
     * Read the current motor absolute angle in degrees.
     * @param motor motor port M5-M8
     */
    export function motorGetAbsoluteAngle(motor: MotorPort): number {
        let motorData = refreshMotorData(motor, MOTOR_DATA_REFRESH_ANGLE)
        if (motorData.length != MOTOR_DATA_RECORD_LENGTH
            || (motorData[0] & MOTOR_DATA_ANGLE_VALID) == 0) {
            return 0
        }
        return normalizeAngleX10(readI32Le(motorData, MOTOR_DATA_ABSOLUTE_ANGLE_OFFSET)) / 10
    }

    //% group="Readings"
    //% blockId=smartmotor_motor_relative_angle block="motor $motor relative angle"
    //% motor.defl=smartMotor.MotorPort.M5
    //% weight=68
    /**
     * Read the current motor relative angle in degrees.
     * @param motor motor port M5-M8
     */
    export function motorGetRelativeAngle(motor: MotorPort): number {
        let motorData = refreshMotorData(motor, MOTOR_DATA_REFRESH_ANGLE)
        if (motorData.length != MOTOR_DATA_RECORD_LENGTH
            || (motorData[0] & MOTOR_DATA_ANGLE_VALID) == 0) {
            return 0
        }
        return readI32Le(motorData, MOTOR_DATA_RELATIVE_ANGLE_OFFSET) / 10
    }

    //% group="Gyroscope"
    //% blockId=smartmotor_gyro_reset block="gyroscope reset"
    //% weight=60
    /**
     * Reset the gyroscope attitude angles.
     */
    export function resetGyroAngle(): void {
        i2cCommandSend(COMMAND_GYRO_RESET, [])
        gyroSpeedHasSample = [false, false, false]
        let deadline = input.runningTime() + GYRO_RESET_CONFIRM_TIMEOUT_MS
        while (input.runningTime() < deadline) {
            let data = readRegisters(REGISTER_GYRO_ANGLE_START, 12,
                deadline - input.runningTime())
            if (lastQueryWasSuccessful
                && Math.abs(readI32Le(data, 0)) <= GYRO_RESET_CONFIRM_TOLERANCE_X10
                && Math.abs(readI32Le(data, 4)) <= GYRO_RESET_CONFIRM_TOLERANCE_X10
                && Math.abs(readI32Le(data, 8)) <= GYRO_RESET_CONFIRM_TOLERANCE_X10) {
                return
            }
            let remainingMs = deadline - input.runningTime()
            if (remainingMs > 0) {
                delayMs(Math.min(GYRO_RESET_CONFIRM_POLL_INTERVAL_MS, remainingMs))
            }
        }
    }

    //% group="Gyroscope"
    //% blockId=smartmotor_gyro_angular_speed block="gyroscope $axis angular speed (degrees/s)"
    //% axis.defl=smartMotor.GyroAxis.Pitch
    //% weight=59
    /**
     * Read the gyroscope angular speed in degrees per second.
     * @param axis pitch, yaw, or roll axis
     */
    export function readGyroAngularSpeed(axis: GyroAxis): number {
        let now = input.runningTime()
        let angle = readGyroAngle(axis)
        let index = axis
        if (!gyroSpeedHasSample[index]) {
            gyroSpeedLastAngle[index] = angle
            gyroSpeedLastTime[index] = now
            gyroSpeedHasSample[index] = true
            return 0
        }
        let elapsedMs = now - gyroSpeedLastTime[index]
        if (elapsedMs <= 0) {
            return 0
        }
        let angularSpeed = (angle - gyroSpeedLastAngle[index]) * 1000 / elapsedMs
        gyroSpeedLastAngle[index] = angle
        gyroSpeedLastTime[index] = now
        return angularSpeed
    }

    //% group="Gyroscope"
    //% blockId=smartmotor_gyro_angle block="gyroscope $axis angle (degrees)"
    //% axis.defl=smartMotor.GyroAxis.Pitch
    //% weight=58
    /**
     * Read the gyroscope attitude angle in degrees.
     * @param axis pitch, yaw, or roll axis
     */
    export function readGyroAngle(axis: GyroAxis): number {
        let data = readRegisters(REGISTER_GYRO_ANGLE_START + axis * 4, 4)
        return data.length == 4 ? readI32Le(data, 0) / 10 : 0
    }

    //% group="Information"
    //% blockId=smartmotor_firmware_version block="firmware version"
    //% weight=50
    /**
     * Read the controller firmware version.
     */
    export function readVersion(): string {
        let data = i2cQueryRead(COMMAND_VERSION, [], 3)
        return "V " + data[0] + "." + data[1] + "." + data[2]
    }
}
