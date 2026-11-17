/**
 * micro:Driver Robot — บล็อกแบบ PTKidsBITRobot สำหรับบอร์ด DFRobot micro:Driver (DFR0548)
 * ต้องเพิ่ม Extension https://github.com/DFRobot/pxt-motor ในโปรเจกต์ก่อน
 *
 * หุ่น: มอเตอร์ M1 = ล้อซ้าย, M2 = ล้อขวา (ตั้งกลับทิศได้)
 *       Servo S1 = แขนยก, S2 = ตัวคีบ (ผ่านชิป I2C บนบอร์ด)
 *       เซนเซอร์เส้นแบบดิจิทัล (0/1) 2–9 ตัว ต่อขา micro:bit ที่บอร์ดมีให้:
 *       P0 P1 P2 P8 P12 P13 P14 P15 P16
 *       ค่าเริ่มต้น 6 ตัว เรียงซ้าย -> ขวา: P8, P12, P13, P14, P15, P16
 *       Ultrasonic (ถ้าติด): Trig = P1, Echo = P2 (เหมือน PTKidsBiT)
 *
 * ต่างจาก PTKidsBiT:
 *   - บอร์ดนี้ไม่มีชิป ADC: เซนเซอร์อ่านได้แค่ เห็นเส้น / ไม่เห็นเส้น (ไม่มี ADCRead)
 *   - ไม่มีไฟ NeoPixel: ไม่มี LED Color / Sensor Color
 *   - SensorCalibrate จำว่าเซนเซอร์แต่ละตัวให้ 0 หรือ 1 บนเส้น (ใช้ได้ทั้งโมดูลที่เส้นดำ = 0 และ = 1)
 */

enum MD_Motor {
    //% block="Left"
    Left,
    //% block="Right"
    Right
}

enum MD_Dir {
    //% block="Left"
    Left,
    //% block="Right"
    Right
}

enum MD_Find {
    //% block="Left"
    Left,
    //% block="Center"
    Center,
    //% block="Right"
    Right
}

enum MD_Servo {
    //% block="S1"
    S1 = 1,
    //% block="S2"
    S2 = 2,
    //% block="S3"
    S3 = 3,
    //% block="S4"
    S4 = 4,
    //% block="S5"
    S5 = 5,
    //% block="S6"
    S6 = 6,
    //% block="S7"
    S7 = 7,
    //% block="S8"
    S8 = 8
}

enum MD_TurnSensor {
    //% block="Center"
    Center = 0,
    //% block="1"
    S1 = 1,
    //% block="2"
    S2 = 2,
    //% block="3"
    S3 = 3,
    //% block="4"
    S4 = 4,
    //% block="5"
    S5 = 5,
    //% block="6"
    S6 = 6,
    //% block="7"
    S7 = 7,
    //% block="8"
    S8 = 8,
    //% block="9"
    S9 = 9
}

//% color="#51cb57" icon="" block="micro:Driver Robot"
//% groups='["Motor Control", "Servo Control", "Sensor", "Line Follower", "Gripper"]'
namespace MDRobot {
    // ---------- เซนเซอร์ ----------
    // ค่าเริ่มต้นตามหุ่นจริง: 8 ตัว เรียงซ้าย -> ขวา P0 P1 P2 P8 P12 P13 P14 P15
    let PINS: DigitalPin[]
    let N: number
    let pullReady: boolean
    // ค่าดิจิทัลที่เซนเซอร์ให้เมื่ออยู่บนเส้น (ได้จาก Calibrate) ค่าเริ่มต้น: เส้นดำ = 1
    let LINE_LEVEL: number[]
    let GROUND_LEVEL: number[]
    // ค่าเซนเซอร์ล่าสุด 0 (พื้น) หรือ 1000 (เส้น)
    let V: number[]

    let pdFirst: number
    let pdLast: number
    let previousError: number
    let lastPos: number

    // ---------- มอเตอร์ ----------
    let leftMotor: motor.Motors
    let rightMotor: motor.Motors
    let leftReverse: boolean
    let rightReverse: boolean
    let compLeft: number
    let compRight: number
    let brakeMs: number

    // ---------- ตัวคีบ ----------
    let armServo: MD_Servo
    let gripServo: MD_Servo
    let armUpDeg: number
    let armDownDeg: number
    let gripOpenDeg: number
    let gripCloseDeg: number
    let armStepMs: number
    let armNow: number

    // MakeCode อาจรันโค้ด on start ของผู้ใช้ก่อนบรรทัดตั้งค่าตัวแปรในไฟล์นี้
    // จึงตั้งค่าเริ่มต้นทั้งหมดในฟังก์ชันนี้ และเรียกทุกครั้งที่ใช้บล็อก
    let ready: boolean
    function initState() {
        if (ready) return
        ready = true
        PINS = [DigitalPin.P0, DigitalPin.P1, DigitalPin.P2, DigitalPin.P8, DigitalPin.P12, DigitalPin.P13, DigitalPin.P14, DigitalPin.P15]
        N = 8
        LINE_LEVEL = [1, 1, 1, 1, 1, 1, 1, 1, 1]
        GROUND_LEVEL = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        V = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        pdFirst = 0
        pdLast = 7
        previousError = 0
        lastPos = 3500
        leftMotor = motor.Motors.M1
        rightMotor = motor.Motors.M2
        leftReverse = false
        rightReverse = false
        compLeft = 0
        compRight = 0
        brakeMs = 0
        armServo = MD_Servo.S1
        gripServo = MD_Servo.S2
        armUpDeg = 45
        armDownDeg = 110
        gripOpenDeg = 45
        gripCloseDeg = 0
        armStepMs = 8
        armNow = -1
        pullReady = false
    }

    // ================= ภายใน =================

    function servoIndex(s: MD_Servo): motor.Servos {
        if (s == MD_Servo.S1) return motor.Servos.S1
        if (s == MD_Servo.S2) return motor.Servos.S2
        if (s == MD_Servo.S3) return motor.Servos.S3
        if (s == MD_Servo.S4) return motor.Servos.S4
        if (s == MD_Servo.S5) return motor.Servos.S5
        if (s == MD_Servo.S6) return motor.Servos.S6
        if (s == MD_Servo.S7) return motor.Servos.S7
        return motor.Servos.S8
    }

    // ความเร็ว -100..100 -> pxt-motor (ทิศ CW/CCW, ความเร็ว 0..255)
    function setOne(m: motor.Motors, v: number, reverse: boolean) {
        v = Math.round(Math.constrain(v, -100, 100))
        if (reverse) v = -v
        if (v == 0) {
            motor.motorStop(m)
        } else if (v > 0) {
            motor.MotorRun(m, motor.Dir.CW, Math.round(v * 255 / 100))
        } else {
            motor.MotorRun(m, motor.Dir.CCW, Math.round(-v * 255 / 100))
        }
    }

    function drive(left: number, right: number) {
        setOne(leftMotor, left + compLeft, leftReverse)
        setOne(rightMotor, right + compRight, rightReverse)
    }

    function stopAll() {
        motor.motorStop(leftMotor)
        motor.motorStop(rightMotor)
    }

    function stopWithBrake(left: number, right: number) {
        if (brakeMs > 0) {
            drive(-left, -right)
            basic.pause(brakeMs)
        }
        stopAll()
    }

    function readAll() {
        if (!pullReady) setupPins()
        for (let i = 0; i < N; i++) {
            V[i] = pins.digitalReadPin(PINS[i]) == LINE_LEVEL[i] ? 1000 : 0
        }
    }

    function onLine(i: number): boolean {
        return V[i] > 500
    }

    function countOnLine(): number {
        let c = 0
        for (let i = 0; i < N; i++) {
            if (V[i] > 500) c += 1
        }
        return c
    }

    function positionFromV(): number {
        let n = pdLast - pdFirst + 1
        let sum = 0
        let wsum = 0
        for (let k = 0; k < n; k++) {
            sum += V[pdFirst + k]
            wsum += V[pdFirst + k] * k * 1000
        }
        if (sum == 0) {
            // หลุดเส้น: คืนค่าสุดขอบฝั่งที่เห็นเส้นครั้งสุดท้าย
            if (lastPos < (n - 1) * 500) return 0
            return (n - 1) * 1000
        }
        lastPos = wsum / sum
        return Math.round(lastPos)
    }

    function pdFromV(minSpeed: number, maxSpeed: number, kp: number, kd: number) {
        let n = pdLast - pdFirst + 1
        let error = positionFromV() - (n - 1) * 500
        let pd = kp * error + kd * (error - previousError)
        previousError = error
        let l = minSpeed + pd
        let r = minSpeed - pd
        if (l > maxSpeed) {
            r = r - (l - maxSpeed)
            l = maxSpeed
        }
        if (r > maxSpeed) {
            l = l - (r - maxSpeed)
            r = maxSpeed
        }
        drive(Math.max(l, -maxSpeed), Math.max(r, -maxSpeed))
    }

    // จำนวนเซนเซอร์ฝั่งซ้าย/ขวาที่ใช้ตัดสินแยก
    function sideCount(): number {
        return Math.max(1, Math.idiv(N + 1, 3))
    }

    function junction(find: MD_Find): boolean {
        if (find == MD_Find.Center) {
            return countOnLine() >= Math.max(2, Math.idiv(N * 3 + 3, 4))
        }
        let k = sideCount()
        if (find == MD_Find.Left) {
            for (let i = 0; i < k; i++) {
                if (!onLine(i)) return false
            }
            return !onLine(N - 1)
        }
        for (let i = N - k; i < N; i++) {
            if (!onLine(i)) return false
        }
        return !onLine(0)
    }

    // เซนเซอร์แถวกลาง (ใช้ในช่วงแรกของ TurnLINE)
    function middleOnLine(): boolean {
        let edge = Math.idiv(N, 4)
        for (let i = edge; i < N - edge; i++) {
            if (onLine(i)) return true
        }
        return false
    }

    function turnSensorOnLine(sensor: MD_TurnSensor): boolean {
        if (sensor == MD_TurnSensor.Center) {
            if (N % 2 == 1) return onLine(Math.idiv(N, 2))
            return onLine(N / 2 - 1) || onLine(N / 2)
        }
        let i = Math.constrain(sensor - 1, 0, N - 1)
        return onLine(i)
    }

    // ปลดตัวต้านทานดึงลงภายในของ micro:bit ทุกขาเซนเซอร์ (ค่าเริ่มต้นเป็น PullDown
    // ซึ่งอาจทำให้เซนเซอร์แบบ open-collector อ่านค่า 1 ไม่ขึ้น)
    function setupPins() {
        for (let i = 0; i < N; i++) {
            pins.setPull(PINS[i], PinPullMode.PullNone)
        }
        pullReady = true
    }

    function p0IsSensor(): boolean {
        for (let i = 0; i < N; i++) {
            if (PINS[i] == DigitalPin.P0) return true
        }
        return false
    }

    // เสียงของ micro:bit ออกที่ขา P0 ด้วย ถ้า P0 ต่อเซนเซอร์อยู่จะไม่เล่นเสียง (กันสัญญาณชนกัน)
    function tone(freq: number) {
        if (p0IsSensor()) {
            basic.pause(100)
            return
        }
        music.playTone(freq, music.beat(BeatFraction.Quarter))
    }

    function waitButtonA() {
        while (!input.buttonIsPressed(Button.A)) {
            basic.pause(10)
        }
        while (input.buttonIsPressed(Button.A)) {
            basic.pause(10)
        }
    }

    // อ่าน 20 ครั้ง ครั้งละ 50 ms แล้วใช้ค่าที่ได้บ่อยกว่า (0 หรือ 1)
    function readMajority(): number[] {
        if (!pullReady) setupPins()
        let ones = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        for (let k = 0; k < 20; k++) {
            for (let i = 0; i < N; i++) {
                ones[i] += pins.digitalReadPin(PINS[i])
            }
            basic.pause(50)
        }
        let r = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        for (let i = 0; i < N; i++) {
            r[i] = ones[i] >= 10 ? 1 : 0
        }
        return r
    }

    function printLevels(title: string, values: number[]) {
        let s = title
        for (let i = 0; i < N; i++) {
            s = s + " " + values[i]
        }
        serial.writeLine(s)
    }

    // ================= Motor Control =================

    //% blockId=md_motorWrite block="motorWrite %m|Speed %speed"
    //% speed.min=-100 speed.max=100 speed.defl=50
    //% group="Motor Control" weight=100
    export function motorWrite(m: MD_Motor, speed: number): void {
        initState()
        if (m == MD_Motor.Left) setOne(leftMotor, speed + compLeft, leftReverse)
        else setOne(rightMotor, speed + compRight, rightReverse)
    }

    //% blockId=md_motorGo block="Motor Left %left|Motor Right %right"
    //% left.min=-100 left.max=100 left.defl=50 right.min=-100 right.max=100 right.defl=50
    //% inlineInputMode=inline
    //% group="Motor Control" weight=99
    export function motorGo(left: number, right: number): void {
        initState()
        drive(left, right)
    }

    //% blockId=md_turn block="Turn %dir|Speed %speed"
    //% speed.min=0 speed.max=100 speed.defl=50
    //% group="Motor Control" weight=98
    export function Turn(dir: MD_Dir, speed: number): void {
        initState()
        if (dir == MD_Dir.Left) drive(0, speed)
        else drive(speed, 0)
    }

    //% blockId=md_spin block="Spin %dir|Speed %speed"
    //% speed.min=0 speed.max=100 speed.defl=50
    //% group="Motor Control" weight=97
    export function Spin(dir: MD_Dir, speed: number): void {
        initState()
        if (dir == MD_Dir.Left) drive(-speed, speed)
        else drive(speed, -speed)
    }

    //% blockId=md_stop block="Motor Stop"
    //% group="Motor Control" weight=96
    export function motorStop(): void {
        initState()
        stopAll()
    }

    /**
     * เลือกช่องมอเตอร์ของล้อซ้าย-ขวา และกลับทิศ (ใช้เมื่อสั่งเดินหน้าแล้วล้อหมุนถอยหลัง)
     */
    //% blockId=md_setmotors block="Set Motor Left %left|reverse %lrev|Right %right|reverse %rrev"
    //% left.defl=motor.Motors.M1 right.defl=motor.Motors.M2
    //% lrev.shadow="toggleOnOff" rrev.shadow="toggleOnOff"
    //% inlineInputMode=inline
    //% group="Motor Control" weight=91 advanced=true
    export function setMotors(left: motor.Motors, lrev: boolean, right: motor.Motors, rrev: boolean): void {
        initState()
        leftMotor = left
        rightMotor = right
        leftReverse = lrev
        rightReverse = rrev
    }

    //% blockId=md_compensate block="Compensate Left %left|Compensate Right %right"
    //% left.min=-100 left.max=100 left.defl=0 right.min=-100 right.max=100 right.defl=0
    //% inlineInputMode=inline
    //% group="Motor Control" weight=90 advanced=true
    export function motorCompensate(left: number, right: number): void {
        initState()
        compLeft = left
        compRight = right
    }

    //% blockId=md_brake block="Set Brake Time %ms ms"
    //% ms.min=0 ms.max=200 ms.defl=0
    //% group="Motor Control" weight=89 advanced=true
    export function setBrakeTime(ms: number): void {
        initState()
        brakeMs = ms
    }

    // ================= Servo Control =================

    //% blockId=md_servo block="Servo %servo|Degree %degree"
    //% degree.min=0 degree.max=180 degree.defl=90
    //% group="Servo Control" weight=100
    export function servoWrite(servo: MD_Servo, degree: number): void {
        initState()
        motor.servo(servoIndex(servo), degree)
        if (servo == armServo) armNow = degree
    }

    /**
     * หยุดจ่ายสัญญาณ Servo (Servo จะคลายตัว ไม่ค้างตำแหน่ง)
     */
    //% blockId=md_servo_stop block="Servo Stop %servo"
    //% group="Servo Control" weight=99
    export function servoStop(servo: MD_Servo): void {
        initState()
        motor.servoOff(servoIndex(servo), 0)
    }

    // ================= Sensor =================

    /**
     * ตั้งจำนวนเซนเซอร์เส้น (2–9) ใช้ร่วมกับ Set Line Sensor ... Pin
     */
    //% blockId=md_sensorcount block="Set Line Sensor Count %n"
    //% n.min=2 n.max=9 n.defl=8
    //% group="Sensor" weight=100
    export function setSensorCount(n: number): void {
        initState()
        N = Math.constrain(Math.round(n), 2, 9)
        while (PINS.length < N) PINS.push(DigitalPin.P16)
        pullReady = false
        pdFirst = 0
        pdLast = N - 1
        lastPos = (N - 1) * 500
        previousError = 0
    }

    /**
     * ตั้งขาของเซนเซอร์ตัวที่ ... (นับจากซ้าย เริ่มที่ 1)
     */
    //% blockId=md_sensorpin block="Set Line Sensor %index|Pin %pin"
    //% index.min=1 index.max=9 index.defl=1
    //% pin.defl=DigitalPin.P0
    //% group="Sensor" weight=99
    export function setSensorPin(index: number, pin: DigitalPin): void {
        initState()
        let i = Math.constrain(Math.round(index), 1, 9) - 1
        while (PINS.length <= i) PINS.push(DigitalPin.P16)
        PINS[i] = pin
        pullReady = false
    }

    /**
     * อ่านเซนเซอร์ตัวที่ ... ได้ 1 = เห็นเส้น, 0 = เห็นพื้น (ตามค่าที่ Calibrate)
     */
    //% blockId=md_sensorread block="SensorRead %index"
    //% index.min=1 index.max=9 index.defl=1
    //% group="Sensor" weight=95
    export function SensorRead(index: number): number {
        initState()
        readAll()
        let i = Math.constrain(Math.round(index), 1, N) - 1
        return onLine(i) ? 1 : 0
    }

    /**
     * ระยะจาก Ultrasonic (cm) เลือกขา Trig / Echo เอง (ห้ามใช้ขาเดียวกับเซนเซอร์เส้น)
     * ไม่มีเสียงสะท้อนหรือเกิน 400 cm ได้ 400
     */
    //% blockId=md_distance block="GETDistance Trig %trig|Echo %echo"
    //% trig.defl=DigitalPin.P16 echo.defl=DigitalPin.P16
    //% group="Sensor" weight=90 advanced=true
    export function distanceRead(trig: DigitalPin, echo: DigitalPin): number {
        initState()
        pins.setPull(echo, PinPullMode.PullNone)
        pins.digitalWritePin(trig, 0)
        control.waitMicros(2)
        pins.digitalWritePin(trig, 1)
        control.waitMicros(10)
        pins.digitalWritePin(trig, 0)
        let d = pins.pulseIn(echo, PulseValue.High, 500 * 58)
        let x = d / 58
        if (x <= 0 || x > 400) return 400
        return Math.round(x)
    }

    // ================= Line Follower =================

    /**
     * Calibrate แบบ PTKidsBiT: วางเซนเซอร์บนเส้น กด A -> วางบนพื้น กด A
     * จำว่าเซนเซอร์แต่ละตัวให้ 0 หรือ 1 บนเส้น ตัวที่ให้ค่าเท่ากันทั้งสองแบบจะแสดงเลขบนจอ
     */
    //% blockId=md_calibrate block="SensorCalibrate"
    //% group="Line Follower" weight=100
    export function SensorCalibrate(): void {
        initState()
        stopAll()
        serial.writeLine("")
        tone(587)
        tone(784)
        basic.showLeds(`
            # # # # #
            # # # # #
            # # # # #
            # # # # #
            # # # # #
            `)
        waitButtonA()
        tone(784)
        let lineLv = readMajority()
        tone(784)
        basic.showLeds(`
            # # # # #
            # . . . #
            # . . . #
            # . . . #
            # # # # #
            `)
        waitButtonA()
        tone(784)
        let groundLv = readMajority()
        tone(784)
        tone(587)
        let bad = 0
        for (let i = 0; i < N; i++) {
            if (lineLv[i] != groundLv[i]) {
                LINE_LEVEL[i] = lineLv[i]
                GROUND_LEVEL[i] = groundLv[i]
            } else {
                bad += 1
                basic.showNumber(i + 1)
                basic.pause(300)
            }
        }
        serial.writeLine("------------------------------------")
        printLevels("Color Line:", LINE_LEVEL)
        printLevels("Color Ground:", GROUND_LEVEL)
        if (bad == 0) basic.showIcon(IconNames.Yes)
        else basic.showIcon(IconNames.Sad)
        previousError = 0
    }

    /**
     * ใส่ค่าเอง: ค่าที่เซนเซอร์แต่ละตัวให้เมื่ออยู่บนเส้น (0 หรือ 1) เรียงจากซ้าย ใช้แทน SensorCalibrate
     */
    //% blockId=md_setcolor block="SETColorLine $line"
    //% group="Line Follower" weight=95
    export function ValueSensorSET(line: number[]): void {
        initState()
        for (let i = 0; i < 9; i++) {
            if (i < line.length) {
                LINE_LEVEL[i] = line[i] == 0 ? 0 : 1
                GROUND_LEVEL[i] = 1 - LINE_LEVEL[i]
            }
        }
        previousError = 0
    }

    //% blockId=md_print block="PrintSensorValue"
    //% group="Line Follower" weight=94
    export function PrintSensorValue(): void {
        initState()
        if (!pullReady) setupPins()
        let s = "Sensor Value:"
        for (let i = 0; i < N; i++) {
            s = s + " " + pins.digitalReadPin(PINS[i])
        }
        serial.writeLine(s)
    }

    /**
     * ตำแหน่งเส้น 0 (ซ้ายสุด) ... (จำนวนเซนเซอร์ - 1) x 1000 (ขวาสุด) เช่น 6 ตัว = 0–5000 กลาง 2500
     */
    //% blockId=md_position block="GETPosition"
    //% group="Line Follower" weight=93
    export function GETPosition(): number {
        initState()
        readAll()
        return positionFromV()
    }

    //% blockId=md_pd block="Min Speed %minSpeed|Max Speed %maxSpeed|KP %kp|KD %kd"
    //% minSpeed.min=0 minSpeed.max=100 minSpeed.defl=30
    //% maxSpeed.min=0 maxSpeed.max=100 maxSpeed.defl=100
    //% kp.defl=0.01 kd.defl=0
    //% inlineInputMode=inline
    //% group="Line Follower" weight=90
    export function Follower(minSpeed: number, maxSpeed: number, kp: number, kd: number): void {
        initState()
        readAll()
        pdFromV(minSpeed, maxSpeed, kp, kd)
    }

    //% blockId=md_find block="Find %find|Min Speed %minSpeed|Max Speed %maxSpeed|KP %kp|KD %kd"
    //% find.defl=MD_Find.Center
    //% minSpeed.min=0 minSpeed.max=100 minSpeed.defl=30
    //% maxSpeed.min=0 maxSpeed.max=100 maxSpeed.defl=100
    //% kp.defl=0.01 kd.defl=0
    //% inlineInputMode=inline
    //% group="Line Follower" weight=85
    export function ForwardLINE(find: MD_Find, minSpeed: number, maxSpeed: number, kp: number, kd: number): void {
        initState()
        readAll()
        while (junction(MD_Find.Center)) {
            drive(minSpeed, minSpeed)
            basic.pause(1)
            readAll()
        }
        drive(50, 50)
        basic.pause(20)
        while (true) {
            readAll()
            if (junction(find)) {
                drive(50, 50)
                basic.pause(20)
                stopWithBrake(50, 50)
                break
            }
            pdFromV(minSpeed, maxSpeed, kp, kd)
        }
    }

    //% blockId=md_findcount block="Find %find|Count %count|Min Speed %minSpeed|Max Speed %maxSpeed|KP %kp|KD %kd"
    //% find.defl=MD_Find.Center
    //% count.min=1 count.defl=1
    //% minSpeed.min=0 minSpeed.max=100 minSpeed.defl=30
    //% maxSpeed.min=0 maxSpeed.max=100 maxSpeed.defl=100
    //% kp.defl=0.01 kd.defl=0
    //% inlineInputMode=inline
    //% group="Line Follower" weight=84
    export function ForwardLINECount(find: MD_Find, count: number, minSpeed: number, maxSpeed: number, kp: number, kd: number): void {
        initState()
        let found = 0
        while (true) {
            readAll()
            if (junction(find)) {
                found += 1
                if (found >= count) {
                    stopWithBrake(minSpeed, minSpeed)
                    break
                }
                while (junction(find)) {
                    drive(minSpeed, minSpeed)
                    basic.pause(1)
                    readAll()
                }
                basic.pause(100)
                readAll()
            }
            pdFromV(minSpeed, maxSpeed, kp, kd)
        }
    }

    /**
     * รูปแบบเรียงจากเซนเซอร์ซ้าย: 1 = ต้องเห็นเส้น, 0 = ต้องเห็นพื้น, - = ไม่สนใจ เช่น 8 ตัว "1------1"
     */
    //% blockId=md_findcustom block="Find %pattern|Min Speed %minSpeed|Max Speed %maxSpeed|KP %kp|KD %kd"
    //% pattern.defl="1------1"
    //% minSpeed.min=0 minSpeed.max=100 minSpeed.defl=30
    //% maxSpeed.min=0 maxSpeed.max=100 maxSpeed.defl=100
    //% kp.defl=0.01 kd.defl=0
    //% inlineInputMode=inline
    //% group="Line Follower" weight=83
    export function ForwardLINECustom(pattern: string, minSpeed: number, maxSpeed: number, kp: number, kd: number): void {
        initState()
        let want = [2, 2, 2, 2, 2, 2, 2, 2, 2]
        let need = 0
        for (let i = 0; i < N; i++) {
            let c = pattern.charAt(i)
            if (c == "1") {
                want[i] = 1
                need += 1
            } else if (c == "0") {
                want[i] = 0
                need += 1
            }
        }
        while (true) {
            readAll()
            let match = 0
            for (let i = 0; i < N; i++) {
                if (want[i] == 1 && onLine(i)) match += 1
                else if (want[i] == 0 && !onLine(i)) match += 1
            }
            if (match >= need) {
                stopWithBrake(minSpeed, minSpeed)
                break
            }
            pdFromV(minSpeed, maxSpeed, kp, kd)
        }
    }

    //% blockId=md_time block="Time %ms|Min Speed %minSpeed|Max Speed %maxSpeed|KP %kp|KD %kd"
    //% ms.shadow="timePicker" ms.defl=200
    //% minSpeed.min=0 minSpeed.max=100 minSpeed.defl=30
    //% maxSpeed.min=0 maxSpeed.max=100 maxSpeed.defl=100
    //% kp.defl=0.01 kd.defl=0
    //% inlineInputMode=inline
    //% group="Line Follower" weight=80
    export function ForwardTIME(ms: number, minSpeed: number, maxSpeed: number, kp: number, kd: number): void {
        initState()
        let t = control.millis()
        while (control.millis() - t < ms) {
            readAll()
            pdFromV(minSpeed, maxSpeed, kp, kd)
        }
        stopWithBrake(minSpeed, minSpeed)
    }

    /**
     * หมุนให้พ้นเส้นเดิม แล้วหมุนจนเซนเซอร์ที่เลือกเจอเส้นใหม่ ความเร็ว Speed ช่วง Fast Time แล้วลดเหลือ 20
     * Center = เซนเซอร์ตัวกลาง (จำนวนคู่ = ตัวกลางตัวใดตัวหนึ่ง)
     */
    //% blockId=md_turnline block="TurnLINE %dir|Speed %speed|Sensor %sensor|Fast Time %fastTime"
    //% speed.min=0 speed.max=100 speed.defl=50
    //% sensor.defl=MD_TurnSensor.Center
    //% fastTime.shadow="timePicker" fastTime.defl=200
    //% inlineInputMode=inline
    //% group="Line Follower" weight=75
    export function TurnLINE(dir: MD_Dir, speed: number, sensor: MD_TurnSensor, fastTime: number): void {
        initState()
        let slow = 20
        let timer = control.millis()
        let s = 0
        while (true) {
            readAll()
            if (!middleOnLine()) break
            s = timer + fastTime - control.millis()
            if (s > speed) s = speed
            else if (s < 0) s = slow
            if (dir == MD_Dir.Left) drive(-s, s)
            else drive(s, -s)
        }
        while (true) {
            readAll()
            if (turnSensorOnLine(sensor)) break
            s = timer + fastTime - control.millis()
            if (s > speed) s = speed
            else if (s < 0) s = slow
            if (dir == MD_Dir.Left) drive(-s, s)
            else drive(s, -s)
        }
        if (dir == MD_Dir.Left) stopWithBrake(-s, s)
        else stopWithBrake(s, -s)
        previousError = 0
        lastPos = (pdLast - pdFirst) * 500
    }

    /**
     * เลือกเซนเซอร์ที่ใช้คำนวณ PD (นับจากซ้าย เริ่มที่ 1) ค่าเริ่มต้น = ทุกตัว
     */
    //% blockId=md_pdsensors block="Set PD Sensor from %first|to %last"
    //% first.min=1 first.max=9 first.defl=1 last.min=1 last.max=9 last.defl=8
    //% group="Line Follower" weight=70 advanced=true
    export function setPDSensors(first: number, last: number): void {
        initState()
        let f = Math.constrain(Math.round(first), 1, N) - 1
        let l = Math.constrain(Math.round(last), 1, N) - 1
        if (l > f) {
            pdFirst = f
            pdLast = l
            lastPos = (pdLast - pdFirst) * 500
            previousError = 0
        }
    }

    // ================= Gripper =================

    /**
     * ตั้งช่อง Servo และมุมของแขนกับตัวคีบ
     */
    //% blockId=md_setgripper block="Set Gripper Arm %arm|Up %up|Down %down|Grip %grip|Open %open|Close %close"
    //% arm.defl=MD_Servo.S1 grip.defl=MD_Servo.S2
    //% up.defl=45 down.defl=110 open.defl=45 close.defl=0
    //% inlineInputMode=inline
    //% group="Gripper" weight=100
    export function setGripper(arm: MD_Servo, up: number, down: number, grip: MD_Servo, open: number, close: number): void {
        initState()
        armServo = arm
        gripServo = grip
        armUpDeg = up
        armDownDeg = down
        gripOpenDeg = open
        gripCloseDeg = close
        armNow = -1
    }

    function moveArm(target: number) {
        if (armNow < 0) {
            motor.servo(servoIndex(armServo), target)
            armNow = target
            basic.pause(300)
        } else {
            while (armNow != target) {
                armNow += armNow < target ? 1 : -1
                motor.servo(servoIndex(armServo), armNow)
                basic.pause(armStepMs)
            }
        }
    }

    //% blockId=md_armup block="Arm Up"
    //% group="Gripper" weight=95
    export function armUp(): void {
        initState()
        moveArm(armUpDeg)
    }

    //% blockId=md_armdown block="Arm Down"
    //% group="Gripper" weight=94
    export function armDown(): void {
        initState()
        moveArm(armDownDeg)
    }

    //% blockId=md_gripopen block="Grip Open"
    //% group="Gripper" weight=93
    export function gripOpen(): void {
        initState()
        motor.servo(servoIndex(gripServo), gripOpenDeg)
        basic.pause(300)
    }

    //% blockId=md_gripclose block="Grip Close"
    //% group="Gripper" weight=92
    export function gripClose(): void {
        initState()
        motor.servo(servoIndex(gripServo), gripCloseDeg)
        basic.pause(300)
    }

    //% blockId=md_pick block="Pick Up"
    //% group="Gripper" weight=90
    export function pickUp(): void {
        initState()
        stopAll()
        gripOpen()
        armDown()
        basic.pause(150)
        gripClose()
        armUp()
    }

    //% blockId=md_put block="Put Down"
    //% group="Gripper" weight=89
    export function putDown(): void {
        initState()
        stopAll()
        armDown()
        basic.pause(150)
        gripOpen()
        drive(-30, -30)
        basic.pause(200)
        stopAll()
        armUp()
    }

    //% blockId=md_armspeed block="Set Arm Step %ms ms"
    //% ms.defl=8
    //% group="Gripper" weight=85 advanced=true
    export function setArmStep(ms: number): void {
        initState()
        armStepMs = ms
    }
}
