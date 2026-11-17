// =====================================================================
//  หุ่น micro:Driver — โปรแกรมทดสอบวิ่งตามเส้นอย่างเดียว
//  ใช้ Extension DF-Driver (https://github.com/DFRobot/pxt-motor) เฉพาะสั่งมอเตอร์และ Servo
//  ส่วนอ่านเซนเซอร์ Calibrate และคำนวณ PD เขียนเองทั้งหมดในไฟล์นี้ (ไม่ต้องใช้ micro:Driver Robot)
//
//  เซนเซอร์ดิจิทัล 8 ตัว ซ้าย -> ขวา: P0 P1 P2 P8 P12 P13 P14 P15
//  มอเตอร์ M1 = ซ้าย, M2 = ขวา (กลับทิศทั้งสองล้อ เหมือน reverse ON ที่ใช้อยู่)
//  Servo S1 = แขน (ยก 80), S2 = ตัวคีบ (อ้า 90) ยกแขนไว้ตอนเริ่ม ไม่ให้ลากพื้น
//
//  วิธีใช้
//   1) เปิดเครื่อง: จอเต็ม = วางเซนเซอร์ทั้ง 8 ตัวบนสีดำ กด A
//                  จอกรอบ = วางทั้ง 8 ตัวบนพื้นขาว กด A
//      ถูก = ✓  | มีตัวเสีย = แสดงเลขตัวนั้น แล้วหน้าบึ้ง (ตัวนั้นจะไม่ถูกใช้)
//   2) กด A+B: โหมดดูเซนเซอร์ (แถวบน = ตัว 1-5, แถวล่าง = ตัว 6-8, แถวกลาง = ตำแหน่งเส้น)
//      กด A+B อีกครั้งเพื่อออก
//   3) กด B: เริ่มวิ่ง / กด B อีกครั้ง: หยุด
//   ห้ามใช้บล็อก Music: เสียงออกที่ P0 ซึ่งต่อเซนเซอร์ตัวที่ 1
// =====================================================================

// ------------------ ค่าที่ปรับได้ ------------------
let BASE_SPEED = 40      // ความเร็วทางตรง (0-100)
let MAX_SPEED = 100      // ความเร็วสูงสุดของล้อ
let KP = 0.015           // ค่า P (error อยู่ช่วง -3500 ถึง 3500)
let KD = 0.05            // ค่า D
let D_INTERVAL = 10      // คำนวณ D ทุกกี่ ms
let LEFT_REVERSE = true  // กลับทิศล้อซ้าย
let RIGHT_REVERSE = true // กลับทิศล้อขวา
let ARM_UP = 80          // มุมแขนยก (S1)
let GRIP_OPEN = 90       // มุมตัวคีบอ้า (S2)

// ------------------ ฮาร์ดแวร์ ------------------
let SENSOR_PINS = [DigitalPin.P0, DigitalPin.P1, DigitalPin.P2, DigitalPin.P8, DigitalPin.P12, DigitalPin.P13, DigitalPin.P14, DigitalPin.P15]
let N = 8

// ------------------ ตัวแปรทำงาน ------------------
let LINE_LEVEL = [1, 1, 1, 1, 1, 1, 1, 1]   // ค่าที่เซนเซอร์ให้เมื่ออยู่บนเส้น (ได้จาก Calibrate)
let OK = [true, true, true, true, true, true, true, true]
let onLineNow = [false, false, false, false, false, false, false, false]
let lastPos = 3500
let previousError = 0
let dTerm = 0
let lastDTime = 0
let running = false
let viewMode = false

// ================= มอเตอร์ (ใช้ Extension DF-Driver: DFRobot/pxt-motor) =================

// speed -100..100 -> motor.MotorRun (ทิศ CW / CCW, ความเร็ว 0..255)
function motorRaw(m: motor.Motors, speed: number, reverse: boolean) {
    let v = Math.constrain(Math.round(speed), -100, 100)
    if (reverse) {
        v = -v
    }
    if (v > 0) {
        motor.MotorRun(m, motor.Dir.CW, Math.idiv(v * 255, 100))
    } else if (v < 0) {
        motor.MotorRun(m, motor.Dir.CCW, Math.idiv(-v * 255, 100))
    } else {
        motor.motorStop(m)
    }
}

function drive(left: number, right: number) {
    motorRaw(motor.Motors.M1, left, LEFT_REVERSE)
    motorRaw(motor.Motors.M2, right, RIGHT_REVERSE)
}

function stopMotors() {
    motor.motorStopAll()
}

// ================= เซนเซอร์ =================

function setupSensorPins() {
    for (let i = 0; i < N; i++) {
        // เซนเซอร์ชุดนี้ส่งสัญญาณแอนะล็อก ต้องดึงลง (PullDown) จึงอ่านสีดำเป็น 0 ได้แน่นอน
        pins.setPull(SENSOR_PINS[i], PinPullMode.PullDown)
    }
}

function readSensors() {
    for (let i = 0; i < N; i++) {
        onLineNow[i] = OK[i] && pins.digitalReadPin(SENSOR_PINS[i]) == LINE_LEVEL[i]
    }
}

// ตำแหน่งเส้น 0 (ซ้ายสุด) .. 7000 (ขวาสุด), กลาง 3500
function linePosition(): number {
    let sum = 0
    let wsum = 0
    for (let i = 0; i < N; i++) {
        if (onLineNow[i]) {
            sum += 1
            wsum += i * 1000
        }
    }
    if (sum == 0) {
        // ไม่เห็นเส้น: เห็นครั้งล่าสุดที่ขอบ -> เลี้ยวกลับเต็มที่, เห็นตรงกลาง -> ใช้ค่าเดิม
        if (lastPos < 1000) return 0
        if (lastPos > 6000) return 7000
        return lastPos
    }
    lastPos = Math.idiv(wsum, sum)
    return lastPos
}

function waitA() {
    while (!input.buttonIsPressed(Button.A)) basic.pause(10)
    while (input.buttonIsPressed(Button.A)) basic.pause(10)
}

// อ่าน 20 ครั้ง ใช้ค่าที่ได้บ่อยกว่าของแต่ละตัว
function readMajority(): number[] {
    let ones = [0, 0, 0, 0, 0, 0, 0, 0]
    for (let k = 0; k < 20; k++) {
        for (let i = 0; i < N; i++) {
            ones[i] += pins.digitalReadPin(SENSOR_PINS[i])
        }
        basic.pause(50)
    }
    let r = [0, 0, 0, 0, 0, 0, 0, 0]
    for (let i = 0; i < N; i++) {
        r[i] = ones[i] >= 10 ? 1 : 0
    }
    return r
}

function calibrate() {
    basic.showLeds(`
        # # # # #
        # # # # #
        # # # # #
        # # # # #
        # # # # #
        `)
    waitA()
    let black = readMajority()
    basic.showLeds(`
        # # # # #
        # . . . #
        # . . . #
        # . . . #
        # # # # #
        `)
    waitA()
    let white = readMajority()
    let bad = 0
    for (let i = 0; i < N; i++) {
        if (black[i] != white[i]) {
            LINE_LEVEL[i] = black[i]
            OK[i] = true
        } else {
            OK[i] = false
            bad += 1
            basic.showNumber(i + 1)
            basic.pause(300)
        }
    }
    serial.writeLine("black: " + black.join(" "))
    serial.writeLine("white: " + white.join(" "))
    if (bad == 0) basic.showIcon(IconNames.Yes)
    else basic.showIcon(IconNames.Sad)
}

// ================= PD =================

function followLine() {
    readSensors()
    let error = linePosition() - 3500
    let now = control.millis()
    if (now - lastDTime > 200) {
        previousError = error
        dTerm = 0
        lastDTime = now
    } else if (now - lastDTime >= D_INTERVAL) {
        dTerm = KD * (error - previousError)
        previousError = error
        lastDTime = now
    }
    let pd = KP * error + dTerm
    let left = BASE_SPEED + pd
    let right = BASE_SPEED - pd
    // ล้อไหนเกิน MAX ให้ลดทั้งสองล้อเท่ากัน (รักษาผลต่างความเร็วไว้ = ยังเลี้ยวได้เท่าเดิม)
    if (left > MAX_SPEED) {
        right -= left - MAX_SPEED
        left = MAX_SPEED
    }
    if (right > MAX_SPEED) {
        left -= right - MAX_SPEED
        right = MAX_SPEED
    }
    drive(Math.max(left, -MAX_SPEED), Math.max(right, -MAX_SPEED))
}

// แสดงเซนเซอร์บนจอ
function showSensors() {
    readSensors()
    basic.clearScreen()
    for (let i = 0; i < N; i++) {
        if (onLineNow[i]) led.plot(i % 5, i < 5 ? 0 : 4)
    }
    let pos = linePosition()
    led.plot(Math.constrain(Math.idiv(pos * 5, 7001), 0, 4), 2)
    serial.writeValue("pos", pos)
}

// ================= เริ่มทำงาน =================

stopMotors()
motor.servo(motor.Servos.S1, ARM_UP)
motor.servo(motor.Servos.S2, GRIP_OPEN)
setupSensorPins()
calibrate()

input.onButtonPressed(Button.B, function () {
    viewMode = false
    running = !(running)
    if (running) {
        lastDTime = 0
        lastPos = 3500
        basic.clearScreen()
    } else {
        stopMotors()
        basic.showIcon(IconNames.Yes)
    }
})

input.onButtonPressed(Button.AB, function () {
    running = false
    stopMotors()
    viewMode = !(viewMode)
    if (!(viewMode)) basic.showIcon(IconNames.Yes)
})

basic.forever(function () {
    if (running) {
        // วนเองให้เร็ว (forever ปกติพักทุกรอบ) และพัก 1 ms ให้ปุ่ม B ทำงานได้
        while (running) {
            followLine()
            basic.pause(1)
        }
        stopMotors()
    } else if (viewMode) {
        showSensors()
        basic.pause(100)
    } else {
        basic.pause(20)
    }
})
