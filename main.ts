// ตัวอย่างโปรแกรมแบบ PTKidsBiT สำหรับหุ่น micro:Driver (ใช้คู่กับ custom.ts)
// เซนเซอร์ดิจิทัล 8 ตัว เรียงซ้าย -> ขวา: P0 P1 P2 P8 P12 P13 P14 P15
// มอเตอร์ M1 = ซ้าย, M2 = ขวา | Servo S1 = แขน, S2 = ตัวคีบ
// ค่าเวลา ความเร็ว KP KD มุม Servo เป็นค่าตั้งต้น ต้องทดสอบและปรับตามหุ่นและสนามจริง

// ค่าตั้งต้นสำหรับจูน (ดูขั้นตอนจูนในคำอธิบาย)
let MIN = 40
let MAX = 100
let KP = 0.015
let KD = 0.05
// เซนเซอร์อยู่หน้าล้อมาก: เจอเส้นตัดแล้วต้องเดินต่อให้ "แกนล้อ" อยู่ตรงทางแยกก่อนเลี้ยว
// ปรับค่านี้จนหุ่นเลี้ยวแล้วตรงเส้นพอดี
let CROSS_MS = 250

// ---- มอเตอร์: หุ่นวิ่งถอยหลัง จึงกลับทิศทั้งสองล้อ (reverse ON) ----
MDRobot.setMotors(motor.Motors.M1, true, motor.Motors.M2, true)
// เบรกสั้นๆ ตอนหยุด ลดการไถลเลยเส้นหลัง TurnLINE / Find
MDRobot.setBrakeTime(30)

// ---- ตั้งค่าเซนเซอร์ (ค่าเริ่มต้นในบล็อกตรงกับหุ่นนี้อยู่แล้ว ใส่ไว้ให้เห็นชัด) ----
MDRobot.setSensorCount(8)
MDRobot.setSensorPin(1, DigitalPin.P0)
MDRobot.setSensorPin(2, DigitalPin.P1)
MDRobot.setSensorPin(3, DigitalPin.P2)
MDRobot.setSensorPin(4, DigitalPin.P8)
MDRobot.setSensorPin(5, DigitalPin.P12)
MDRobot.setSensorPin(6, DigitalPin.P13)
MDRobot.setSensorPin(7, DigitalPin.P14)
MDRobot.setSensorPin(8, DigitalPin.P15)

// ---- ตัวคีบ: S1 = แขน (ยก 80, วาง 10), S2 = ตัวคีบ (อ้า 90, คีบ 0) ----
MDRobot.setGripper(MD_Servo.S1, 80, 10, MD_Servo.S2, 90, 0)
MDRobot.armUp()
MDRobot.gripOpen()

// ---- Calibrate: A บนเส้น, A บนพื้น ----
MDRobot.SensorCalibrate()

// ---- กด B: ภารกิจตัวอย่าง ----
input.onButtonPressed(Button.B, function () {
    MDRobot.ForwardTIME(500, MIN, MAX, KP, KD)
    MDRobot.ForwardLINECount(MD_Find.Center, 1, MIN, MAX, KP, KD)
    MDRobot.ForwardTIME(CROSS_MS, MIN, MAX, KP, KD)
    basic.pause(100)
    MDRobot.TurnLINE(MD_Dir.Left, 50, MD_TurnSensor.Center, 200)
    MDRobot.ForwardTIME(200, MIN, MAX, KP, KD)
    MDRobot.ForwardLINECount(MD_Find.Center, 1, MIN, MAX, KP, KD)
    MDRobot.pickUp()
    MDRobot.TurnLINE(MD_Dir.Right, 50, MD_TurnSensor.Center, 200)
    MDRobot.TurnLINE(MD_Dir.Right, 50, MD_TurnSensor.Center, 200)
    MDRobot.ForwardTIME(200, MIN, MAX, KP, KD)
    MDRobot.ForwardLINECount(MD_Find.Center, 1, MIN, MAX, KP, KD)
    MDRobot.ForwardTIME(CROSS_MS, MIN, MAX, KP, KD)
    basic.pause(100)
    MDRobot.TurnLINE(MD_Dir.Right, 50, MD_TurnSensor.Center, 200)
    MDRobot.ForwardTIME(200, MIN, MAX, KP, KD)
    MDRobot.ForwardLINECount(MD_Find.Center, 1, MIN, MAX, KP, KD)
    MDRobot.ForwardTIME(150, MIN, MAX, KP, KD)
    MDRobot.putDown()
    basic.showIcon(IconNames.Happy)
})

// ---- กด A+B: วิ่งตามเส้นต่อเนื่อง (จูน KP KD) ----
let freeRun = false
input.onButtonPressed(Button.AB, function () {
    freeRun = !(freeRun)
    if (!(freeRun)) {
        MDRobot.motorStop()
    }
})
basic.forever(function () {
    if (freeRun) {
        MDRobot.Follower(MIN, MAX, KP, KD)
    }
})
