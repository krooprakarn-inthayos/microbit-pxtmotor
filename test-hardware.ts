// โปรแกรมทดสอบฮาร์ดแวร์ (ใช้แทน main.ts ชั่วคราว) - ยกล้อให้ลอยก่อนทดสอบ
// A   : มอเตอร์ซ้ายเดินหน้า 1 วินาที แล้วมอเตอร์ขวาเดินหน้า 1 วินาที
//       ถ้าล้อหมุนถอยหลัง -> ใช้ Set Motor ... reverse true ของล้อนั้น
//       ถ้าล้อขวาหมุนก่อน -> สลับ M1/M2 ในบล็อก Set Motor
// B   : ทดสอบ Servo แขน/ตัวคีบ ตามมุมใน setGripper (ปรับมุมจนตรงกลไกจริง)
// A+B : เปิด/ปิดแสดงค่าเซนเซอร์บนจอ (คอลัมน์ซ้าย->ขวา) และส่งค่าออก Serial
//       แต่ละเซนเซอร์ต้องเปลี่ยน 0/1 เมื่อเลื่อนผ่านเส้นดำ

MDRobot.setGripper(MD_Servo.S1, 80, 10, MD_Servo.S2, 90, 0)
let showSensor = false

input.onButtonPressed(Button.A, function () {
    MDRobot.motorGo(50, 0)
    basic.pause(1000)
    MDRobot.motorGo(0, 50)
    basic.pause(1000)
    MDRobot.motorStop()
})

input.onButtonPressed(Button.B, function () {
    MDRobot.armUp()
    basic.pause(500)
    MDRobot.armDown()
    basic.pause(500)
    MDRobot.gripClose()
    basic.pause(500)
    MDRobot.gripOpen()
    basic.pause(500)
    MDRobot.armUp()
})

input.onButtonPressed(Button.AB, function () {
    showSensor = !(showSensor)
    basic.clearScreen()
})

basic.forever(function () {
    if (showSensor) {
        MDRobot.PrintSensorValue()
        basic.clearScreen()
        // เซนเซอร์ 1-5 แถวบน, 6-8 แถวล่าง (ค่าดิบ 1 = ไฟติด)
        for (let i = 0; i < 8; i++) {
            if (pins.digitalReadPin([DigitalPin.P0, DigitalPin.P1, DigitalPin.P2, DigitalPin.P8, DigitalPin.P12, DigitalPin.P13, DigitalPin.P14, DigitalPin.P15][i]) == 1) {
                led.plot(i % 5, i < 5 ? 0 : 4)
            }
        }
        basic.pause(100)
    }
})
