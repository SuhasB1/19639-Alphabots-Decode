package org.firstinspires.ftc.teamcode.opmode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp
public class colorSensorTest extends OpMode {

    DigitalChannel pin0, pin1;
    @Override
    public void init() {
        pin0 = hardwareMap.digitalChannel.get("digital0");
        pin1 = hardwareMap.digitalChannel.get("digital1");
    }

    @Override
    public void loop() {
        telemetry.addData("pin 0", pin0.getState());
        telemetry.addData("pin1", pin1.getState());
        telemetry.update();
    }
}
