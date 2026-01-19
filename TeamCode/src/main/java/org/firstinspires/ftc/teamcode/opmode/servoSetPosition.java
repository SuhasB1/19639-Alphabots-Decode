package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.*;

@Config
@TeleOp(name = "set servo test")
public class servoSetPosition extends OpMode {
    public Servo gate;
    public Servo hood;
    AnalogInput pin0;

    @Override
    public void init() {
        gate = hardwareMap.get(Servo.class, "gate");
        hood = hardwareMap.get(Servo.class, "hood");
        pin0 = hardwareMap.analogInput.get("analog0");
        hood.setDirection(Servo.Direction.REVERSE);
    }
    @Override
    public void loop() {
        gate.setPosition(0);
        hood.setPosition(1);
        telemetry.addData("voltage", pin0.getVoltage());
        telemetry.addData("mm distance", pin0.getVoltage() / 3.3 * 100);
    }

}
