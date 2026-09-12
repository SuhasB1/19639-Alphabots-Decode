package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.*;
@Config
@TeleOp
public class servoSetPosition extends OpMode {
    public Servo leverLeft;
    public Servo leverRight;

    public static double leftPos = 0.1;
    public static double rightPos = 0.05;



    @Override
    public void init() {
        leverLeft = hardwareMap.get(Servo.class, "leverLeft");
        leverRight = hardwareMap.get(Servo.class, "leverRight");
        leverRight.setDirection(Servo.Direction.REVERSE);
        leverLeft.setDirection(Servo.Direction.REVERSE);

    }
    @Override
    public void loop() {
        leverLeft.setPosition(leftPos);
        leverRight.setPosition(rightPos);
    }

}
