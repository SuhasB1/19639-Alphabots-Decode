package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.*;

@Config
@TeleOp(name = "set servo test")
public class servoSetPosition extends OpMode {
    public Servo hoodLeft;
    public Servo hood;

    @Override
    public void init() {
        hoodLeft = hardwareMap.get(Servo.class, "hoodLeft");
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.REVERSE);
    }
    @Override
    public void loop() {
        hoodLeft.setPosition(RobotConstants.hoodMin);
        hood.setPosition(RobotConstants.hoodMin);
    }

}
