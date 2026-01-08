package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSubsystem {
    public DcMotorEx shooterLeft;
    public DcMotorEx shooterRight;
    public Servo hood;
    public Timer autoTimer = new Timer();
    private Telemetry telemetry;

    public ShooterSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        hood = hardwareMap.get(Servo.class, "hood");

        hood.setDirection(Servo.Direction.REVERSE);
        shooterRight.setDirection(DcMotorEx.Direction.REVERSE);
        this.telemetry = telemetry;
    }

    public void shoot() {
    }

    public void init() {
    }

    public void start() {
        autoTimer.resetTimer();
    }

    public void telemetry() {
    }
}

