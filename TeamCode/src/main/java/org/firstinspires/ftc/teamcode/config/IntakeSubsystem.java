package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem {

    public enum WiggleState {
        IDLE,
        LEFT_DOWN,
        RIGHT_DOWN
    }

    private WiggleState wiggleState = WiggleState.IDLE;
    private final ElapsedTime wiggleTimer = new ElapsedTime();

    public double wiggleMoveTime = 0.25;

    private DcMotorEx intake;
    private Servo gate;

    public Servo leverLeft;
    public Servo leverRight;
    public Servo rgb;

    private DigitalChannel leftBallSensor;
    private DigitalChannel rightBallSensor;

    private final Telemetry telemetry;

    public double gateUpPos = 0.015;
    public double gateDownPos = 0.235;

    private double intakePower = 0.0;

    public double leverUpLeftPos = 0.03;
    public double leverUpRightPos = 0.11;
    public double leverDownLeftPos = 0.35;
    public double leverDownRightPos = 0.42;

    public IntakeSubsystem(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;

        intake = hw.get(DcMotorEx.class, "intake");
        rgb = hw.get(Servo.class, "rgb");
        gate = hw.get(Servo.class, "gate");

        leftBallSensor = hw.get(DigitalChannel.class, "pin0SL");
        rightBallSensor = hw.get(DigitalChannel.class, "pin0SR");

        leftBallSensor.setMode(DigitalChannel.Mode.INPUT);
        rightBallSensor.setMode(DigitalChannel.Mode.INPUT);

        leverLeft = hw.get(Servo.class, "leverLeft");
        leverRight = hw.get(Servo.class, "leverRight");

        leverRight.setDirection(Servo.Direction.REVERSE);
        leverLeft.setDirection(Servo.Direction.REVERSE);
    }

    public void init() {
        gateUp();
        leverUp();
        setPower(0);
        setRgbGreen();
    }

    public boolean leftBallDetected() {
        return leftBallSensor.getState();
    }

    public boolean rightBallDetected() {
        return rightBallSensor.getState();
    }

    public boolean ballDetected() {
        return leftBallDetected() || rightBallDetected();
    }

    public void updateRgbFromSensors() {
        if (ballDetected()) {
            setRgbOrange();
        } else {
            setRgbGreen();
        }
    }

    public void setRgbOrange() {
        rgb.setPosition(0.3);
    }

    public void setRgbGreen() {
        rgb.setPosition(0.5);
    }

    public void setPower(double p) {
        intakePower = p;
        intake.setPower(p);
    }

    public void gateUp() {
        gate.setPosition(gateUpPos);
    }

    public void gateDown() {
        gate.setPosition(gateDownPos);
    }

    public void leverDown() {
        if (RobotConstants.currentTeam == Alliance.RED) {
            leverLeft.setPosition(leverDownLeftPos);
            leverRight.setPosition(leverUpRightPos);
        } else {
            leverRight.setPosition(leverDownRightPos);
            leverLeft.setPosition(leverUpLeftPos);
        }
    }

    public void leverUp() {
        leverLeft.setPosition(leverUpLeftPos);
        leverRight.setPosition(leverUpRightPos);
    }

    public void startWiggle() {
        wiggleState = WiggleState.LEFT_DOWN;
        wiggleTimer.reset();
    }

    public void stopWiggle() {
        wiggleState = WiggleState.IDLE;
    }

    public void updateWiggle(boolean held) {
        if (!held) {
            wiggleState = WiggleState.IDLE;
            return;
        }

        if (wiggleState == WiggleState.IDLE) {
            wiggleState = WiggleState.LEFT_DOWN;
            wiggleTimer.reset();
        }

        switch (wiggleState) {
            case LEFT_DOWN:
                if (wiggleTimer.seconds() > wiggleMoveTime) {
                    leverLeft.setPosition(leverDownLeftPos);
                    leverRight.setPosition(leverUpRightPos);
                    wiggleTimer.reset();
                    wiggleState = WiggleState.RIGHT_DOWN;
                }
                break;

            case RIGHT_DOWN:
                if (wiggleTimer.seconds() > wiggleMoveTime) {
                    leverRight.setPosition(leverDownRightPos);
                    leverLeft.setPosition(leverUpLeftPos);
                    wiggleTimer.reset();
                    wiggleState = WiggleState.LEFT_DOWN;
                }
                break;

            case IDLE:
                break;
        }
    }

    public double getGatePos() {
        return gate.getPosition();
    }

    public double getIntakePower() {
        return intakePower;
    }

    public boolean isGateUp() {
        return Math.abs(gate.getPosition() - gateUpPos) < 0.03;
    }

    public boolean isLeverUp() {
        if (RobotConstants.currentTeam == Alliance.RED) {
            return Math.abs(leverLeft.getPosition() - leverUpLeftPos) < 0.03;
        } else {
            return Math.abs(leverRight.getPosition() - leverUpRightPos) < 0.03;
        }
    }

    public void telemetry() {
        telemetry.addData("Left Ball Sensor", leftBallDetected());
        telemetry.addData("Right Ball Sensor", rightBallDetected());
        telemetry.addData("Any Ball Detected", ballDetected());
        telemetry.addData("Intake Power", intakePower);
    }
}