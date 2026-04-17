package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem {


    public enum WiggleState {
        IDLE,
        LEFT_DOWN,
        LEFT_PAUSE,
        RIGHT_DOWN,
        RIGHT_PAUSE
    }

    private WiggleState wiggleState = WiggleState.IDLE;
    private final ElapsedTime wiggleTimer = new ElapsedTime();

    public double wigglePauseTime = 0.1;
    public double wiggleMoveTime = 0.25;

    private DcMotorEx intake;
    private Servo gate;
    public Servo leverLeft;
    public Servo leverRight;
    public Servo rgb;
    private RevColorSensorV3 sensor;
    private final Telemetry telemetry;

    // tune these
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
        sensor = hw.get(RevColorSensorV3.class, "color");
        leverLeft = hw.get(Servo.class, "leverLeft");
        leverRight = hw.get(Servo.class, "leverRight");
        leverRight.setDirection(Servo.Direction.REVERSE);
        leverLeft.setDirection(Servo.Direction.REVERSE);
        ((LynxI2cDeviceSynch) sensor.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
    }

    public void init() {
        gateUp();
        leverUp();
        setPower(0);
    }

    public double getDistance(){
        return sensor.getDistance(DistanceUnit.MM);
    }

    public void setPower(double p) {
        intakePower = p;
        intake.setPower(p);
    }

    public void gateUp()   {
        gate.setPosition(gateUpPos);
    }
    public void gateDown() {
        gate.setPosition(gateDownPos);
    }

    public void leverDown(){
        if(RobotConstants.currentTeam == Alliance.RED){
            leverLeft.setPosition(leverDownLeftPos);
            leverRight.setPosition(leverUpRightPos);
        }else{
            leverRight.setPosition(leverDownRightPos);
            leverLeft.setPosition(leverUpLeftPos);
        }
    }

    public void leverUp(){
        if(RobotConstants.currentTeam == Alliance.RED){
            leverLeft.setPosition(leverUpLeftPos);
            leverRight.setPosition(leverUpRightPos);
        }else{
            leverRight.setPosition(leverUpRightPos);
            leverLeft.setPosition(leverUpLeftPos);
        }
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
            default:
                break;
        }
    }

    public double getGatePos(){ return gate.getPosition();}

    public double getIntakePower() { return intakePower; }

    public boolean isGateUp() {
        return Math.abs(gate.getPosition() - gateUpPos) < 0.03;
    }
    public boolean isLeverUp() {
        if(RobotConstants.currentTeam == Alliance.RED){
            return Math.abs(leverLeft.getPosition() - leverUpLeftPos) < 0.03;
        }else{
            return Math.abs(leverRight.getPosition() - leverUpRightPos) < 0.03;
        }
    }

    public void telemetry() {
        telemetry.addData("Distance", sensor.getDistance(DistanceUnit.MM));
    }
}
