package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeSubsystem {

    private DcMotorEx intake;
    private Servo gate;
    public Servo rgb;
    private RevColorSensorV3 sensor;
    private final Telemetry telemetry;

    // tune these
    public double gateUpPos = 0.01;
    public double gateDownPos = 0.2;
    private double intakePower = 0.0;

    public IntakeSubsystem(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        intake = hw.get(DcMotorEx.class, "intake");
        rgb = hw.get(Servo.class, "rgb");
        gate = hw.get(Servo.class, "gate");
        sensor = hw.get(RevColorSensorV3.class, "color");
        ((LynxI2cDeviceSynch) sensor.getDeviceClient()).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
    }

    public void init() {
        gateUp();
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

    public double getGatePos(){ return gate.getPosition();}

    public double getIntakePower() { return intakePower; }

    public boolean isGateUp() {
        return Math.abs(gate.getPosition() - gateUpPos) < 0.03;
    }
    public void telemetry() {
        telemetry.addData("Distance", sensor.getDistance(DistanceUnit.MM));
    }
}
