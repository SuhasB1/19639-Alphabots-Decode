package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.config.shootAnywhere.ShooterCloseBaseModel;
import org.firstinspires.ftc.teamcode.config.shootAnywhere.ShooterCloseLUT;
import org.firstinspires.ftc.teamcode.config.shootAnywhere.ShooterFarLUT;
import org.firstinspires.ftc.teamcode.config.shootAnywhere.ShooterParams;

public class ShooterSubsystem {

    public enum TargetMode { BASE_MODEL, LUT }

    private DcMotorEx left, right;
    public Servo hood;

    private final Telemetry telemetry;
    private final ShooterCloseLUT closeLUT = new ShooterCloseLUT();

    private final ShooterFarLUT farLUT = new ShooterFarLUT();


    ShooterParams params;

    public double kP = 0.0125;
    public double kV = 0.0003215;
    public double kS = 0.09;

    private double targetVelocity = 0;
    private double measuredVelocity = 0;

    private TargetMode mode = TargetMode.LUT;

    public ShooterSubsystem(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        left  = hw.get(DcMotorEx.class, "shooterLeft");
        right = hw.get(DcMotorEx.class, "shooterRight");
        hood  = hw.get(Servo.class, "hood");

        hood.setDirection(Servo.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void init() {
        stop();
    }

    public void setMode(TargetMode m) { mode = m; }
    public TargetMode getMode() { return mode; }

    public double getMeasuredVelocity() { return measuredVelocity; }
    public double getTargetVelocity() { return targetVelocity; }

    public void setHood(double pos) { hood.setPosition(pos); }

    public void stop() {
        left.setPower(0);
        right.setPower(0);
        targetVelocity = 0;
    }

    public void setGoal(double x, double y) {
        RobotConstants.goalX = x;
        RobotConstants.goalY = y;
    }

    public void updateVelocityControl(double robotX, double robotY) {
        measuredVelocity = -left.getVelocity();

        if(robotY<50) {
            params = farLUT.getFromRobotPose(robotX, robotY);

        }else{
            params = closeLUT.getFromRobotPose(robotX, robotY);
        }
        targetVelocity = params.shooterVel;

        double power =
                kP * (targetVelocity - measuredVelocity)
                        + kV * measuredVelocity
                        + kS * Math.signum(targetVelocity);

        left.setPower(-power);
        right.setPower(-power);
    }

    public boolean atSpeed(double tolerance) {
        return Math.abs(targetVelocity - measuredVelocity) <= tolerance;
    }

    public void telemetry() {
        telemetry.addData("ShooterMode", mode);
        telemetry.addData("TargetVel", targetVelocity);
        telemetry.addData("MeasuredVel", measuredVelocity);
        telemetry.addData("RPM", measuredVelocity / 28.0 * 60.0);
    }
}
