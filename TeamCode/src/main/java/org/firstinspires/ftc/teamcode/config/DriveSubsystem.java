package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static java.lang.Math.atan2;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.control.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
public class DriveSubsystem {

    private final Telemetry telemetry;

    public double speed = 1;

    public static double turnSpeed = 1;

    public static double headingKp = 0.75;
    public double headingKi = 0.0;
    public static double headingKd = 0.06;
    public double headingKf = 0.05;

    private double power;

    public static double headingDeadbandDeg = 2.5;
    private final ElapsedTime headingTimer = new ElapsedTime();
    PIDFController drivePID;

    private Pose holdPose = null;
    private boolean holdEnabled = false;

    // If error <= this, we do NOT lock translation OR heading (your request)
    public static double holdHeadingDeadbandDeg = 3;

    public DriveSubsystem(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        follower = Constants.createFollower(hw);
    }

    public void init() {
        drivePID = new PIDFController(new PIDFCoefficients(headingKp, headingKi, headingKd, headingKf));
    }

    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    public Pose getPose() { return follower.getPose(); }
    public double getX() { return getPose().getX(); }
    public double getY() { return getPose().getY(); }
    public double getHeading() { return getPose().getHeading(); }

    public void updateTeleop(Gamepad gp1) {
        follower.setTeleOpDrive(
                -gp1.left_stick_y * speed,
                -gp1.left_stick_x * speed,
                -gp1.right_stick_x * speed,
                true
        );
        follower.update();
    }

    public void startHoldPosition(Pose poseToHold) {
        holdPose = poseToHold;
        holdEnabled = true;
    }

    public void stopHoldPosition() {
        holdEnabled = false;
        holdPose = null;
    }

    public boolean isHoldEnabled() {
        return holdEnabled && holdPose != null;
    }

    public Pose getHoldPose() { return holdPose; }

    /**
     * Holds translation ONLY when heading error exceeds holdHeadingDeadbandDeg.
     * If heading error is within the band, does nothing (no translation lock, no heading lock).
     */
    public void holdTranslationToPose(double desiredHeadingRad) {
        if (!isHoldEnabled()) return;

        // Keep follower state fresh
        follower.update();

        double errRad = angleWrap(desiredHeadingRad - getHeading());
        double errDeg = Math.abs(Math.toDegrees(errRad));

        // Gate the entire hold behavior on heading error
        if (errDeg <= holdHeadingDeadbandDeg) {
            return; // <-- no lock at all inside band
        }

        PathChain chain = follower.pathBuilder()
                .addPath(new Path(new BezierLine(
                        follower::getPose,
                        () -> new Pose(holdPose.getX(), holdPose.getY(), desiredHeadingRad)
                )))
                .setConstantHeadingInterpolation(desiredHeadingRad)
                .build();
        follower.setMaxPower(0.8);
        follower.followPath(chain);
        follower.update();
    }

    public double convertHeading(double target) {
        if (getY() < 50) {
            if (RobotConstants.currentTeam.equals(Alliance.BLUE) && getX() < 67) {
                target = target + Math.toRadians(1); //1.6
            } else if (RobotConstants.currentTeam.equals(Alliance.BLUE) && getX() > 67) {
                target = target + Math.toRadians(1); //1
            } else if (RobotConstants.currentTeam.equals(Alliance.RED) && getX() > (141.5 - 67)) {
                target = target - Math.toRadians(1); // 1.6
            } else {
                target = target + Math.toRadians(1); // 1
            }
        } else if (getY() > 110) {
            if (RobotConstants.currentTeam.equals(Alliance.BLUE) && getX() < 72) {
                target = target + Math.toRadians(1.67);
            } else if(RobotConstants.currentTeam.equals(Alliance.BLUE) && getX() > 72){
                target = target + Math.toRadians(3.5);
            } else if (RobotConstants.currentTeam.equals(Alliance.RED) && getX() > 72) {
                target = target - Math.toRadians(1.67);
            } else{
                target = target - Math.toRadians(3.5);
            }
        } else {
            if (RobotConstants.currentTeam.equals(Alliance.BLUE) && getX() < 80) {
                target = target + Math.toRadians(1.67);
            } else if (RobotConstants.currentTeam.equals(Alliance.BLUE) && getX() > 80) {
                target = target + Math.toRadians(2.67);
            } else if (RobotConstants.currentTeam.equals(Alliance.RED) && getX() > 60) {
                target = target - Math.toRadians(1.67);
            } else {
                target = target - Math.toRadians(2.67);
            }
        }
        return target;
    }

    public void returnToTeleop() {
        follower.startTeleopDrive();
        follower.setMaxPower(1);
        follower.update();
    }

    public void resetHeadingPID() {
        headingTimer.reset();
    }

    public void headingLockPID(Gamepad gp1, double goalX, double goalY) {
        double vx = -gp1.left_stick_y * speed;
        double vy = -gp1.left_stick_x * speed;

        double target = atan2(goalY - getY(), goalX - getX());
        target = convertHeading(target);

        headingLockPIDToHeadingInternal(vx, vy, target);
    }

    public void headingLockPIDToHeading(Gamepad gp1, double targetHeading) {
        double vx = -gp1.left_stick_y * speed;
        double vy = -gp1.left_stick_x * speed;
        targetHeading = convertHeading(targetHeading);
        headingLockPIDToHeadingInternal(vx, vy, targetHeading);
    }

    private void headingLockPIDToHeadingInternal(double vx, double vy, double targetHeading) {
        double err = angleWrap(targetHeading - getHeading());
        double errDeg = Math.abs(Math.toDegrees(err));

        drivePID.setCoefficients(new PIDFCoefficients(headingKp, headingKi, headingKd, headingKf));

        if (errDeg < headingDeadbandDeg) {
            power = 0.0;
            drivePID.updateError(0.0);
        } else {
            drivePID.updateError(err);
            power = drivePID.run();
            power = clamp(power, -turnSpeed, turnSpeed);
            power += Math.signum(err) * 0.03;
            power = clamp(power, -turnSpeed, turnSpeed);
        }

        follower.setTeleOpDrive(vx, vy, power, true);
        follower.update();
    }

    private static double angleWrap(double a) {
        while (a <= -Math.PI) a += 2.0 * Math.PI;
        while (a >  Math.PI)  a -= 2.0 * Math.PI;
        return a;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public void resetPose(double x, double y, double headingRad) {
        follower.setPose(new Pose(x, y, headingRad));
        follower.startTeleopDrive();
        follower.update();
    }
}