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

    public static double headingKp = 0.5;
    public double headingKi = 0.0;
    public static double headingKd = 0.04;
    public double headingKf = 0.05;

    private double power;

    public static double headingDeadbandDeg = 0.4;
    private final ElapsedTime headingTimer = new ElapsedTime();
    PIDFController drivePID;

    private Pose holdPose = null;
    private boolean holdEnabled = false;
    private final ElapsedTime holdBuildTimer = new ElapsedTime();
    private static final double HOLD_REBUILD_PERIOD_S = 0.05;

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
        holdBuildTimer.reset();
    }

    public void stopHoldPosition() {
        holdEnabled = false;
        holdPose = null;
    }

    public boolean isHoldEnabled() {
        return holdEnabled && holdPose != null;
    }

    public void holdTranslationToPose(double headingRad) {
        if (!isHoldEnabled()) return;

        if (holdBuildTimer.seconds() < HOLD_REBUILD_PERIOD_S) {
            follower.update();
            return;
        }
        holdBuildTimer.reset();

        PathChain chain = follower.pathBuilder()
                .addPath(new Path(new BezierLine(
                        follower::getPose,
                        () -> new Pose(holdPose.getX(), holdPose.getY(), headingRad)
                )))
                .setConstantHeadingInterpolation(headingRad)
                .build();

        follower.followPath(chain);
        follower.update();
    }

    public double convertHeading(double target){
        if (getY() < 50) {
            if (RobotConstants.currentTeam.equals(Alliance.BLUE) && getX() < 72) {
                target = target + Math.toRadians(1.6);
            } else if (RobotConstants.currentTeam.equals(Alliance.BLUE) && getX() > 72) {
                target = target + Math.toRadians(4);
            } else if (RobotConstants.currentTeam.equals(Alliance.RED) && getX() > 72) {
                target = target - Math.toRadians(1.6);
            } else {
                target = target - Math.toRadians(4);
            }
        }
        else if (getY() > 125) {
            if (RobotConstants.currentTeam.equals(Alliance.BLUE)) {
                target = target + Math.toRadians(4);
            } else if (RobotConstants.currentTeam.equals(Alliance.RED)) {
                target = target - Math.toRadians(4);
            }
        }
        else{
            if (RobotConstants.currentTeam.equals(Alliance.BLUE) && getX() < 72) {
                target = target - Math.toRadians(1);
            } else if (RobotConstants.currentTeam.equals(Alliance.BLUE) && getX() > 72) {
                target = target + Math.toRadians(2);
            } else if (RobotConstants.currentTeam.equals(Alliance.RED) && getX() > 72) {
                target = target - Math.toRadians(1);
            } else {
                target = target + Math.toRadians(2);
            }
        }
        return target;
    }

    public void returnToTeleop() {
        follower.startTeleopDrive();
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