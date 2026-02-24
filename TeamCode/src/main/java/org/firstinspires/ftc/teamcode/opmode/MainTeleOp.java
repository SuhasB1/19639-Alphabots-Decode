package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import static java.lang.Math.atan2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.fsm.IntakeFSM;
import org.firstinspires.ftc.teamcode.config.fsm.ShootFSM;

@Config
@TeleOp(name = "! MAIN TELEOP !")
public class MainTeleOp extends OpMode {

    public static boolean LUT_TESTING = false;
    public static double manualVelocity = 0.0;
    public static double xPos = 44.0;
    public static double yPos = 134.0;
    public static double headingOffsetDeg = 0.0;

    private Robot robot;

    private boolean prevLT = false;
    private boolean prevRB = false;
    private boolean prevLB = false;
    private boolean prevDpadDown = false;
    private boolean prevRT = false;

    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevDpadUp = false;

    private boolean prevShare = false;
    private boolean prevOptions = false;
    private boolean prevOptionsLUT = false;
    private boolean prevLB2 = false;
    private boolean prevRB2 = false;
    private boolean prevLStickBtn = false;
    private boolean intakeEnabledToggle = true;
    private boolean headingPathLockEnabled = true;
    private boolean wasShootingActive = false;

    private ShootFSM.State prevShootState = ShootFSM.State.IDLE;
    private final ElapsedTime loopTime = new ElapsedTime();

    private PathChain lutPath = null;
    private boolean lutDriving = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, RobotConstants.currentTeam);
        robot.init();
        follower.setStartingPose(RobotConstants.currentPose);
        follower.update();
        robot.drive.speed = 0.55;
        telemetry.setMsTransmissionInterval(50);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void start() {
        robot.start();
        loopTime.reset();

        prevLT = (gamepad1.left_trigger > 0.1);
        prevRB = gamepad1.right_bumper;
        prevLB = gamepad1.left_bumper;
        prevRT = (gamepad1.right_trigger > 0.1);

        prevDpadLeft = gamepad2.dpad_left;
        prevDpadRight = gamepad2.dpad_right;
        prevDpadUp = gamepad2.dpad_up;
        prevDpadDown = gamepad2.dpad_down;
        prevOptions = gamepad2.options;
        prevLB2 = gamepad2.left_bumper;
        prevRB2 = gamepad2.right_bumper;


        prevShare = gamepad1.share;
        prevOptionsLUT = gamepad1.options;

        prevLStickBtn = gamepad1.left_stick_button;

        prevShootState = robot.shootFSM.getState();
        wasShootingActive = false;
    }

    private double aimHeadingWithOffset(double goalX, double goalY) {
        double base = Math.atan2(goalY - robot.drive.getY(), goalX - robot.drive.getX());
        return base + Math.toRadians(headingOffsetDeg);
    }

    private void applyIntakeState(boolean lbHeld, boolean triangleHeld) {
        if (triangleHeld) {
            robot.intakeFSM.startOuttake();
            return;
        }
        if (lbHeld) {
            robot.intakeFSM.stop();
            return;
        }
        if (intakeEnabledToggle) robot.intakeFSM.startIntake();
        else robot.intakeFSM.stop();
    }

    @Override
    public void loop() {
        loopTime.reset();

        boolean dpadDown = gamepad2.cross;
        if (dpadDown && !prevDpadDown) {
            if (RobotConstants.currentTeam == Alliance.RED) {
                robot.drive.resetPose(5, 6.5, Math.toRadians(0));
            }else{
                robot.drive.resetPose(141.5 - 5, 6.5, Math.toRadians(180));

            }
        }
        prevDpadDown = dpadDown;

        boolean dpadLeft = gamepad2.square;
        if (dpadLeft && !prevDpadLeft) {
            if (RobotConstants.currentTeam == Alliance.RED) {
                robot.drive.resetPose(141.5 - 51.67, 6.67, Math.toRadians(90));
            }else{
                robot.drive.resetPose(51.67, 6.67, Math.toRadians(90));

            }
        }
        prevDpadLeft = dpadLeft;

        boolean dpadRight = gamepad2.circle;
        if (dpadRight && !prevDpadRight) {
            if (RobotConstants.currentTeam == Alliance.RED) {
                robot.drive.resetPose(141.5-28.5, 131, Math.toRadians(90));
            }else{
                robot.drive.resetPose(28.5, 131, Math.toRadians(90));

            }
        }
        prevDpadRight = dpadRight;

        boolean dpadUp = gamepad2.triangle;
        if (dpadUp && !prevDpadUp) {
            if (RobotConstants.currentTeam == Alliance.RED) {
                robot.drive.resetPose(141.5-8, 62, Math.toRadians(0));
            }else{
                robot.drive.resetPose(8, 62, Math.toRadians(180));

            }
        }
        prevDpadUp = dpadUp;

        boolean lb2 = gamepad2.left_bumper;
        if (lb2 && !prevLB2) {
            if (RobotConstants.currentTeam.equals(Alliance.BLUE)) headingOffsetDeg += 1.0;
            else headingOffsetDeg += 1.0;
        }
        prevLB2 = lb2;

        boolean rb2 = gamepad2.right_bumper;
        if (rb2 && !prevRB2) {
            if (RobotConstants.currentTeam.equals(Alliance.BLUE)) headingOffsetDeg -= 1.0;
            else headingOffsetDeg -= 1.0;
        }
        prevRB2 = rb2;

        boolean ltHeld = gamepad1.left_trigger > 0.1;
        if (ltHeld) robot.intake.leverDown();
        else robot.intake.leverUp();

        boolean triangleHeld = gamepad1.triangle;

        boolean lbHeld = gamepad1.left_bumper;

        boolean squarePressed = gamepad1.square;
        if (squarePressed && !prevLT) {
            intakeEnabledToggle = !intakeEnabledToggle;
        }
        prevLT = squarePressed;

        applyIntakeState(lbHeld, triangleHeld);

        boolean share = gamepad1.share;
        boolean options_lut = gamepad1.options;
        boolean options = gamepad2.options;

        if (!LUT_TESTING) {
            if (options && !prevOptions) headingPathLockEnabled = !headingPathLockEnabled;
        }

        if (LUT_TESTING) {
            if (share && !prevShare) {
                robot.shooter.setManualVelocityOverride(manualVelocity);
                robot.shootFSM.start();
            }

            if (options_lut && !prevOptionsLUT) {
                double startH = robot.drive.getHeading();
                double endH = Math.atan2(RobotConstants.goalY - yPos, RobotConstants.goalX - xPos) + Math.toRadians(headingOffsetDeg);

                lutPath = follower.pathBuilder()
                        .addPath(new BezierCurve(
                                new Pose(robot.drive.getX(), robot.drive.getY(), startH),
                                new Pose((robot.drive.getX() + xPos) * 0.5, (robot.drive.getY() + yPos) * 0.5),
                                new Pose(xPos, yPos, endH)
                        ))
                        .setLinearHeadingInterpolation(startH, endH)
                        .build();

                follower.setMaxPower(0.8);
                follower.followPath(lutPath, true);
                lutDriving = true;
            }
        }

        prevShare = share;
        prevOptions = options;
        prevOptionsLUT = options_lut;

        boolean rb = gamepad1.right_bumper;
        if (rb && !prevRB && !LUT_TESTING) {
            ShootFSM.State shootState = robot.shootFSM.getState();
            if (shootState == ShootFSM.State.IDLE) robot.shootFSM.start();
            else {
                robot.shootFSM.stop();
                robot.intakeFSM.stop();
            }
        }
        prevRB = rb;

        robot.update(gamepad1, gamepad2, robot.drive.getX(), robot.drive.getY());

        boolean lStickBtn = gamepad1.left_stick_button;
        if (LUT_TESTING && lStickBtn && !prevLStickBtn) {
            robot.drive.returnToTeleop();
            lutDriving = false;
        }
        prevLStickBtn = lStickBtn;

        ShootFSM.State shootNow = robot.shootFSM.getState();
        boolean shootingActive = (shootNow == ShootFSM.State.SPINUP || shootNow == ShootFSM.State.DONE);

        if (shootingActive && !wasShootingActive) {
            robot.drive.startHoldPosition(robot.drive.getPose());
        }
        if (!shootingActive && wasShootingActive) {
            robot.drive.stopHoldPosition();
        }
        wasShootingActive = shootingActive;

        if (!LUT_TESTING) {
            if (shootingActive) {
                if (headingPathLockEnabled) {
                    double target = aimHeadingWithOffset(RobotConstants.goalX, RobotConstants.goalY);
                    robot.drive.holdTranslationToPose(robot.drive.convertHeading(target));
                } else {
                    robot.drive.holdTranslationToPose(robot.drive.getHeading());
                }
            } else {
                if (prevShootState != ShootFSM.State.IDLE && shootNow == ShootFSM.State.IDLE) {
                    robot.drive.returnToTeleop();
                    robot.intakeFSM.state = IntakeFSM.State.IDLE;
                    robot.shooter.clearManualVelocityOverride();
                }

                robot.shooter.idleRev(robot.drive.getX(), robot.drive.getY());

                boolean rtHeld = gamepad1.right_trigger > 0.1;
                if (rtHeld && !prevRT) robot.drive.resetHeadingPID();
                prevRT = rtHeld;

                if (rtHeld) {
                    double target = aimHeadingWithOffset(RobotConstants.goalX, RobotConstants.goalY);
                    robot.drive.headingLockPIDToHeading(gamepad1, target);
                } else {
                    robot.drive.updateTeleop(gamepad1);
                }
            }
        } else {
            if (!lutDriving) robot.drive.updateTeleop(gamepad1);
            else follower.update();
        }

        prevShootState = shootNow;

        telemetry.addData("loop time (ms)", loopTime.milliseconds());
        telemetry.addData("Robot X", robot.drive.getX());
        telemetry.addData("Robot Y", robot.drive.getY());
        telemetry.addData("Robot Heading", Math.toDegrees(robot.drive.getHeading()));
        telemetry.addData("headingOffsetDeg", headingOffsetDeg);
        telemetry.addData("pathLockEnabled", headingPathLockEnabled);
        telemetry.addData("intakeEnabledToggle", intakeEnabledToggle);
        telemetry.addData("heading from goal", Math.toDegrees(atan2( RobotConstants.goalY - robot.drive.getY(), RobotConstants.goalX - robot.drive.getX())));
        telemetry.addData("Shooter Measured Vel", robot.shooter.getMeasuredVelocity());
        robot.telemetry();
        telemetry.update();
    }
}