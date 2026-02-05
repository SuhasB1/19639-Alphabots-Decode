package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.fsm.IntakeFSM;
import org.firstinspires.ftc.teamcode.config.fsm.ShootFSM;

@Config
@TeleOp(name = "LIMELIGHT MAIN TELEOP", group = "Main")
public class LL_MainTeleOp extends OpMode {

    public static boolean LUT_TESTING = false;
    public static double manualVelocity = 0;
    public static double xPos = 44.0;
    public static double yPos = 134.0;

    public static double headingOffsetDeg = 0.0;

    public static boolean LIMELIGHT_ENABLED = true;
    public static double LIMELIGHT_POLL_HZ = 60;
    public static double LIMELIGHT_MAX_STALE_MS = 120;
    public static boolean LIMELIGHT_ALLOW_RELOCALIZE = true;

    private Limelight3A limelight;

    private Robot robot;

    private boolean prevLT = false;
    private boolean prevRB = false;
    private boolean prevDpadDown = false;
    private boolean prevRT = false;

    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevDpadUp = false;

    private boolean prevShare = false;
    private boolean prevOptions = false;

    private boolean prevShareReloc = false;

    private ShootFSM.State prevShootState = ShootFSM.State.IDLE;
    private final ElapsedTime loopTime = new ElapsedTime();

    private PathChain lutPath = null;
    private boolean lutDriving = false;

    private boolean headingPathLockEnabled = false;

    private Pose lastLLPedroPose = null;
    private double lastLLStaleMs = 0;
    private int lastLLTagCount = 0;
    private boolean lastLLValid = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, RobotConstants.currentTeam);
        robot.init();

        follower.setStartingPose(RobotConstants.currentPose);
        follower.update();

        robot.drive.speed = 0.55;

        if (LIMELIGHT_ENABLED) {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz((int) LIMELIGHT_POLL_HZ);
            limelight.start();
        }
    }

    @Override
    public void start() {
        robot.start();
        loopTime.reset();

        prevLT = (gamepad1.left_trigger > 0.1);
        prevRB = gamepad1.right_bumper;
        prevDpadDown = gamepad1.dpad_down;
        prevRT = (gamepad1.right_trigger > 0.1);

        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
        prevDpadUp = gamepad1.dpad_up;

        prevShare = gamepad1.share;
        prevOptions = gamepad1.options;

        prevShareReloc = gamepad1.share;

        prevShootState = robot.shootFSM.getState();
    }

    private double aimHeadingWithOffset(double goalX, double goalY) {
        double base = Math.atan2(goalY - robot.drive.getY(), goalX - robot.drive.getX());
        return base + Math.toRadians(headingOffsetDeg);
    }

    private void updateLimelight() {
        lastLLPedroPose = null;
        lastLLValid = false;
        lastLLStaleMs = 0;
        lastLLTagCount = 0;

        if (!LIMELIGHT_ENABLED || limelight == null) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        lastLLStaleMs = result.getStaleness();
        lastLLTagCount = (result.getFiducialResults() == null) ? 0 : result.getFiducialResults().size();
        if (lastLLStaleMs > LIMELIGHT_MAX_STALE_MS) return;

        Pose3D botPose = result.getBotpose();
        if (botPose == null) return;

        Pose2D ftcPose = new Pose2D(
                DistanceUnit.METER,
                botPose.getPosition().x,
                botPose.getPosition().y,
                AngleUnit.DEGREES,
                botPose.getOrientation().getYaw()
        );

        Pose pedroLLPose = PoseConverter
                .pose2DToPose(ftcPose, InvertedFTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);

        lastLLPedroPose = pedroLLPose;
        lastLLValid = true;
    }

    private void relocalizeFromLimelightIfRequested() {
        if (!LIMELIGHT_ALLOW_RELOCALIZE) return;

        boolean shareBtn = gamepad1.share;

        if (!LUT_TESTING) {
            if (shareBtn && !prevShareReloc) {
                if (lastLLValid && lastLLPedroPose != null) {
                    follower.setPose(lastLLPedroPose);
                    follower.update();
                }
            }
        }

        prevShareReloc = shareBtn;
    }

    @Override
    public void loop() {
        loopTime.reset();

        robot.drive.speed = (gamepad1.left_bumper) ? 0.3 : 0.55;

        boolean dpadDown = gamepad1.dpad_down;
        if (dpadDown && !prevDpadDown) {
            if (RobotConstants.currentTeam == Alliance.RED) {
                robot.drive.resetPose(6.6, 8.1, Math.toRadians(0));
            } else {
                robot.drive.resetPose(144 - 6.6, 8.1, Math.toRadians(180));
            }
        }
        prevDpadDown = dpadDown;

        boolean dpadLeft = gamepad1.dpad_left;
        if (dpadLeft && !prevDpadLeft) {
            if (RobotConstants.currentTeam.equals(Alliance.BLUE)) headingOffsetDeg += 1.0;
            else headingOffsetDeg -= 1.0;
        }
        prevDpadLeft = dpadLeft;

        boolean dpadUp = gamepad1.dpad_up;
        if (dpadUp && !prevDpadUp) headingOffsetDeg = 0;
        prevDpadUp = dpadUp;

        boolean dpadRight = gamepad1.dpad_right;
        if (dpadRight && !prevDpadRight) {
            if (RobotConstants.currentTeam.equals(Alliance.BLUE)) headingOffsetDeg -= 1.0;
            else headingOffsetDeg += 1.0;
        }
        prevDpadRight = dpadRight;

        boolean square = gamepad1.square;
        if (square) {
            IntakeFSM.State s = robot.intakeFSM.getState();
            if (s == IntakeFSM.State.IDLE) robot.intakeFSM.startOuttake();
            else robot.intakeFSM.stop();
        }

        boolean lt = gamepad1.left_trigger > 0.1;
        if (lt && !prevLT) {
            IntakeFSM.State s = robot.intakeFSM.getState();
            if (s == IntakeFSM.State.IDLE) robot.intakeFSM.startIntake();
            else robot.intakeFSM.stop();
        }
        prevLT = lt;

        if (gamepad1.triangle) robot.intakeFSM.startTransfer();

        boolean share = gamepad1.share;
        boolean options = gamepad1.options;

        if (!LUT_TESTING) {
            if (options && !prevOptions) {
                headingPathLockEnabled = !headingPathLockEnabled;
            }
        }

        if (LUT_TESTING) {
            if (share && !prevShare) {
                robot.shooter.setManualVelocityOverride(manualVelocity);
                robot.shootFSM.start();
            }

            if (options && !prevOptions) {
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

        follower.update();
        robot.update(gamepad1, gamepad2, robot.drive.getX(), robot.drive.getY());

        updateLimelight();
        relocalizeFromLimelightIfRequested();

        ShootFSM.State shootNow = robot.shootFSM.getState();
        boolean shootingActive = (shootNow == ShootFSM.State.SPINUP || shootNow == ShootFSM.State.DONE);

        if (!LUT_TESTING) {
            if (shootingActive) {
                if (headingPathLockEnabled) {
                    double target = aimHeadingWithOffset(RobotConstants.goalX, RobotConstants.goalY);
                    robot.drive.headingLockPathModeToHeading(target);
                }
            } else {
                if (prevShootState != ShootFSM.State.IDLE && shootNow == ShootFSM.State.IDLE) {
                    robot.drive.returnToTeleop();
                    robot.intakeFSM.state = IntakeFSM.State.IDLE;
                    robot.shooter.clearManualVelocityOverride();
                }

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
        }

        prevShootState = shootNow;

        telemetry.addData("loop time (ms)", loopTime.milliseconds());
        telemetry.addData("headingOffsetDeg", headingOffsetDeg);
        telemetry.addData("headingPathLockEnabled", headingPathLockEnabled);

        telemetry.addData("LL valid", lastLLValid);
        telemetry.addData("LL staleness (ms)", lastLLStaleMs);
        telemetry.addData("LL tagCount", lastLLTagCount);
        if (lastLLValid && lastLLPedroPose != null) {
            telemetry.addData("LL X", lastLLPedroPose.getX());
            telemetry.addData("LL Y", lastLLPedroPose.getY());
            telemetry.addData("LL H(deg)", Math.toDegrees(lastLLPedroPose.getHeading()));
        }

        telemetry.addData("LL Relocalize", "Share (only when LUT_TESTING=false)");
        robot.telemetry();
        telemetry.update();
    }
}