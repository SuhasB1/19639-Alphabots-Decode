package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.worldsPaths.GateCycling;

@Autonomous(name = "RED_21BallPARTNER", group = "Main")
public class RED_TwentyOneBallPARTNER extends OpMode {

    private Robot robot;
    private GateCycling paths;

    private int pathState = 0;
    private int gateRepeatsDone = 0;

    private static final double T_DONE_SCORE = 0.9;
    private static final double T_DONE_INTAKE = 0.9;

    // ===== GATE TIMING =====
    public static double[] GATE_WAIT_PER_CYCLE = {0.9, 0.95, 1, 0.9};
    public static double GATE_HOLD_TIME = 0.15;

    // ===== SECOND SET GATE HIT WAIT =====
    public static double SECOND_SET_GATE_HIT_WAIT = 0.25;

    private static final double SHOOT_SETTLE_SECONDS = 0.0;
    private static final double GATE_RETURN_STOP_T = 0.67;
    private static final double GATE_Y_INCREMENT = 0;

    private final Timer gateWaitTimer = new Timer();
    private final Timer gateHoldTimer = new Timer();
    private final Timer shootWaitTimer = new Timer();
    private final Timer secondSetGateHitTimer = new Timer();

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, Alliance.RED);
        paths = new GateCycling(robot, Alliance.RED);

        RobotConstants.currentPose = paths.start;

        robot.init();

        follower.setStartingPose(paths.start);
        follower.update();
        follower.setMaxPower(1.0);

        pathState = 0;
        gateRepeatsDone = 0;
    }

    @Override
    public void start() {
        pathState = 0;
        gateRepeatsDone = 0;

        gateWaitTimer.resetTimer();
        gateHoldTimer.resetTimer();
        shootWaitTimer.resetTimer();
        secondSetGateHitTimer.resetTimer();

        robot.intakeFSM.startTransfer();
        follower.followPath(paths.scorePreload(), true);
    }

    @Override
    public void loop() {
        follower.update();
        robot.update(gamepad1, gamepad2,
                follower.getPose().getX(),
                follower.getPose().getY());

        robot.shooter.updateVelocityControl(
                follower.getPose().getX(),
                follower.getPose().getY());

        autonomousPathUpdate();

        telemetry.addData("state", pathState);
        telemetry.addData("cycle", gateRepeatsDone);
        telemetry.addData("gateYOffset", getGateYOffset());
        telemetry.addData("t", follower.getCurrentTValue());
        telemetry.update();
    }

    @Override
    public void stop() {
        RobotConstants.currentPose = follower.getPose();
        follower.setMaxPower(1.0);
    }

    private boolean driveDone(double thresh) {
        return follower.getCurrentTValue() > thresh && !follower.isBusy();
    }

    private boolean actionDone() {
        return !RobotConstants.actionBusy;
    }

    private double getGateYOffset() {
        return gateRepeatsDone * GATE_Y_INCREMENT;
    }

    private double getGateWait() {
        return GATE_WAIT_PER_CYCLE[
                Math.min(gateRepeatsDone, GATE_WAIT_PER_CYCLE.length - 1)
                ];
    }

    private void startShoot() {
        robot.intakeFSM.startTransfer();
        robot.shootFSM.start();
    }

    private void startIntake() {
        robot.intakeFSM.startIntake();
    }

    private void stopIntake() {
        robot.intakeFSM.stop();
    }

    private void leverDown() {
        robot.intake.leverDown();
    }

    private void leverUp() {
        robot.intake.leverUp();
    }

    private void autonomousPathUpdate() {

        switch (pathState) {

            // PRELOAD
            case 0:
                if (driveDone(T_DONE_SCORE)) {
                    leverUp();
                    shootWaitTimer.resetTimer();
                    pathState = 1;
                }
                break;

            case 1:
                if (shootWaitTimer.getElapsedTimeSeconds() > SHOOT_SETTLE_SECONDS) {
                    startShoot();
                    pathState = 2;
                }
                break;

            // SECOND SET
            case 2:
                if (actionDone()) {
                    follower.followPath(paths.toIntake2(), true);
                    startIntake();
                    pathState = 3;
                }
                break;

            case 3:
                if (driveDone(T_DONE_INTAKE)) {
                    robot.intakeFSM.startTransfer();
                    follower.followPath(paths.intake2ToGateHit(), true);
                    pathState = 4;
                }
                break;

            // ===== NEW: WAIT AT GATE HIT =====
            case 4:
                if (driveDone(T_DONE_INTAKE)) {
                    secondSetGateHitTimer.resetTimer();
                    pathState = 5;
                }
                break;

            case 5:
                if (secondSetGateHitTimer.getElapsedTimeSeconds() >= SECOND_SET_GATE_HIT_WAIT) {
                    follower.followPath(paths.gateHitToScore(), true);
                    pathState = 6;
                }
                break;

            case 6:
                if (driveDone(T_DONE_SCORE)) {
                    leverUp();
                    shootWaitTimer.resetTimer();
                    pathState = 7;
                }
                break;

            case 7:
                if (shootWaitTimer.getElapsedTimeSeconds() > SHOOT_SETTLE_SECONDS) {
                    startShoot();
                    pathState = 8;
                }
                break;

            // GATE CYCLES
            case 8:
                if (actionDone()) {
                    leverDown();
                    startIntake();
                    follower.followPath(paths.scoreToGateIntake(getGateYOffset()), true);
                    pathState = 9;
                }
                break;

            case 9:
                if (driveDone(T_DONE_INTAKE)) {
                    gateWaitTimer.resetTimer();
                    pathState = 10;
                }
                break;

            // ===== WAIT AT GATE =====
            case 10:
                if (gateWaitTimer.getElapsedTimeSeconds() >= getGateWait()) {
                    gateHoldTimer.resetTimer();
                    pathState = 11;
                }
                break;

            // ===== HOLD POSITION =====
            case 11:
                if (gateHoldTimer.getElapsedTimeSeconds() >= GATE_HOLD_TIME) {
                    follower.followPath(paths.gateIntakeToScore(getGateYOffset()), true);
                    pathState = 12;
                }
                break;

            // ===== STOP INTAKE MID-RETURN =====
            case 12:
                if (follower.getCurrentTValue() > GATE_RETURN_STOP_T) {
                    stopIntake();
                    pathState = 13;
                }
                break;

            case 13:
                if (driveDone(T_DONE_SCORE)) {
                    leverUp();
                    shootWaitTimer.resetTimer();
                    pathState = 14;
                }
                break;

            case 14:
                if (shootWaitTimer.getElapsedTimeSeconds() > SHOOT_SETTLE_SECONDS) {
                    startShoot();
                    pathState = 15;
                }
                break;

            case 15:
                if (actionDone()) {
                    gateRepeatsDone++;

                    if (gateRepeatsDone < 4) {
                        leverDown();
                        startIntake();
                        follower.followPath(paths.scoreToGateIntake(getGateYOffset()), true);
                        pathState = 9;
                    } else {
                        pathState = 16;
                    }
                }
                break;

            // INTAKE 1 FINAL
            case 16:
                if (actionDone()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(paths.toIntake1(), true);
                    startIntake();
                    pathState = 17;
                }
                break;

            case 17:
                if (driveDone(T_DONE_INTAKE)) {
                    robot.intakeFSM.startTransfer();
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.intake1ToParkScore(), true);
                    pathState = 18;
                }
                break;

            case 18:
                if (driveDone(T_DONE_SCORE)) {
                    leverUp();
                    shootWaitTimer.resetTimer();
                    pathState = 19;
                }
                break;

            case 19:
                if (shootWaitTimer.getElapsedTimeSeconds() > SHOOT_SETTLE_SECONDS) {
                    startShoot();
                    pathState = 20;
                }
                break;

            case 20:
                break;
        }
    }
}