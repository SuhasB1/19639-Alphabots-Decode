package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.worldsPaths.GateCycling;

@Autonomous(name = "!RED_21Ball", group = "Main")
public class RED_TwentyOneBall extends OpMode {

    private Robot robot;
    private GateCycling paths;

    private int pathState = 0;
    private int gateRepeatsDone = 0;

    private static final double T_DONE_SCORE = 0.9;
    private static final double T_DONE_INTAKE = 0.9;

    private static final double GATE_WAIT_SECONDS = 1.367;
    private static final double SHOOT_SETTLE_SECONDS = 0.0;
    private static final double GATE_RETURN_STOP_T = 0.67;

    private static final double GATE_Y_INCREMENT = 0.3;

    private final Timer gateWaitTimer = new Timer();
    private final Timer shootWaitTimer = new Timer();

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
        shootWaitTimer.resetTimer();

        robot.intakeFSM.startTransfer();
        follower.followPath(paths.scorePreload(), true);
    }

    @Override
    public void loop() {
        follower.update();
        robot.update(gamepad1, gamepad2, follower.getPose().getX(), follower.getPose().getY());
        robot.shooter.updateVelocityControl(follower.getPose().getX(), follower.getPose().getY());

        autonomousPathUpdate();

        telemetry.addData("pathState", pathState);
        telemetry.addData("gateRepeatsDone", gateRepeatsDone);
        telemetry.addData("gateYOffset", gateRepeatsDone * GATE_Y_INCREMENT);
        telemetry.addData("t", follower.getCurrentTValue());
        telemetry.addData("actionBusy", RobotConstants.actionBusy);
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
                    follower.followPath(paths.intake2ToScore(), true);
                    pathState = 4;
                }
                break;

            case 4:
                if (driveDone(T_DONE_SCORE)) {
                    leverUp();
                    shootWaitTimer.resetTimer();
                    pathState = 5;
                }
                break;

            case 5:
                if (shootWaitTimer.getElapsedTimeSeconds() > SHOOT_SETTLE_SECONDS) {
                    startShoot();
                    pathState = 6;
                }
                break;

            case 6:
                if (actionDone()) {
                    leverDown();
                    startIntake();
                    follower.followPath(paths.scoreToGateIntake(getGateYOffset()), true);
                    pathState = 7;
                }
                break;

            case 7:
                if (driveDone(T_DONE_INTAKE)) {
                    gateWaitTimer.resetTimer();
                    pathState = 8;
                }
                break;

            case 8:
                if (gateWaitTimer.getElapsedTimeSeconds() >= GATE_WAIT_SECONDS) {
                    follower.followPath(paths.gateIntakeToScore(getGateYOffset()), true);
                    pathState = 9;
                }
                break;

            case 9:
                if (follower.getCurrentTValue() > GATE_RETURN_STOP_T) {
                    stopIntake();
                    pathState = 10;
                }
                break;

            case 10:
                if (driveDone(T_DONE_SCORE)) {
                    leverUp();
                    shootWaitTimer.resetTimer();
                    pathState = 11;
                }
                break;

            case 11:
                if (shootWaitTimer.getElapsedTimeSeconds() > SHOOT_SETTLE_SECONDS) {
                    startShoot();
                    pathState = 12;
                }
                break;

            case 12:
                if (actionDone()) {
                    gateRepeatsDone++;

                    if (gateRepeatsDone < 4) {
                        leverDown();
                        startIntake();
                        follower.followPath(paths.scoreToGateIntake(getGateYOffset()), true);
                        pathState = 7;
                    } else {
                        pathState = 13;
                    }
                }
                break;

            case 13:
                if (actionDone()) {
                    follower.setMaxPower(0.8);
                    follower.followPath(paths.toIntake1(), true);
                    startIntake();
                    pathState = 14;
                }
                break;

            case 14:
                if (driveDone(T_DONE_INTAKE)) {
                    robot.intakeFSM.startTransfer();
                    follower.setMaxPower(1);
                    follower.followPath(paths.intake1ToParkScore(), true);
                    pathState = 15;
                }
                break;

            case 15:
                if (driveDone(T_DONE_SCORE)) {
                    leverUp();
                    shootWaitTimer.resetTimer();
                    pathState = 16;
                }
                break;

            case 16:
                if (shootWaitTimer.getElapsedTimeSeconds() > SHOOT_SETTLE_SECONDS) {
                    startShoot();
                    pathState = 17;
                }
                break;

            case 17:
                if (actionDone()) {
                    pathState = 18;
                }
                break;

            case 18:
                break;
        }
    }
}