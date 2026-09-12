package org.firstinspires.ftc.teamcode.opmode.oldRegionals;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.paths.GateCycle1st;
@Disabled
@Autonomous(name = "FIRST INTAKE BLUE_GateCyclex3", group = "Main")
public class FIRSTIntakeGateCyclex3Auto_BLUE extends OpMode {

    private Robot robot;
    private GateCycle1st paths;

    private int pathState = 0;
    private int gateRepeatsDone = 0;

    private static final double T_DONE_SCORE = 0.95;
    private static final double T_DONE_INTAKE = 0.95;
    private static final double GATE_WAIT_SECONDS = 1.35;
    private static final double SHOOT_SETTLE = 0.05;

    private final Timer gateWaitTimer = new Timer();
    private final Timer shootWaitTimer = new Timer();

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, Alliance.BLUE);
        paths = new GateCycle1st(robot, Alliance.BLUE);

        RobotConstants.currentPose = paths.start;

        robot.init();

        follower.setStartingPose(paths.start);
        follower.update();
        follower.setMaxPower(1);

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
        follower.followPath(paths.scorePreload());
    }

    @Override
    public void loop() {
        follower.update();
        robot.update(gamepad1, gamepad2, follower.getPose().getX(), follower.getPose().getY());
        robot.shooter.idleRev(follower.getPose().getX(), follower.getPose().getY());
        autonomousPathUpdate();

        telemetry.addData("pathState", pathState);
        telemetry.addData("gateRepeatsDone", gateRepeatsDone);
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

    private void startShoot() {
        robot.intakeFSM.startTransfer();
        robot.shootFSM.start();
    }

    private void startIntake() {
        robot.intakeFSM.startIntake();
    }

    private void leverDown() {
        robot.intake.leverDown();
    }

    private void leverUp() {
        robot.intake.leverUp();
    }

    private void stopIntake() {
        robot.intakeFSM.stop();
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
                if (shootWaitTimer.getElapsedTimeSeconds() > 0.05) {
                    startShoot();
                    pathState = 2;
                }
                break;

            // INTAKE 2
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
                    shootWaitTimer.resetTimer();
                    pathState = 5;
                }
                break;

            case 5:
                if (shootWaitTimer.getElapsedTimeSeconds() > SHOOT_SETTLE) {
                    leverUp();
                    startShoot();
                    pathState = 6;
                }
                break;

            // GATE CYCLES
            case 6:
                if (actionDone()) {
                    stopIntake();
                    leverDown();
                    startIntake();
                    follower.followPath(paths.scoreToGateIntake(), true);
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
                    follower.followPath(paths.gateIntakeToScore(), true);
                    pathState = 9;
                }
                break;

            case 9:
                if (driveDone(T_DONE_SCORE)) {
                    shootWaitTimer.resetTimer();
                    pathState = 10;
                }
                break;

            case 10:
                if (shootWaitTimer.getElapsedTimeSeconds() > SHOOT_SETTLE) {
                    leverUp();
                    startShoot();
                    pathState = 11;
                }
                break;

            case 11:
                if (actionDone()) {
                    gateRepeatsDone++;
                    if (gateRepeatsDone < 3) {
                        leverDown();
                        startIntake();
                        follower.followPath(paths.scoreToGateIntake(), true);
                        pathState = 7;
                    } else {
                        pathState = 12;
                    }
                }
                break;

            // INTAKE 1
            case 12:
                if (actionDone()) {
                    follower.followPath(paths.toIntake1(), true);
                    startIntake();
                    pathState = 13;
                }
                break;

            case 13:
                if (driveDone(T_DONE_INTAKE)) {
                    follower.followPath(paths.intake1ToScore(), true);
                    pathState = 14;
                }
                break;

            case 14:
                if (driveDone(T_DONE_SCORE)) {
                    shootWaitTimer.resetTimer();
                    pathState = 15;
                }
                break;

            case 15:
                if (shootWaitTimer.getElapsedTimeSeconds() > SHOOT_SETTLE) {
                    leverUp();
                    startShoot();
                    pathState = 16;
                }
                break;

            case 16:
                break;
        }
    }
}