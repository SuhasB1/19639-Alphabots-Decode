package org.firstinspires.ftc.teamcode.opmode.oldRegionals;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.paths.GateCycle;
@Disabled
@Autonomous(name = "RED_GateCyclex2 Third Auto", group = "Main")
public class GateCyclex2ThirdAuto_RED extends OpMode {

    private Robot robot;
    private GateCycle paths;

    private int pathState = 0;
    private int gateRepeatsDone = 0;

    private static final double T_DONE_SCORE = 0.95;
    private static final double T_DONE_INTAKE = 0.9;

    private static final double GATE_WAIT_SECONDS = 1.3;

    private final Timer pathTimer = new Timer();
    private final Timer gateWaitTimer = new Timer();

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, Alliance.RED);
        paths = new GateCycle(robot, Alliance.RED);

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
        pathTimer.resetTimer();
        gateWaitTimer.resetTimer();
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

    private void startShoot() {
        robot.intakeFSM.startTransfer();
        robot.shootFSM.start();
    }

    private void startIntake() {
        robot.intakeFSM.startIntake();
    }

    private void leverDown(){
        robot.intake.leverDown();
    }

    private void leverUp(){
        robot.intake.leverUp();
    }

    private void stopIntake() {
        robot.intakeFSM.stop();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                if (driveDone(T_DONE_SCORE)) {
                    leverUp();
                    startShoot();
                    pathState = 1;
                }
                break;

            case 1:
                if (actionDone()) {
                    follower.followPath(paths.toIntake1(), true);
                    startIntake();
                    pathState = 2;
                }
                break;

            case 2:
                if (driveDone(T_DONE_INTAKE)) {
                    follower.followPath(paths.intake1ToScore(), true);
                    pathState = 3;
                }
                break;

            case 3:
                if (driveDone(T_DONE_SCORE)) {
                    startShoot();
                    pathState = 4;
                }
                break;

            case 4:
                if (actionDone()) {
                    robot.intakeFSM.startTransfer();
                    follower.followPath(paths.toIntake2(), true);
                    startIntake();
                    pathState = 5;
                }
                break;

            case 5:
                if (driveDone(T_DONE_INTAKE)) {
                    robot.intakeFSM.startTransfer();
                    follower.followPath(paths.intake2ToGate(), true);
                    pathState = 6;
                }
                break;

            case 6:
                if (driveDone(T_DONE_SCORE)) {
                    pathTimer.resetTimer();
                    pathState = 7;
                }
                break;

            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(paths.gateToScore(), true);
                    pathState = 8;
                }
                break;

            case 8:
                if (driveDone(T_DONE_SCORE)) {
                    startShoot();
                    pathState = 9;
                }
                break;

            case 9:
                if (actionDone()) {
                    stopIntake();
                    follower.followPath(paths.scoreToGateIntake(), true);
                    leverDown();
                    startIntake();
                    pathState = 10;
                }
                break;

            case 10:
                if (driveDone(T_DONE_INTAKE)) {
                    gateWaitTimer.resetTimer();
                    pathState = 11;
                }
                break;

            case 11:
                if (gateWaitTimer.getElapsedTimeSeconds() >= GATE_WAIT_SECONDS) {
                    follower.followPath(paths.gateIntakeToScore(), true);
                    pathState = 12;
                }
                break;
            case 12:
                if (driveDone(T_DONE_SCORE)) {
                    leverUp();
                    startShoot();
                    pathState = 13;
                }
                break;

            case 13:
                if (actionDone()) {
                    gateRepeatsDone++;
                    if (gateRepeatsDone < 2) {
                        leverDown();
                        follower.followPath(paths.scoreToGateIntake(), true);
                        startIntake();
                        pathState = 10;
                    } else {
                        follower.followPath(paths.toIntake3(), true);
                        startIntake();
                        pathState = 20;
                    }
                }
                break;

            case 20:
                if (driveDone(T_DONE_INTAKE)) {
                    robot.intakeFSM.startTransfer();
                    follower.followPath(paths.intake3ToParkScore(), true);
                    pathState = 21;
                }
                break;

            case 21:
                if (driveDone(0.85)) {
                    startShoot();
                    pathState = 22;
                }
                break;

            case 22:
                if (actionDone()) {
                    stopIntake();
                    pathState = 23;
                }
                break;

            case 23:
                break;
        }
    }
}