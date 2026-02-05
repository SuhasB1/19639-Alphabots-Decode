package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.paths.Fast15;

@Autonomous(name = "RED_Fast15 Auto", group = "Main")
public class Fast15Auto_RED extends OpMode {

    private Robot robot;
    private Fast15 paths;

    private int pathState = 0;

    private static final double T_DONE_SCORE  = 0.95;
    private static final double T_DONE_INTAKE = 0.95;

    Timer pathTimer = new Timer();

    Timer autoTimer = new Timer();


    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, Alliance.RED);
        paths = new Fast15(robot, Alliance.RED);

        RobotConstants.currentPose = paths.start;

        robot.init();

        follower.setStartingPose(paths.start);
        follower.update();

        follower.setMaxPower(0.95);

        pathState = 0;
    }

    @Override
    public void start() {
        pathState = 0;
        pathTimer.resetTimer();
        autoTimer.resetTimer();
        robot.intakeFSM.startTransfer();
        follower.followPath(paths.scoreP());
    }

    @Override
    public void loop() {
        follower.update();

        robot.update(gamepad1, gamepad2, follower.getPose().getX(), follower.getPose().getY());

        autonomousPathUpdate();

        telemetry.addData("pathState", pathState);
        telemetry.addData("t", follower.getCurrentTValue());
        telemetry.addData("actionBusy", RobotConstants.actionBusy);
        telemetry.update();
    }

    @Override
    public void stop(){
        RobotConstants.currentPose = follower.getPose();
        follower.setMaxPower(1);
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

    private void stopIntake() {
        robot.intakeFSM.stop();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                if (driveDone(T_DONE_SCORE)) {
                    startShoot();
                    pathState = 1;
                }
                break;

            case 1:
                if (actionDone()) {
                    follower.followPath(paths.intake1(), true);
                    startIntake();
                    pathState = 2;
                }
                break;

            case 2:
                if (driveDone(T_DONE_INTAKE)) {
                    follower.followPath(paths.score1(), true);
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
                    follower.followPath(paths.intake2(), true);
                    startIntake();
                    pathState = 5;
                }
                break;

            case 5:
                if (driveDone(T_DONE_INTAKE)) {
                    robot.intakeFSM.startTransfer();
                    follower.followPath(paths.gate(), true);
                    pathState = 550;
                }
                break;
            case 550:
                if (driveDone(T_DONE_SCORE)) {
                    pathTimer.resetTimer();
                    pathState = 6;
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 0.9) {
                    follower.followPath(paths.score2(), true);
                    pathState = 7;
                }
                break;

            case 7:
                if (driveDone(T_DONE_SCORE)) {
                    startShoot();
                    pathState = 8;
                }
                break;

            case 8:
                if (actionDone()) {
                    stopIntake();
                    follower.followPath(paths.intake3(), true);
                    startIntake();
                    pathTimer.resetTimer();
                    pathState = 9;
                }
                break;

            case 9:
                if (driveDone(T_DONE_INTAKE) && pathTimer.getElapsedTime() > 0.3) {
                    robot.intakeFSM.startTransfer();
                    follower.followPath(paths.score3(), true);
                    pathState = 10;
                }
                break;

            case 10:
                if (driveDone(T_DONE_SCORE)) {
                    startShoot();
                    pathState = 11;
                }
                break;

            case 11:
                if (actionDone()) {
                    stopIntake();
                    follower.followPath(paths.intakeCorner(), true);
                    startIntake();
                    pathState = 1150;
                }
                break;
            case 1150:
                if(driveDone(T_DONE_SCORE)){
                    pathTimer.resetTimer();
                    pathState = 12;
                }
            case 12:
                if(driveDone(T_DONE_SCORE)){
                    follower.setMaxPower(0.75);
                    follower.followPath(paths.scoreCorner(), true);
                    pathState = 1250;
                }
                break;
            case 1250:
                if (driveDone(0.4)) {
                    stopIntake();
                    pathState = 13;
                }
                break;
            case 13:
                if (driveDone(T_DONE_SCORE)) {
                    startShoot();
                    pathState = 14;
                }
                break;
            case 14:
                if (actionDone()) {
                    stopIntake();
                    pathState = 15;
                }
                break;
            case 15:
                if (driveDone(T_DONE_SCORE)) {
                    //done
                }
                break;
        }
    }
}