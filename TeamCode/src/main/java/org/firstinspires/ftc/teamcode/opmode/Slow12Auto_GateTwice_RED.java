package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.paths.Slow12;

@Autonomous(name = "RED_Slow12 Gate Twice", group = "Main")
public class Slow12Auto_GateTwice_RED extends OpMode {

    private Robot robot;
    private Slow12 paths;

    private int pathState = 0;

    private static final double T_DONE = 0.95;

    private static final double DRIVE_POWER_NORMAL = 0.85;
    private static final double DRIVE_POWER_INTAKE = 0.75;

    private final Timer pathTimer = new Timer();

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, Alliance.RED);
        paths = new Slow12(robot, Alliance.RED);

        RobotConstants.currentPose = paths.start;

        robot.init();

        follower.setStartingPose(paths.start);
        follower.update();

        follower.setMaxPower(DRIVE_POWER_NORMAL);

        pathState = 0;
    }

    @Override
    public void start() {
        pathState = 0;
        pathTimer.resetTimer();
        robot.intakeFSM.startTransfer();
        follower.setMaxPower(DRIVE_POWER_NORMAL);
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

    private boolean driveDone() {
        return follower.getCurrentTValue() > T_DONE && !follower.isBusy();
    }

    private boolean actionDone() {
        return !RobotConstants.actionBusy;
    }

    private void setDrivePower(double p) {
        follower.setMaxPower(p);
    }

    private void startShoot() {
        robot.intakeFSM.startTransfer();
        robot.shootFSM.start();
    }

    private void startIntake() {
        robot.intakeFSM.startIntake();
    }

    private void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                if (driveDone()) {
                    startShoot();
                    pathState = 1;
                }
                break;

            case 1:
                if (actionDone()) {
                    setDrivePower(DRIVE_POWER_INTAKE);
                    follower.followPath(paths.intake1(), true);
                    startIntake();
                    pathState = 2;
                }
                break;

            case 2:
                if (driveDone()) {
                    robot.intakeFSM.startTransfer();
                    setDrivePower(0.75);
                    follower.followPath(paths.gateFromIntake1(), true);
                    pathState = 25;
                }
                break;

            case 25:
                if (driveDone()) {
                    pathTimer.resetTimer();
                    pathState = 3;
                }
                break;

            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                    follower.followPath(paths.scoreFromGate1(), true);
                    pathState = 4;
                }
                break;

            case 4:
                if (driveDone()) {
                    startShoot();
                    pathState = 5;
                }
                break;

            case 5:
                if (actionDone()) {
                    setDrivePower(DRIVE_POWER_INTAKE);
                    follower.followPath(paths.intake2(), true);
                    startIntake();
                    pathState = 6;
                }
                break;

            case 6:
                if (driveDone()) {
                    robot.intakeFSM.startTransfer();
                    setDrivePower(DRIVE_POWER_NORMAL);
                    follower.followPath(paths.gate(), true);
                    pathState = 65;
                }
                break;

            case 65:
                if (driveDone()) {
                    pathTimer.resetTimer();
                    pathState = 7;
                }
                break;

            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    follower.followPath(paths.score2(), true);
                    pathState = 8;
                }
                break;

            case 8:
                if (driveDone()) {
                    startShoot();
                    pathState = 9;
                }
                break;

            case 9:
                if (actionDone()) {
                    setDrivePower(DRIVE_POWER_INTAKE);
                    follower.followPath(paths.intake3(), true);
                    startIntake();
                    pathState = 10;
                }
                break;

            case 10:
                if (driveDone()) {
                    robot.intakeFSM.startTransfer();
                    setDrivePower(0.75);
                    follower.followPath(paths.score3(), true);
                    pathState = 11;
                }
                break;

            case 11:
                if (driveDone()) {
                    startShoot();
                    pathState = 12;
                }
                break;

            case 12:
                if (actionDone()) {
                    pathState = 13;
                }
                break;

            case 13:
                break;
        }
    }
}

