package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.paths.FarCycles;

@Autonomous(name = "RED_Far Cycles 3x", group = "Main")
public class FarCycles12Auto_RED extends OpMode {

    private Robot robot;
    private FarCycles paths;

    private static final double T_DONE = 0.95;

    private static final double DRIVE_POWER_SHOOT  = 0.9;
    private static final double DRIVE_POWER_INTAKE = 0.85;

    public static double WAIT_PRELOAD_SHOT = 1;

    public static double WAIT_CYCLE1_SHOT = 0.5;
    public static double WAIT_CYCLE2_SHOT = 0.5;
    public static double WAIT_CYCLE3_SHOT = 0.5;

    public static double WAIT_INTAKE1_SETTLE = 0.1;
    public static double WAIT_INTAKE2_SETTLE = 0.1;
    public static double WAIT_INTAKE3_SETTLE = 0.1;

    public static double WAIT_WIGGLE1 = 0.1;
    public static double WAIT_WIGGLE2 = 0.1;
    public static double WAIT_WIGGLE3 = 0.1;

    private final Timer guardTimer = new Timer();
    private final Timer waitTimer = new Timer();

    private int state = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, Alliance.RED);
        robot.init();

        paths = new FarCycles(robot, Alliance.RED);

        follower.setStartingPose(paths.start);
        follower.update();
        RobotConstants.currentPose = paths.start;

        follower.setMaxPower(DRIVE_POWER_SHOOT);

        state = 0;
    }

    @Override
    public void start() {
        robot.intakeFSM.startTransfer();

        follower.setMaxPower(DRIVE_POWER_SHOOT);
        follower.followPath(paths.startToShoot(), true);

        guardTimer.resetTimer();
        waitTimer.resetTimer();
        state = 0;
    }

    private boolean driveDone() {
        return !follower.isBusy() || follower.getCurrentTValue() > T_DONE;
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

    @Override
    public void loop() {
        follower.update();
        robot.update(gamepad1, gamepad2, follower.getPose().getX(), follower.getPose().getY());

        switch (state) {

            case 0:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 1;
                }
                break;

            case 1:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_PRELOAD_SHOT) {
                    startShoot();
                    state = 2;
                }
                break;

            case 2:
                if (!RobotConstants.actionBusy) {
                    follower.setMaxPower(DRIVE_POWER_INTAKE);
                    follower.followPath(paths.shootToIntakeHP1(), true);
                    startIntake();
                    guardTimer.resetTimer();
                    state = 3;
                }
                break;

            case 3:
                if (driveDone()) {
                    follower.followPath(paths.intakeWiggle(paths.intakeHP1), true);
                    waitTimer.resetTimer();
                    guardTimer.resetTimer();
                    state = 4;
                }
                break;

            case 4:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_WIGGLE1 && driveDone()) {
                    waitTimer.resetTimer();
                    state = 5;
                }
                break;

            case 5:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_INTAKE1_SETTLE) {
                    stopIntake();
                    robot.intakeFSM.startTransfer();

                    follower.setMaxPower(DRIVE_POWER_SHOOT);
                    follower.followPath(paths.intakeHP1ToShoot(), true);

                    guardTimer.resetTimer();
                    state = 6;
                }
                break;

            case 6:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 7;
                }
                break;

            case 7:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_CYCLE1_SHOT) {
                    startShoot();
                    state = 8;
                }
                break;

            case 8:
                if (!RobotConstants.actionBusy) {
                    follower.setMaxPower(DRIVE_POWER_INTAKE);
                    follower.followPath(paths.shootToIntakeHP2(), true);
                    startIntake();
                    guardTimer.resetTimer();
                    state = 9;
                }
                break;

            case 9:
                if (driveDone()) {
                    follower.followPath(paths.intakeWiggle(paths.intakeHP2), true);
                    waitTimer.resetTimer();
                    guardTimer.resetTimer();
                    state = 10;
                }
                break;

            case 10:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_WIGGLE2 && driveDone()) {
                    waitTimer.resetTimer();
                    state = 11;
                }
                break;

            case 11:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_INTAKE2_SETTLE) {
                    stopIntake();
                    robot.intakeFSM.startTransfer();

                    follower.setMaxPower(DRIVE_POWER_SHOOT);
                    follower.followPath(paths.intakeHP2ToShoot(), true);

                    guardTimer.resetTimer();
                    state = 12;
                }
                break;

            case 12:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 13;
                }
                break;

            case 13:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_CYCLE2_SHOT) {
                    startShoot();
                    state = 14;
                }
                break;

            case 14:
                if (!RobotConstants.actionBusy) {
                    follower.setMaxPower(DRIVE_POWER_INTAKE);
                    follower.followPath(paths.shootToIntakeHP3(), true);
                    startIntake();
                    guardTimer.resetTimer();
                    state = 15;
                }
                break;

            case 15:
                if (driveDone()) {
                    follower.followPath(paths.intakeWiggle(paths.intakeHP3), true);
                    waitTimer.resetTimer();
                    guardTimer.resetTimer();
                    state = 16;
                }
                break;

            case 16:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_WIGGLE3 && driveDone()) {
                    waitTimer.resetTimer();
                    state = 17;
                }
                break;

            case 17:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_INTAKE3_SETTLE) {
                    stopIntake();
                    robot.intakeFSM.startTransfer();

                    follower.setMaxPower(DRIVE_POWER_SHOOT);
                    follower.followPath(paths.intakeHP3ToShoot(), true);

                    guardTimer.resetTimer();
                    state = 18;
                }
                break;

            case 18:
                if ( driveDone()) {
                    waitTimer.resetTimer();
                    state = 19;
                }
                break;

            case 19:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_CYCLE3_SHOT) {
                    startShoot();
                    state = 20;
                }
                break;

            case 20:
                if (!RobotConstants.actionBusy) {
                    follower.followPath(paths.parkFromShoot(), true);
                    state = 99;
                }
                break;

            case 99:
                break;
        }

        telemetry.addData("state", state);
        telemetry.addData("t", follower.getCurrentTValue());
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("actionBusy", RobotConstants.actionBusy);
        telemetry.update();
    }

    @Override
    public void stop() {
        RobotConstants.currentPose = follower.getPose();
        follower.setMaxPower(1);
    }
}

