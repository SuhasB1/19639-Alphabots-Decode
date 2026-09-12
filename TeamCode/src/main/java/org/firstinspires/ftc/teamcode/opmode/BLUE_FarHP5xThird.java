package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.worldsPaths.FarPaths;

@Autonomous(name = "BLUE_Far HP 5x + Third Row", group = "Main")
public class BLUE_FarHP5xThird extends OpMode {

    private Robot robot;
    private FarPaths paths;

    private static final double T_DONE = 0.9;
    private static final double RETURN_STOP_INTAKE_T = 0.75;

    private static final double DRIVE_POWER_SHOOT = 1.0;
    private static final double DRIVE_POWER_INTAKE = 1.0;

    public static double WAIT_PRELOAD_SHOT = 0.3;
    public static double WAIT_THIRD_ROW_SHOT = 0;

    public static double WAIT_HP1_SHOT = 0.05;
    public static double WAIT_HP2_SHOT = 0.05;
    public static double WAIT_HP3_SHOT = 0.05;
    public static double WAIT_HP4_SHOT = 0.05;
    public static double WAIT_HP5_SHOT = 0.05;
    public static double WAIT_HP6_SHOT = 0;

    public static double WAIT_HP1_SETTLE = 0.1;
    public static double WAIT_HP2_SETTLE = 0.1;
    public static double WAIT_HP3_SETTLE = 0.1;
    public static double WAIT_HP4_SETTLE = 0.1;
    public static double WAIT_HP5_SETTLE = 0.1;
    public static double WAIT_HP6_SETTLE = 0.05;

    private final Timer waitTimer = new Timer();

    private int state = 0;
    private boolean stopIntakeOnReturn = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, Alliance.BLUE);
        robot.init();

        paths = new FarPaths(robot, Alliance.BLUE);

        follower.setStartingPose(paths.start);
        follower.update();
        RobotConstants.currentPose = paths.start;

        follower.setMaxPower(DRIVE_POWER_SHOOT);
        state = 0;
        stopIntakeOnReturn = false;
    }

    @Override
    public void start() {
        robot.intakeFSM.startTransfer();
        follower.followPath(paths.startToShoot(), true);
        waitTimer.resetTimer();
        state = 0;
        stopIntakeOnReturn = false;
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
        stopIntakeOnReturn = false;
    }

    private void stopIntakeAndTransfer() {
        robot.intakeFSM.stop();
        robot.intakeFSM.startTransfer();
    }

    private void beginReturnToShoot() {
        stopIntakeOnReturn = false;
        follower.setMaxPower(DRIVE_POWER_SHOOT);
    }

    private void stopIntakeDuringReturn() {
        if (!stopIntakeOnReturn && follower.getCurrentTValue() > RETURN_STOP_INTAKE_T) {
            robot.intakeFSM.stop();
            stopIntakeOnReturn = true;
        }
    }

    @Override
    public void loop() {
        follower.update();

        robot.update(
                gamepad1,
                gamepad2,
                follower.getPose().getX(),
                follower.getPose().getY()
        );

        robot.shooter.updateVelocityControl(
                follower.getPose().getX(),
                follower.getPose().getY()
        );

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
                    follower.followPath(paths.shootToIntake3rd(), true);
                    startIntake();
                    state = 3;
                }
                break;

            case 3:
                if (driveDone()) {
                    stopIntakeAndTransfer();
                    follower.setMaxPower(DRIVE_POWER_SHOOT);
                    follower.followPath(paths.intake3rdToShoot(), true);
                    state = 4;
                }
                break;

            case 4:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 5;
                }
                break;

            case 5:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_THIRD_ROW_SHOT) {
                    startShoot();
                    state = 6;
                }
                break;

            case 6:
                if (!RobotConstants.actionBusy) {
                    follower.setMaxPower(DRIVE_POWER_INTAKE);
                    follower.followPath(paths.shootToIntakeHP1(), true);
                    startIntake();
                    state = 7;
                }
                break;

            case 7:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 8;
                }
                break;

            case 8:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 9;
                }
                break;

            case 9:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_HP1_SETTLE) {
                    beginReturnToShoot();
                    follower.followPath(paths.intakeHP1ToShoot(), true);
                    state = 10;
                }
                break;

            case 10:
                stopIntakeDuringReturn();
                if (driveDone()) {
                    robot.intakeFSM.startTransfer();
                    waitTimer.resetTimer();
                    state = 11;
                }
                break;

            case 11:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_HP1_SHOT) {
                    startShoot();
                    state = 12;
                }
                break;

            case 12:
                if (!RobotConstants.actionBusy) {
                    follower.setMaxPower(DRIVE_POWER_INTAKE);
                    follower.followPath(paths.shootToIntakeHP2(), true);
                    startIntake();
                    state = 13;
                }
                break;

            case 13:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 14;
                }
                break;

            case 14:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 15;
                }
                break;

            case 15:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_HP2_SETTLE) {
                    beginReturnToShoot();
                    follower.followPath(paths.intakeHP2ToShoot(), true);
                    state = 16;
                }
                break;

            case 16:
                stopIntakeDuringReturn();
                if (driveDone()) {
                    robot.intakeFSM.startTransfer();
                    waitTimer.resetTimer();
                    state = 17;
                }
                break;

            case 17:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_HP2_SHOT) {
                    startShoot();
                    state = 18;
                }
                break;

            case 18:
                if (!RobotConstants.actionBusy) {
                    follower.setMaxPower(DRIVE_POWER_INTAKE);
                    follower.followPath(paths.shootToIntakeHP3(), true);
                    startIntake();
                    state = 19;
                }
                break;

            case 19:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 20;
                }
                break;

            case 20:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 21;
                }
                break;

            case 21:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_HP3_SETTLE) {
                    beginReturnToShoot();
                    follower.followPath(paths.intakeHP3ToShoot(), true);
                    state = 22;
                }
                break;

            case 22:
                stopIntakeDuringReturn();
                if (driveDone()) {
                    robot.intakeFSM.startTransfer();
                    waitTimer.resetTimer();
                    state = 23;
                }
                break;

            case 23:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_HP3_SHOT) {
                    startShoot();
                    state = 24;
                }
                break;

            case 24:
                if (!RobotConstants.actionBusy) {
                    follower.setMaxPower(DRIVE_POWER_INTAKE);
                    follower.followPath(paths.shootToIntakeHP4(), true);
                    startIntake();
                    state = 25;
                }
                break;

            case 25:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 26;
                }
                break;

            case 26:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 27;
                }
                break;

            case 27:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_HP4_SETTLE) {
                    beginReturnToShoot();
                    follower.followPath(paths.intakeHP4ToShoot(), true);
                    state = 28;
                }
                break;

            case 28:
                stopIntakeDuringReturn();
                if (driveDone()) {
                    robot.intakeFSM.startTransfer();
                    waitTimer.resetTimer();
                    state = 29;
                }
                break;

            case 29:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_HP4_SHOT) {
                    startShoot();
                    state = 30;
                }
                break;

            case 30:
                if (!RobotConstants.actionBusy) {
                    follower.setMaxPower(DRIVE_POWER_INTAKE);
                    follower.followPath(paths.shootToIntakeHP5(), true);
                    startIntake();
                    state = 31;
                }
                break;

            case 31:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 32;
                }
                break;

            case 32:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 33;
                }
                break;

            case 33:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_HP5_SETTLE) {
                    beginReturnToShoot();
                    follower.followPath(paths.intakeHP5ToShoot(), true);
                    state = 34;
                }
                break;

            case 34:
                stopIntakeDuringReturn();
                if (driveDone()) {
                    robot.intakeFSM.startTransfer();
                    waitTimer.resetTimer();
                    state = 35;
                }
                break;

            case 35:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_HP5_SHOT) {
                    startShoot();
                    state = 36;
                }
                break;

            case 36:
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
        telemetry.addData("stopIntakeOnReturn", stopIntakeOnReturn);
        telemetry.update();
    }

    @Override
    public void stop() {
        RobotConstants.currentPose = follower.getPose();
        follower.setMaxPower(1.0);
    }
}