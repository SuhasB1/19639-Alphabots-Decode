package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.paths.FarCycles;

@Autonomous(name = "BLUE_Far Cycles 4x + Third Row", group = "Main")
public class FarCyclesx4ThirdAuto_BLUE extends OpMode {

    private Robot robot;
    private FarCycles paths;

    private static final double T_DONE = 0.9;

    private static final double DRIVE_POWER_SHOOT  = 0.9;
    private static final double DRIVE_POWER_INTAKE = 0.9;

    public static double WAIT_PRELOAD_SHOT = 0.4;

    public static double WAIT_CYCLE1_SHOT = 0.4;
    public static double WAIT_CYCLE2_SHOT = 0.4;
    public static double WAIT_CYCLE3_SHOT = 0.4;
    public static double WAIT_CYCLE4_SHOT = 0.4;

    public static double WAIT_INTAKE1_SETTLE = 1;
    public static double WAIT_INTAKE2_SETTLE = 0.2;
    public static double WAIT_INTAKE3_SETTLE = 0.2;


    public static double WAIT_WIGGLE1 = 1.2;
    public static double WAIT_WIGGLE2 = 1.2;
    public static double WAIT_WIGGLE3 = 1.2;


    private final Timer waitTimer = new Timer();

    private int state = 0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, Alliance.BLUE);
        robot.init();

        paths = new FarCycles(robot, Alliance.BLUE);

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
        robot.shooter.updateVelocityControl(follower.getPose().getX(), follower.getPose().getY());

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
                    follower.setMaxPower(DRIVE_POWER_INTAKE);
                    robot.intakeFSM.startTransfer();
                    follower.followPath(paths.intake3rdToShoot(), true);
                    state = 4;
                }
                break;

            case 4:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 45;
                }
                break;

            case 45:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_CYCLE1_SHOT) {
                    startShoot();
                    state = 5;
                }
                break;

            case 5:
                if (!RobotConstants.actionBusy) {
                    follower.setMaxPower(DRIVE_POWER_INTAKE);
                    follower.followPath(paths.shootToIntakeHP1(), true);
                    startIntake();
                    state = 6;
                }
                break;

            case 6:
                if (driveDone()) {
                    //follower.followPath(paths.intakeWiggle(paths.intakeHP1), true);
                    waitTimer.resetTimer();
                    state = 7;
                }
                break;

            case 7:
                if (driveDone() || waitTimer.getElapsedTimeSeconds() > WAIT_WIGGLE1) {
                    waitTimer.resetTimer();
                    state = 8;
                }
                break;

            case 8:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_INTAKE1_SETTLE) {
                    stopIntake();
                    robot.intakeFSM.startTransfer();

                    follower.setMaxPower(DRIVE_POWER_SHOOT);
                    follower.followPath(paths.intakeHP1ToShoot(), true);

                    state = 9;
                }
                break;

            case 9:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 10;
                }
                break;

            case 10:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_CYCLE2_SHOT) {
                    startShoot();
                    state = 11;
                }
                break;

            case 11:
                if (!RobotConstants.actionBusy) {
                    follower.setMaxPower(DRIVE_POWER_INTAKE);
                    follower.followPath(paths.shootToIntakeHP2(), true);
                    startIntake();
                    state = 12;
                }
                break;

            case 12:
                if (driveDone()) {
                    follower.followPath(paths.intakeWiggle(paths.intakeHP2), true);
                    waitTimer.resetTimer();
                    state = 13;
                }
                break;

            case 13:
                if (driveDone() || waitTimer.getElapsedTimeSeconds() > WAIT_WIGGLE2) {
                    waitTimer.resetTimer();
                    state = 14;
                }
                break;

            case 14:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_INTAKE2_SETTLE) {
                    stopIntake();
                    robot.intakeFSM.startTransfer();

                    follower.setMaxPower(DRIVE_POWER_SHOOT);
                    follower.followPath(paths.intakeHP2ToShoot(), true);

                    state = 15;
                }
                break;

            case 15:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 16;
                }
                break;

            case 16:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_CYCLE3_SHOT) {
                    startShoot();
                    state = 17;
                }
                break;

            case 17:
                if (!RobotConstants.actionBusy) {
                    follower.setMaxPower(DRIVE_POWER_INTAKE);
                    follower.followPath(paths.shootToIntakeHP3(), true);
                    startIntake();
                    state = 18;
                }
                break;

            case 18:
                if (driveDone()) {
                    follower.followPath(paths.intakeWiggle(paths.intakeHP3), true);
                    waitTimer.resetTimer();
                    state = 19;
                }
                break;

            case 19:
                if (driveDone() || waitTimer.getElapsedTimeSeconds() > WAIT_WIGGLE3) {
                    waitTimer.resetTimer();
                    state = 20;
                }
                break;

            case 20:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_INTAKE3_SETTLE) {
                    stopIntake();
                    robot.intakeFSM.startTransfer();

                    follower.setMaxPower(DRIVE_POWER_SHOOT);
                    follower.followPath(paths.intakeHP3ToShoot(), true);

                    state = 21;
                }
                break;

            case 21:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 22;
                }
                break;

            case 22:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_CYCLE4_SHOT) {
                    startShoot();
                    state = 23;
                }
                break;
            case 23:
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

