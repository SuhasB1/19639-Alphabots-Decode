package org.firstinspires.ftc.teamcode.opmode.oldRegionals;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.paths.FarCycles;
@Disabled
@Autonomous(name = "BLUE_Far Cycles 4x + Third Row", group = "Main")
public class FarCyclesx4ThirdAuto_BLUE extends OpMode {

    private Robot robot;
    private FarCycles paths;

    private static final double T_DONE = 0.9;

    private static final double DRIVE_POWER_SHOOT  = 1;
    private static final double DRIVE_POWER_INTAKE = 1;

    public static double WAIT_PRELOAD_SHOT = 0.5;

    public static double WAIT_CYCLE1_SHOT = 0.65;
    public static double WAIT_CYCLE2_SHOT = 0.65;
    public static double WAIT_CYCLE3_SHOT = 0.65;
    public static double WAIT_CYCLE4_SHOT = 0.65;

    public static double WAIT_INTAKE1_SETTLE = 0.3;
    public static double WAIT_INTAKE2_SETTLE = 0.1;
    public static double WAIT_INTAKE3_SETTLE = 0.1;

    public static double WAIT_WIGGLE1 = 0;
    public static double WAIT_WIGGLE2 = 1;
    public static double WAIT_WIGGLE3 = 1;

    public static double WAIT_SHAKE = 0.6;



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
        robot.update(gamepad1, gamepad2,
                follower.getPose().getX(),
                follower.getPose().getY());
        robot.shooter.updateVelocityControl(
                follower.getPose().getX(),
                follower.getPose().getY());

        switch (state) {

            // PRELOAD
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

            // THIRD ROW (NO WIGGLE)
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
                    robot.intakeFSM.startTransfer();
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
                if (waitTimer.getElapsedTimeSeconds() > WAIT_CYCLE1_SHOT) {
                    startShoot();
                    state = 6;
                }
                break;

            // HP1
            case 6:
                if (!RobotConstants.actionBusy) {
                    follower.followPath(paths.shootToIntakeHP1(), true);
                    startIntake();
                    state = 7;
                }
                break;

            case 7:
                if (driveDone()) {
                    follower.followPath(paths.intakeWiggle(paths.intakeHP1), true);
                    waitTimer.resetTimer();
                    state = 8;
                }
                break;

            case 8:
                if (driveDone() || waitTimer.getElapsedTimeSeconds() > WAIT_WIGGLE1) {
                    follower.followPath(paths.intakeShake(paths.intakeHP1), true);
                    waitTimer.resetTimer();
                    state = 9;
                }
                break;

            case 9:
                if (driveDone() || waitTimer.getElapsedTimeSeconds() > WAIT_SHAKE) {
                    waitTimer.resetTimer();
                    state = 10;
                }
                break;

            case 10:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_INTAKE1_SETTLE) {
                    stopIntake();
                    robot.intakeFSM.startTransfer();
                    follower.setMaxPower(DRIVE_POWER_SHOOT);
                    follower.followPath(paths.intakeHP1ToShoot(), true);
                    state = 11;
                }
                break;

            case 11:
                if (driveDone()) {
                    waitTimer.resetTimer();
                    state = 12;
                }
                break;

            case 12:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_CYCLE2_SHOT) {
                    startShoot();
                    state = 13;
                }
                break;

            // HP2
            case 13:
                if (!RobotConstants.actionBusy) {
                    follower.setMaxPower(DRIVE_POWER_INTAKE);
                    follower.followPath(paths.shootToIntakeTHIRDHP2(), true);
                    startIntake();
                    state = 14;
                }
                break;

            case 14:
                if (driveDone()) {
                    follower.followPath(paths.intakeWiggle(paths.intakeTHIRDHP2), true);
                    waitTimer.resetTimer();
                    state = 15;
                }
                break;

            case 15:
                if (driveDone() || waitTimer.getElapsedTimeSeconds() > WAIT_WIGGLE2) {
                    follower.followPath(paths.intakeShake(paths.intakeTHIRDHP2), true);
                    waitTimer.resetTimer();
                    state = 16;
                }
                break;

            case 16:
                if (driveDone() || waitTimer.getElapsedTimeSeconds() > WAIT_SHAKE) {
                    waitTimer.resetTimer();
                    state = 17;
                }
                break;

            case 17:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_INTAKE2_SETTLE) {
                    stopIntake();
                    robot.intakeFSM.startTransfer();
                    follower.setMaxPower(DRIVE_POWER_SHOOT);
                    follower.followPath(paths.intakeTHIRDHP2ToShoot(), true);
                    state = 18;
                }
                break;

            case 18:
                if (driveDone()) {
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

            // HP3
            case 20:
                if (!RobotConstants.actionBusy) {
                    follower.setMaxPower(DRIVE_POWER_INTAKE);
                    follower.followPath(paths.shootToIntakeHP3(), true);
                    startIntake();
                    state = 21;
                }
                break;

            case 21:
                if (driveDone()) {
                    follower.followPath(paths.intakeWiggle(paths.intakeHP3), true);
                    waitTimer.resetTimer();
                    state = 22;
                }
                break;

            case 22:
                if (driveDone() || waitTimer.getElapsedTimeSeconds() > WAIT_WIGGLE3) {
                    follower.followPath(paths.intakeShake(paths.intakeHP3), true);
                    waitTimer.resetTimer();
                    state = 23;
                }
                break;

            case 23:
                if (driveDone() || waitTimer.getElapsedTimeSeconds() > WAIT_SHAKE) {
                    waitTimer.resetTimer();
                    state = 24;
                }
                break;

            case 24:
                if (waitTimer.getElapsedTimeSeconds() > WAIT_INTAKE3_SETTLE) {
                    stopIntake();
                    robot.intakeFSM.startTransfer();
                    follower.setMaxPower(DRIVE_POWER_SHOOT);
                    follower.followPath(paths.intakeHP3ToShoot(), true);
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
                if (waitTimer.getElapsedTimeSeconds() > WAIT_CYCLE4_SHOT) {
                    startShoot();
                    state = 27;
                }
                break;

            case 27:
                if (!RobotConstants.actionBusy) {
                    follower.followPath(paths.parkFromShoot(), true);
                    state = 99;
                }
                break;

            case 99:
                break;
        }

        telemetry.addData("state", state);
        telemetry.update();
    }

    @Override
    public void stop() {
        RobotConstants.currentPose = follower.getPose();
        follower.setMaxPower(1);
    }
}