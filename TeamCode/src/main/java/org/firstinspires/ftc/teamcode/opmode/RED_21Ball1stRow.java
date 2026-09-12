package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.*;
import org.firstinspires.ftc.teamcode.config.worldsPaths.GateCycling;

@Autonomous(name = "RED_21Ball1stRow", group = "Main")
public class RED_21Ball1stRow extends OpMode {

    private Robot robot;
    private GateCycling paths;

    private int state = 0;
    private int cycle = 0;

    private final Timer timer = new Timer();

    private static final double T = 0.9;
    private static final double GATE_RETURN_STOP_T = 0.8;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, Alliance.RED);
        paths = new GateCycling(robot, Alliance.RED);

        RobotConstants.currentPose = paths.start;

        robot.init();

        follower.setStartingPose(paths.start);
        follower.update();
        follower.setMaxPower(1.0);

        state = 0;
        cycle = 0;
    }

    @Override
    public void start() {
        state = 0;
        cycle = 0;

        timer.resetTimer();

        robot.intakeFSM.startTransfer();
        follower.followPath(paths.scorePreload(), true);
    }

    @Override
    public void stop() {
        RobotConstants.currentPose = follower.getPose();
        follower.setMaxPower(1.0);
    }

    private boolean done() {
        return follower.getCurrentTValue() > T && !follower.isBusy();
    }

    private void shoot() {
        robot.intakeFSM.startTransfer();
        robot.shootFSM.start();
    }

    private void startIntake() {
        robot.intakeFSM.startIntake();
    }

    private void stopIntake() {
        robot.intakeFSM.stop();
    }

    private void transfer() {
        robot.intakeFSM.startTransfer();
    }

    private void leverDown() {
        robot.intake.leverDown();
    }

    private void leverUp() {
        robot.intake.leverUp();
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

            // preload shoot
            case 0:
                if (done()) {
                    leverUp();
                    shoot();
                    state = 1;
                }
                break;

            // go to intake1
            case 1:
                if (!RobotConstants.actionBusy) {
                    follower.followPath(paths.toIntake1(), true);
                    startIntake();
                    state = 2;
                }
                break;

            // intake1 back to score
            case 2:
                if (done()) {
                    transfer();
                    follower.followPath(paths.intake1ToScore(), true);
                    state = 3;
                }
                break;

            // shoot intake1 balls
            case 3:
                if (done()) {
                    leverUp();
                    shoot();
                    state = 4;
                }
                break;

            // go to intake2
            case 4:
                if (!RobotConstants.actionBusy) {
                    follower.followPath(paths.toIntake2(), true);
                    startIntake();
                    state = 5;
                }
                break;

            // intake2 to gate hit
            case 5:
                if (done()) {
                    transfer();
                    follower.followPath(paths.intake2ToGateHit(), true);
                    state = 6;
                }
                break;

            // wait at gate hit
            case 6:
                if (done()) {
                    timer.resetTimer();
                    state = 7;
                }
                break;

            case 7:
                if (timer.getElapsedTimeSeconds() > 0.7) {
                    follower.followPath(paths.gateHitToScore(), true);
                    state = 8;
                }
                break;

            // shoot intake2 balls
            case 8:
                if (done()) {
                    leverUp();
                    shoot();
                    state = 9;
                }
                break;

            // start gate cycle
            case 9:
                if (!RobotConstants.actionBusy) {
                    leverDown();
                    startIntake();
                    follower.followPath(paths.scoreToGateIntake(cycle*0.25), true);
                    state = 10;
                }
                break;

            // wait at gate intake
            case 10:
                if (done()) {
                    timer.resetTimer();
                    state = 11;
                }
                break;

            case 11:
                if (timer.getElapsedTimeSeconds() > 1.12) {
                    if (cycle == 3) {
                        follower.followPath(paths.gateIntakeToParkScore(cycle*0.25), true);
                    } else {
                        follower.followPath(paths.gateIntakeToScore(cycle*0.25), true);
                    }
                    state = 12;
                }
                break;

            // stop intake mid-return
            case 12:
                if (follower.getCurrentTValue() > GATE_RETURN_STOP_T) {
                    //stopIntake();
                    state = 13;
                }
                break;

            // shoot gate balls
            case 13:
                if (done()) {
                    leverUp();
                    shoot();
                    state = 14;
                }
                break;

            // repeat gate cycle
            case 14:
                if (!RobotConstants.actionBusy) {
                    cycle++;

                    if (cycle < 4) {
                        leverDown();
                        startIntake();
                        follower.followPath(paths.scoreToGateIntake(cycle *0.25), true);
                        state = 10;
                    } else {
                        state = 15;
                    }
                }
                break;

            case 15:
                break;
        }

        telemetry.addData("state", state);
        telemetry.addData("cycle", cycle);
        telemetry.addData("t", follower.getCurrentTValue());
        telemetry.addData("busy", follower.isBusy());
        telemetry.addData("actionBusy", RobotConstants.actionBusy);
        telemetry.update();
    }
}