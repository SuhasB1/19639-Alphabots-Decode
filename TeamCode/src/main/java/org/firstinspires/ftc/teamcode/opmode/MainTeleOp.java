package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.fsm.IntakeFSM;
import org.firstinspires.ftc.teamcode.config.fsm.ShootFSM;

@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends OpMode {

    private Robot robot;

    private boolean prevLT = false;
    private boolean prevRB = false;
    private boolean prevDpadDown = false;
    private boolean prevRT = false;

    private ShootFSM.State prevShootState = ShootFSM.State.IDLE;
    private final ElapsedTime loopTime = new ElapsedTime();

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry, RobotConstants.currentTeam);
        robot.init();
        follower.setStartingPose(RobotConstants.currentPose);
        follower.update();
        robot.drive.speed = 0.55;
    }

    @Override
    public void start() {
        robot.start();
        loopTime.reset();

        prevLT = (gamepad1.left_trigger > 0.1);
        prevRB = gamepad1.right_bumper;
        prevDpadDown = gamepad1.dpad_down;
        prevRT = (gamepad1.right_trigger > 0.1);
        prevShootState = robot.shootFSM.getState();
    }

    @Override
    public void loop() {
        loopTime.reset();

        robot.drive.speed = (gamepad1.left_bumper) ? 0.3 : 0.55;

        boolean dpadDown = gamepad1.dpad_down;
        if (dpadDown && !prevDpadDown) {
            robot.drive.resetPose(6.1, 8.1, Math.toRadians(0));
        }
        prevDpadDown = dpadDown;

        boolean lt = gamepad1.left_trigger > 0.1;
        if (lt && !prevLT) {
            IntakeFSM.State s = robot.intakeFSM.getState();
            if (s == IntakeFSM.State.IDLE) robot.intakeFSM.startIntake();
            else robot.intakeFSM.stop();
        }
        prevLT = lt;

        if (gamepad1.triangle) {
            robot.intakeFSM.startTransfer();
        }

        boolean rb = gamepad1.right_bumper;
        if (rb && !prevRB) {
            ShootFSM.State shootState = robot.shootFSM.getState();
            if (shootState == ShootFSM.State.IDLE) {
                robot.shootFSM.start();
            } else {
                robot.shootFSM.stop();
                robot.intakeFSM.stop();
            }
        }
        prevRB = rb;

        robot.update(gamepad1, gamepad2, robot.drive.getX(), robot.drive.getY());

        ShootFSM.State shootNow = robot.shootFSM.getState();
        boolean shootingActive = (shootNow == ShootFSM.State.SPINUP || shootNow == ShootFSM.State.DONE);

        if (shootingActive) {
            robot.drive.headingLockPathMode(RobotConstants.goalX, RobotConstants.goalY);

        } else {
            if (prevShootState != ShootFSM.State.IDLE && shootNow == ShootFSM.State.IDLE) {
                robot.drive.returnToTeleop();
                robot.intakeFSM.state = IntakeFSM.State.IDLE;
            }

            boolean rtHeld = gamepad1.right_trigger > 0.1;
            if (rtHeld && !prevRT) robot.drive.resetHeadingPID();
            prevRT = rtHeld;

            if (rtHeld) robot.drive.headingLockPID(gamepad1, RobotConstants.goalX, RobotConstants.goalY);
            else robot.drive.updateTeleop(gamepad1);
        }

        prevShootState = shootNow;

        telemetry.addData("loop time (ms)", loopTime.milliseconds());
        telemetry.addData("shootState", shootNow);
        robot.telemetry();
        telemetry.update();
    }
}
