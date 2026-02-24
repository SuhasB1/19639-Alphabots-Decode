package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.shootAnywhere.ShooterCloseBaseModel;
import org.firstinspires.ftc.teamcode.config.shootAnywhere.ShooterCloseLUT;
import org.firstinspires.ftc.teamcode.config.shootAnywhere.ShooterParams;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class ShooterBaseVsLUTTest extends OpMode {

    private DcMotorEx shooterLeft, shooterRight;

    private Follower follower;

    private ShooterCloseLUT closeLUT;

    private static final double GOAL_X = 6.0;
    private static final double GOAL_Y = 138.0;

    double velocity = 0;

    public static double kP = 0.0125;
    public static double kV = 0.0003215;
    public static double kS = 0.09;

    private enum Mode {
        OFF,
        BASE_MODEL,
        LUT
    }

    private Mode mode = Mode.OFF;

    @Override
    public void init() {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(5,6.5));

        closeLUT = new ShooterCloseLUT();

        telemetry.addLine("Shooter Base vs LUT Test Ready");
        telemetry.addLine("Square = BASE MODEL");
        telemetry.addLine("Circle = LUT");
        telemetry.addLine("Cross = OFF");
    }

    @Override
    public void loop() {
        follower.update();
        velocity = -shooterLeft.getVelocity();

        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        double dx = robotX - GOAL_X;
        double dy = robotY - GOAL_Y;
        double distance = Math.hypot(dx, dy);

        double baseVel = ShooterCloseBaseModel.velocityFromDistance(distance);

        ShooterParams lutParams = closeLUT.getFromRobotPose(robotX, robotY);
        double lutVel = lutParams.shooterVel;

        if (gamepad1.square) mode = Mode.BASE_MODEL;
        if (gamepad1.circle) mode = Mode.LUT;
        if (gamepad1.cross) mode = Mode.OFF;

        switch (mode) {
            case BASE_MODEL:
                double basePower =
                        kP * (baseVel - velocity)
                                + kV * baseVel
                                + kS * Math.signum(baseVel);

                shooterLeft.setPower(-basePower);
                shooterRight.setPower(-basePower);
                break;

            case LUT:
                double lutPower =
                        kP * (lutVel - velocity)
                                + kV * lutVel
                                + kS * Math.signum(lutVel);

                shooterLeft.setPower(-lutPower);
                shooterRight.setPower(-lutPower);
                break;

            case OFF:
            default:
                shooterLeft.setPower(0);
                shooterRight.setPower(0);
                break;
        }

        telemetry.addData("Mode", mode);
        telemetry.addData("Robot X", robotX);
        telemetry.addData("Robot Y", robotY);
        telemetry.addData("Distance to Goal", distance);

        telemetry.addData("Base Model Velocity", baseVel);
        telemetry.addData("LUT Velocity", lutVel);
        telemetry.addData("Velocity Error (LUT - Base)", lutVel - baseVel);

        telemetry.addData("Actual Shooter Velocity", shooterLeft.getVelocity());

        telemetry.update();
    }
}

