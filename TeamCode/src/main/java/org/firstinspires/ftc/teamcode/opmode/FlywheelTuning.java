package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Config
@TeleOp(name = "Flywheel Tuning", group = "Tuning")
public class FlywheelTuning extends OpMode {

    private DcMotorEx motor, motor2,intake;
    private Servo hood, gate;

    public static double targetVelocity = 0;
    public static double intakePower = 0;
    public static double hoodPosition = 0;

    public static double gatePosition = 0;
    public double kP = 0.001;
    public double kV = 0.00034;
    public double kS = 0.086;


    private double velocity;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        motor2 = hardwareMap.get(DcMotorEx.class, "shooterRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        hood = hardwareMap.get(Servo.class, "hood");
        gate = hardwareMap.get(Servo.class, "gate");

        hood.setDirection(Servo.Direction.REVERSE);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0));

    }
    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void loop() {
        velocity = -motor.getVelocity();

        // Feedforward + proportional feedback
        double power =
                kP * (targetVelocity - velocity)
                        + kV * targetVelocity
                        + kS * Math.signum(targetVelocity);

        motor.setPower(-power);
        motor2.setPower(-power);

        follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        follower.update();

        telemetry.addData("x:", follower.getPose().getX());
        telemetry.addData("y:", follower.getPose().getY());
        telemetry.addData("heading:", follower.getPose().getHeading());
        telemetry.addData("total heading:", follower.getTotalHeading());
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Measured Velocity", velocity);
        telemetry.addData("Flywheel Power", power);
        telemetry.addData("Flywheel RPM", velocity / 28 * 60);
        telemetry.addData("kP", kP);
        telemetry.addData("kV", kV);
        telemetry.addData("kS", kS);
        telemetry.update();

        intake.setPower(intakePower);

        hood.setPosition(hoodPosition);
        gate.setPosition(gatePosition);
    }

    /*
kS stands for "static." It's the greatest power you can give the flywheel motor(s) without it moving.
So it should be a really small number below like 0.05 or something.
You should increase it until it begins to move and then decrease it until it stops moving again.

kV is the next number you tune.
Pick a velocity like 1500 or something and then increase kV until it gets there.

Tune P. Send a ball through the launcher and increase P until it doesn't overshoot past the target
velocity in the recovery phase. Keep increasing though because unless you see overshoot you're fine.

Tune I. After all the other numbers are tuned, save them, and then make kS slightly negative.
This will simulate a drop in power to the motor from a low battery.
Increase I until your confident that you can still get to your target velocity.
     */
}
