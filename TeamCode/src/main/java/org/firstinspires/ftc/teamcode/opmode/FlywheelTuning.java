package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static java.lang.Math.atan2;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.config.DriveSubsystem;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.shootAnywhere.ShooterCloseBaseModel;
import org.firstinspires.ftc.teamcode.config.shootAnywhere.ShooterCloseLUT;
import org.firstinspires.ftc.teamcode.config.shootAnywhere.ShooterParams;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Config
@TeleOp(name = "Flywheel Tuning", group = "Tuning")
public class FlywheelTuning extends OpMode {
    public PathChain Path1;
    public PathChain Path2;
    private DcMotorEx motor, motor2, intake;
    private Servo hood, gate;

    public static double targetVelocity = 0;
    public static double intakePower = 0.6;
    public static double hoodPosition = 1;
    public static double gatePosition = 0.2;

    public static double kP = 0.0125;
    public static double kV = 0.0003215;
    public static double kS = 0.09;

    public static double xPos = 44;
    public static double yPos  = 134;

    public static boolean autoPath = false;
    public static boolean autoShoot = false;

    public static boolean far = false;

    private static final double GOAL_X = 6.0;
    private static final double GOAL_Y = 138.0;

    public static double speed  = 0.7;

    private double velocity;
    private boolean headingLock = false;
    private ShooterCloseLUT closeLUT;
    private DriveSubsystem drive;

    boolean prevRT = false;

    private enum Mode {
        OFF,
        BASE_MODEL,
        LUT
    }

    private Mode mode = Mode.OFF;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        motor2 = hardwareMap.get(DcMotorEx.class, "shooterRight");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        hood = hardwareMap.get(Servo.class, "hood");
        gate = hardwareMap.get(Servo.class, "gate");
        drive = new DriveSubsystem(hardwareMap, telemetry);


        hood.setDirection(Servo.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(6.1,8.1));
        closeLUT = new ShooterCloseLUT();
        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(6.100, 8.100),
                                new Pose(72.000, 24.000),
                                new Pose(xPos, yPos)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), atan2(RobotConstants.BLUE_CASTLE.getY() - yPos, RobotConstants.BLUE_CASTLE.getX() - xPos))
                .build();
        Path1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(6.100, 8.100),
                                new Pose(10.00, 20.00),
                                new Pose(xPos, yPos)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), atan2(RobotConstants.BLUE_CASTLE.getY() - yPos, RobotConstants.BLUE_CASTLE.getX() - xPos) - Math.toRadians(1.5))
                .build();
        telemetry.setMsTransmissionInterval(50);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    @Override
    public void loop() {
        boolean rtHeld = gamepad1.right_trigger > 0.1;
        velocity = -motor.getVelocity();

        if (rtHeld && !prevRT) {
            drive.resetHeadingPID();
        }
        prevRT = rtHeld;

        if (rtHeld) {
            drive.headingLockPID(gamepad1, GOAL_X, GOAL_Y);
        } else {
            drive.updateTeleop(gamepad1);
        }
        if(gamepad1.right_bumper && autoPath && far){
            follower.followPath(Path1);
        }
        if(gamepad1.right_bumper && autoPath && !far){
            follower.followPath(Path2);
        }
        if(gamepad1.right_trigger > 0.1){
            speed = 0.3;
        }
        if(autoShoot){
            autoShoot();
        }else{
            manualShoot();
        }
        follower.update();

        telemetry.addData("x:", follower.getPose().getX());
        telemetry.addData("y:", follower.getPose().getY());
        telemetry.addData("heading:", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("heading from goal", Math.toDegrees(atan2( RobotConstants.BLUE_CASTLE.getY() - follower.getPose().getY(), RobotConstants.BLUE_CASTLE.getX() - follower.getPose().getX())));
        telemetry.addData("Target Velocity", targetVelocity);
        telemetry.addData("Measured Velocity", velocity);
        telemetry.addData("Flywheel RPM", velocity / 28 * 60);
        telemetry.addData("Shooter 1 Current", motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Shooter 2 Current", motor2.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
        if(gamepad1.dpad_up){
            intake.setPower(intakePower);
        }
        if(gamepad1.dpad_down){
            intake.setPower(0);
        }
        hood.setPosition(hoodPosition);
        if(gamepad1.dpad_left) {
            gate.setPosition(gatePosition);
        }
        if(gamepad1.dpad_right) {
            gate.setPosition(0);
        }
    }

    public void manualShoot(){
        double power =
                kP * (targetVelocity - velocity)
                        + kV * velocity
                        + kS * Math.signum(velocity);

        motor.setPower(-power);
        motor2.setPower(-power);
    }
    public void autoShoot(){
        double robotX = follower.getPose().getX();
        double robotY = follower.getPose().getY();

        double dx = robotX - GOAL_X;
        double dy = robotY - GOAL_Y;
        double distance = Math.hypot(dx, dy);

        // Base model velocity
        double baseVel = ShooterCloseBaseModel.velocityFromDistance(distance);

        // LUT velocity
        ShooterParams lutParams = closeLUT.getFromRobotPose(robotX, robotY);
        double lutVel = lutParams.shooterVel;

        // Button control
        if (gamepad1.square) mode = Mode.BASE_MODEL;
        if (gamepad1.circle) mode = Mode.LUT;
        if (gamepad1.cross) mode = Mode.OFF;

        // Apply shooter command
        switch (mode) {
            case BASE_MODEL:
                double basePower =
                        kP * (baseVel - velocity)
                                + kV * baseVel
                                + kS * Math.signum(baseVel);

                motor.setPower(-basePower);
                motor2.setPower(-basePower);
                break;

            case LUT:
                double lutPower =
                        kP * (lutVel - velocity)
                                + kV * lutVel
                                + kS * Math.signum(lutVel);

                motor.setPower(-lutPower);
                motor2.setPower(-lutPower);
                break;

            case OFF:
            default:
                motor.setPower(0);
                motor2.setPower(0);
                break;
        }
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
