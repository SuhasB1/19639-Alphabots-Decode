package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static java.lang.Math.atan2;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.config.DriveSubsystem;
import org.firstinspires.ftc.teamcode.config.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.config.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.config.fsm.IntakeFSM;
import org.firstinspires.ftc.teamcode.config.fsm.ShootFSM;

public class Robot {

    public final DriveSubsystem drive;
    public final ShooterSubsystem shooter;
    public final IntakeSubsystem intake;

    public final IntakeFSM intakeFSM;
    public final ShootFSM shootFSM;

    private final Telemetry telemetry;

    // private final Allience allience;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Alliance a) {
        this.telemetry = telemetry;

        drive   = new DriveSubsystem(hardwareMap, telemetry);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
        intake  = new IntakeSubsystem(hardwareMap, telemetry);

        intakeFSM = new IntakeFSM(intake);
        shootFSM  = new ShootFSM(drive, shooter, intake);

        if (a != null && a.equals(Alliance.RED)) {
            RobotConstants.goalX = RobotConstants.RED_CASTLE.getX();
            RobotConstants.goalY = RobotConstants.RED_CASTLE.getY();
        }else{
            RobotConstants.goalX = RobotConstants.BLUE_CASTLE.getX();
            RobotConstants.goalY = RobotConstants.BLUE_CASTLE.getY();
        }
    }

    public void init() {
        drive.init();
        shooter.init();
        intake.init();
    }

    public void start() {
        drive.start();
    }

    public void update(Gamepad gamepad1, Gamepad gamepad2, double x, double y) {
        intakeFSM.update();
        shootFSM.update(x,y);
    }

    public void telemetry(){
        telemetry.addData("IntakeFSM", intakeFSM.getState());
        telemetry.addData("ShootFSM", shootFSM.getState());
        telemetry.addData("Robot X", drive.getX());
        telemetry.addData("Robot Y", drive.getY());
        telemetry.addData("Robot Heading", Math.toDegrees(drive.getHeading()));
        telemetry.addData("heading from goal", Math.toDegrees(atan2( RobotConstants.BLUE_CASTLE.getY() - drive.getY(), RobotConstants.BLUE_CASTLE.getX() - drive.getX())));
        telemetry.addData("Shooter Target Vel", shooter.getTargetVelocity());
        telemetry.addData("Shooter Measured Vel", shooter.getMeasuredVelocity());
        intake.telemetry();
        telemetry.update();
    }
}
