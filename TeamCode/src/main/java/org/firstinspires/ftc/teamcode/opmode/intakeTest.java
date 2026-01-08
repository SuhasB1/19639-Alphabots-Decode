package org.firstinspires.ftc.teamcode.opmode;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.*;
@Config
@TeleOp(name = "intake test")
public class intakeTest extends OpMode {
    public IntakeSubsystem intake;
    public ShooterSubsystem shooter;
    public intakeTest(HardwareMap hardwareMap, Telemetry telemetry) {
        intake = new IntakeSubsystem(hardwareMap, telemetry, IntakeSubsystem.IntakeState.IDLE, IntakeSubsystem.GateState.OPEN);
        shooter = new ShooterSubsystem(hardwareMap, telemetry);
    }
    @Override
    public void init() {
        intake.idle();
        intake.open();
    }

    @Override
    public void loop() {
        if(gamepad1.left_bumper) {
            intake.intake();
        }
        if(gamepad1.right_bumper){
            intake.outtake();
        }
        if(gamepad1.square){
            intake.switchGateState();
        }
        if(gamepad1.triangle){
            shooter.hoodLeft.setPosition(RobotConstants.hoodMin);
            shooter.hoodRight.setPosition(RobotConstants.hoodMin);
        }
    }


}
