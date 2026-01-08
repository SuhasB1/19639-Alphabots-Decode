package org.firstinspires.ftc.teamcode.config;
import static org.firstinspires.ftc.teamcode.config.RobotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {
    public enum IntakeState {
        INTAKE, IDLE, OUTTAKE
    }
    public enum GateState {
        OPEN, CLOSE
    }
    public DcMotorEx intake;

    public Servo gate;
    public IntakeState intakeState;
    public GateState gateState;
    public Timer autoTimer = new Timer();
    private Telemetry telemetry;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry, IntakeState intakeState, GateState gateState) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        gate = hardwareMap.get(Servo.class, "gate");
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        this.telemetry = telemetry;
        this.intakeState = intakeState;
        this.gateState = gateState;
    }

    public void setIntakeState(IntakeState intakeState) {
        if (intakeState == IntakeState.IDLE) {
            intake.setPower(intakeIdle);; // Explicitly referencing constant
            this.intakeState = IntakeState.IDLE;
        } else if (intakeState == IntakeState.INTAKE) {
            intake.setPower(intakeIntake); // Explicitly referencing constant
            this.intakeState = IntakeState.INTAKE;
        } else if (intakeState == IntakeState.OUTTAKE) {
            intake.setPower(intakeOuttake);
            this.intakeState = IntakeState.OUTTAKE;
        }
    }
    public void switchIntakeState() {
        if (intakeState == IntakeState.INTAKE) {
            setIntakeState(IntakeState.IDLE);
        } else if (intakeState == IntakeState.IDLE || intakeState == IntakeState.OUTTAKE) {
            setIntakeState(IntakeState.INTAKE);
        }
    }
    public void switchOuttakeState() {
        if (intakeState == IntakeState.OUTTAKE) {
            setIntakeState(IntakeState.IDLE);
        } else if (intakeState == IntakeState.INTAKE || intakeState == IntakeState.IDLE) {
            setIntakeState(IntakeState.OUTTAKE);
        }
    }
    public void setGateState(GateState gateState) {
        if (gateState == GateState.OPEN) {
            gate.setPosition(gateOpen);// Explicitly referencing constant
            this.gateState = GateState.OPEN;
        } else if (gateState == GateState.CLOSE) {
            gate.setPosition(gateClose);// Explicitly referencing constant
            this.gateState = GateState.CLOSE;
        }
    }
    public void switchGateState() {
        if (gateState == GateState.OPEN) {
            setGateState(GateState.CLOSE);
        } else if (gateState == GateState.CLOSE) {
            setGateState(GateState.OPEN);
        }
    }
    public void intake() {
        setIntakeState(IntakeState.INTAKE);
    }
    public void outtake(){
        setIntakeState(IntakeState.OUTTAKE);
    }
    public void idle() {
        setIntakeState(IntakeState.IDLE);
    }
    public void open(){
        setGateState(GateState.OPEN);
    }
    public void close(){
        setGateState(GateState.CLOSE);
    }

    public void init() {
        setIntakeState(IntakeState.IDLE);
        setGateState(GateState.OPEN);
    }

    public void start() {
        autoTimer.resetTimer();
    }

    public void telemetry() {
        telemetry.addData("Intake State: ", intakeState);
        telemetry.addData("Gate State: ", gateState);

    }
}
