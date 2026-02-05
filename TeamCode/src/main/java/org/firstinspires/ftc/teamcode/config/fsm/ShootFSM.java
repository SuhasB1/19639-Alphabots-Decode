package org.firstinspires.ftc.teamcode.config.fsm;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.DriveSubsystem;
import org.firstinspires.ftc.teamcode.config.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.config.ShooterSubsystem;

public class ShootFSM {

    public enum State {
        IDLE,
        SPINUP,
        DONE
    }

    private final DriveSubsystem drive;
    private final ShooterSubsystem shooter;
    private final IntakeSubsystem intake;

    private double transferPower = 0.6;

    private double minSpinupTime = 0.1;
    private double doneHoldTime = 0;

    private final ElapsedTime timer = new ElapsedTime();
    private State state = State.IDLE;

    public ShootFSM(DriveSubsystem drive, ShooterSubsystem shooter, IntakeSubsystem intake) {
        this.drive = drive;
        this.shooter = shooter;
        this.intake = intake;
    }

    public void start() {
        timer.reset();
        state = State.SPINUP;

        RobotConstants.actionBusy = true;
        intake.gateUp();
    }

    public void stop() {
        state = State.IDLE;

        shooter.stop();
        intake.setPower(0.0);
        RobotConstants.actionBusy = false;
    }

    public void update(double x, double y) {
        switch (state) {
            case IDLE:
                break;

            case SPINUP:
                intake.setPower(transferPower);
                shooter.updateVelocityControl(x, y);

                if (timer.seconds() > minSpinupTime && shooter.atSpeed(20)) {
                    intake.gateDown();
                    timer.reset();
                    state = State.DONE;
                }
                break;

            case DONE:
                shooter.updateVelocityControl(x, y);

                if (y < 50) doneHoldTime = 1.3;
                else doneHoldTime = 0.85;

                if (timer.seconds() > doneHoldTime) {
                    stop();
                }
                break;
        }
    }

    public State getState() { return state; }
}