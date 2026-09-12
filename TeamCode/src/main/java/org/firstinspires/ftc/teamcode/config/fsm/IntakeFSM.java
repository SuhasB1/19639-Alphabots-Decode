package org.firstinspires.ftc.teamcode.config.fsm;

import org.firstinspires.ftc.teamcode.config.IntakeSubsystem;

public class IntakeFSM {

    public enum State {
        IDLE,
        INTAKING,
        TRANSFER,
        OUTTAKE
    }

    private final IntakeSubsystem intake;
    public State state = State.IDLE;

    public boolean auto = false;

    // About 1.2 seconds if loop runs around 50 Hz
    private static final int AUTO_WINDOW_SAMPLES = 60;
    private static final double AUTO_TRANSFER_FRAC = 0.70;

    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 0.8;
    private static final double OUTTAKE_POWER = -1.0;

    private final boolean[] autoWindow = new boolean[AUTO_WINDOW_SAMPLES];
    private int autoIdx = 0;
    private int autoFilled = 0;
    private int autoTrue = 0;

    public IntakeFSM(IntakeSubsystem intake) {
        this.intake = intake;
        intake.setRgbGreen();
    }

    public void startIntake() {
        state = State.INTAKING;
        intake.setPower(INTAKE_POWER);
        resetAutoWindow();
    }

    public void startTransfer() {
        state = State.TRANSFER;
        intake.setPower(TRANSFER_POWER);
        resetAutoWindow();
    }

    public void startOuttake() {
        state = State.OUTTAKE;
        intake.gateUp();
        intake.setPower(OUTTAKE_POWER);
        resetAutoWindow();
    }

    public void stop() {
        state = State.IDLE;
        intake.setPower(0.0);
        resetAutoWindow();
    }

    public void update() {
        intake.updateRgbFromSensors();

        if (state == State.INTAKING) {
            boolean sample = intake.ballDetected();
            pushAutoSample(sample);

            if (autoFilled >= AUTO_WINDOW_SAMPLES) {
                double frac = autoTrue / (double) AUTO_WINDOW_SAMPLES;

                if (frac >= AUTO_TRANSFER_FRAC) {
                    if (auto) {
                        stop();
                    } else {
                        startTransfer();
                    }
                }
            }
        }
    }

    public State getState() {
        return state;
    }

    private void resetAutoWindow() {
        for (int i = 0; i < AUTO_WINDOW_SAMPLES; i++) {
            autoWindow[i] = false;
        }

        autoIdx = 0;
        autoFilled = 0;
        autoTrue = 0;
    }

    private void pushAutoSample(boolean sample) {
        if (autoFilled >= AUTO_WINDOW_SAMPLES) {
            if (autoWindow[autoIdx]) autoTrue--;
        } else {
            autoFilled++;
        }

        autoWindow[autoIdx] = sample;
        if (sample) autoTrue++;

        autoIdx++;
        if (autoIdx >= AUTO_WINDOW_SAMPLES) autoIdx = 0;
    }
}