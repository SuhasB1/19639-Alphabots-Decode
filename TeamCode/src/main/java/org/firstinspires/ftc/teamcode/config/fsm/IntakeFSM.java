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

    private static final double BALL_THRESH_MM = 135.0;

    private static final int POSSESSION_WINDOW_SAMPLES = 35;
    private static final double POSSESSION_ON_FRAC  = 0.60;
    private static final double POSSESSION_OFF_FRAC = 0.35;

    private static final boolean IGNORE_INVALID_SAMPLES = true;

    private static final int AUTO_WINDOW_SAMPLES = 10;
    private static final double AUTO_TRANSFER_FRAC = 0.60;

    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 0.6;
    private static final double OUTTAKE_POWER = -1.0;

    private final boolean[] possessionWindow = new boolean[POSSESSION_WINDOW_SAMPLES];
    private int possIdx = 0, possFilled = 0, possTrue = 0;

    private final boolean[] autoWindow = new boolean[AUTO_WINDOW_SAMPLES];
    private int autoIdx = 0, autoFilled = 0, autoTrue = 0;

    private boolean hasThreeBalls = false;

    public IntakeFSM(IntakeSubsystem intake) {
        this.intake = intake;
        intake.rgb.setPosition(0.5);
    }

    public void startIntake() {
        state = State.INTAKING;
        intake.gateUp();
        intake.setPower(INTAKE_POWER);
        resetAutoWindow();
    }

    public void startTransfer() {
        state = State.TRANSFER;
        intake.setPower(TRANSFER_POWER);
        resetAutoWindow();

        // FAST RGB: if we’re transferring because we detected possession, turn RGB on immediately
        setRgb(true);
    }

    public void startOuttake() {
        state = State.OUTTAKE;
        intake.gateUp();
        intake.setPower(OUTTAKE_POWER);
        resetAutoWindow();

        setRgb(false);
    }

    public void stop() {
        state = State.IDLE;
        intake.setPower(0.0);
        resetAutoWindow();
    }

    public void update() {
        updateHasThreeBallsAndRgbSlow();

        if (state == State.INTAKING) {
            Boolean sample = readBottomBallSample();
            if (sample != null) pushAutoSample(sample);

            if (autoFilled >= AUTO_WINDOW_SAMPLES) {
                double frac = autoTrue / (double) AUTO_WINDOW_SAMPLES;
                if (frac >= AUTO_TRANSFER_FRAC) {
                    startTransfer();
                }
            }
        }
    }

    public State getState() {
        return state;
    }

    private void updateHasThreeBallsAndRgbSlow() {
        Boolean sample = readBottomBallSample();
        if (sample == null) return;

        pushPossessionSample(sample);

        if (possFilled < POSSESSION_WINDOW_SAMPLES) return;

        double frac = possTrue / (double) POSSESSION_WINDOW_SAMPLES;

        boolean newHasThree = hasThreeBalls;
        if (!hasThreeBalls && frac >= POSSESSION_ON_FRAC) newHasThree = true;
        else if (hasThreeBalls && frac <= POSSESSION_OFF_FRAC) newHasThree = false;

        setRgb(newHasThree);
    }

    private void setRgb(boolean on) {
        if (on != hasThreeBalls) {
            hasThreeBalls = on;
            intake.rgb.setPosition(hasThreeBalls ? 0.3 : 0.5);
        }
    }

    private Boolean readBottomBallSample() {
        double distMm = intake.getDistance();
        boolean valid = Double.isFinite(distMm) && distMm > 0.0 && distMm < 5000.0;
        if (!valid && IGNORE_INVALID_SAMPLES) return null;
        return valid && (distMm < BALL_THRESH_MM);
    }

    private void pushPossessionSample(boolean sample) {
        if (possFilled >= POSSESSION_WINDOW_SAMPLES) {
            if (possessionWindow[possIdx]) possTrue--;
        } else {
            possFilled++;
        }

        possessionWindow[possIdx] = sample;
        if (sample) possTrue++;

        possIdx++;
        if (possIdx >= POSSESSION_WINDOW_SAMPLES) possIdx = 0;
    }

    private void resetAutoWindow() {
        for (int i = 0; i < AUTO_WINDOW_SAMPLES; i++) autoWindow[i] = false;
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
