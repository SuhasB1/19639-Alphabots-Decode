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

    public boolean vibrate = false;

    private static final double BALL_THRESH_MM = 135.0;

    // ===== Slow “possession” filter (mainly for turning OFF / stability) =====
    private static final int POSSESSION_WINDOW_SAMPLES = 20;
    private static final double POSSESSION_ON_FRAC  = 0.70;
    private static final double POSSESSION_OFF_FRAC = 0.3;

    private static final boolean IGNORE_INVALID_SAMPLES = true;

    private static final int AUTO_WINDOW_SAMPLES = 20;
    private static final double AUTO_TRANSFER_FRAC = 0.70;

    private static final double INTAKE_POWER = 1.0;
    private static final double TRANSFER_POWER = 0.8;
    private static final double OUTTAKE_POWER = -1.0;

    // ===== Fast “turn ON now” filter =====
    // Tune this: 2–4 usually feels instant without flicker.
    private static final int FAST_ON_CONSECUTIVE = 4;
    private int fastOnCount = 0;

    private final boolean[] possessionWindow = new boolean[POSSESSION_WINDOW_SAMPLES];
    private int possIdx = 0, possFilled = 0, possTrue = 0;

    private final boolean[] autoWindow = new boolean[AUTO_WINDOW_SAMPLES];
    private int autoIdx = 0, autoFilled = 0, autoTrue = 0;

    private boolean hasThreeBalls = false;

    public IntakeFSM(IntakeSubsystem intake) {
        this.intake = intake;
        vibrate = false;
        intake.rgb.setPosition(0.5);
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

        // If you want transfer to imply "we have it", you can keep this,
        // but it can mask the sensor-based logic. Leaving as-is from your code:
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
        updateHasThreeBallsAndRgbFast();

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

    /**
     * Fast ON (few consecutive positives),
     * slow OFF (window/hysteresis).
     */
    private void updateHasThreeBallsAndRgbFast() {
        Boolean sample = readBottomBallSample();
        if (sample == null) return;

        // Always update the slow window (for stable OFF)
        pushPossessionSample(sample);

        // Fast ON: consecutive “present” samples
        if (sample) fastOnCount++;
        else fastOnCount = 0;

        // If we’re currently OFF, allow instant-ish ON
        if (!hasThreeBalls && fastOnCount >= FAST_ON_CONSECUTIVE) {
            setRgb(true);
            return;
        }

        // Slow OFF / hysteresis handling once window is filled
        if (possFilled < POSSESSION_WINDOW_SAMPLES) return;

        double frac = possTrue / (double) POSSESSION_WINDOW_SAMPLES;

        boolean newHasThree = hasThreeBalls;
        if (!hasThreeBalls && frac >= POSSESSION_ON_FRAC) newHasThree = true;
        else if (hasThreeBalls && frac <= POSSESSION_OFF_FRAC) newHasThree = false;

        setRgb(newHasThree);
    }

    private void setRgb(boolean on) {
        if (on == hasThreeBalls) return;

        hasThreeBalls = on;

        // IMPORTANT: set BOTH states, not just ON
        intake.rgb.setPosition(hasThreeBalls ? 0.3 : 0.5);

        // Reset fast counter when switching states to avoid weird re-triggers
        fastOnCount = 0;
    }

    private Boolean readBottomBallSample() {
        double distMm = intake.getDistance();
        boolean valid = Double.isFinite(distMm) && distMm > 0.0 && distMm < 5000.0;
        if (!valid && IGNORE_INVALID_SAMPLES) return null;
        return distMm < BALL_THRESH_MM;
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