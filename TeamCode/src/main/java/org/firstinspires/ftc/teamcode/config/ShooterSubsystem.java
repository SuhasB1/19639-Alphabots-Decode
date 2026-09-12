package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.shootAnywhere.ShooterCloseLUT;
import org.firstinspires.ftc.teamcode.config.shootAnywhere.ShooterFarLUT;
import org.firstinspires.ftc.teamcode.config.shootAnywhere.ShooterParams;

@Config
public class ShooterSubsystem {

    public enum TargetMode { BASE_MODEL, LUT }

    private final DcMotorEx left, right;
    private final Telemetry telemetry;

    private final ShooterCloseLUT closeLUT = new ShooterCloseLUT();
    private final ShooterFarLUT farLUT = new ShooterFarLUT();
    private ShooterParams params;

    public static double kP = 0.01;
    public static double kD = 0;
    public static double kV = 0.0003525;
    public static double kS = 0.06;

    public static boolean ENABLE_IDLE_REV = true;

    private double targetVelocity = 0.0;
    private double measuredVelocity = 0.0;

    private TargetMode mode = TargetMode.LUT;

    private boolean manualOverride = false;
    private double manualVel = 0.0;

    private final PIDFController velPID = new PIDFController(new PIDFCoefficients(kP, 0.0, kD, 0.0));

    private static final double C_X1 = -19.0909, C_Y1 = 130.0;
    private static final double C_X2 = 72.0,     C_Y2 = 50.0;
    private static final double C_X3 = 165.0909, C_Y3 = 130.0;

    private static final double F_X1 = 7.0,  F_Y1 = -5.0;
    private static final double F_X2 = 80.0,  F_Y2 = 55.0;
    private static final double F_X3 = 140.0, F_Y3 = -5.0;

    private static class TriCheck {
        final boolean inside;
        final double u, v;
        TriCheck(boolean inside, double u, double v) { this.inside = inside; this.u = u; this.v = v; }
    }

    private double lastX = 0.0, lastY = 0.0;
    private TriCheck lastClose = new TriCheck(false, 0.0, 0.0);
    private TriCheck lastFar   = new TriCheck(false, 0.0, 0.0);
    private boolean lastInClose = false;
    private boolean lastInFar = false;
    private boolean lastInAny = false;

    public ShooterSubsystem(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;

        left = hw.get(DcMotorEx.class, "shooterLeft");
        right = hw.get(DcMotorEx.class, "shooterRight");
        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void init() {
        stop();
        velPID.updateError(0.0);
    }

    public void setMode(TargetMode m) { mode = m; }
    public TargetMode getMode() { return mode; }

    public double getMeasuredVelocity() { return measuredVelocity; }
    public double getTargetVelocity() { return targetVelocity; }

    public void stop() {
        left.setPower(0.0);
        right.setPower(0.0);
        targetVelocity = 0.0;
        velPID.updateError(0.0);
    }

    public void setGoal(double x, double y) {
        RobotConstants.goalX = x;
        RobotConstants.goalY = y;
    }

    public void setManualVelocityOverride(double v) {
        manualOverride = true;
        manualVel = v;
    }

    public void clearManualVelocityOverride() {
        manualOverride = false;
    }

    public void idleRev(double robotX, double robotY) {
        if (!ENABLE_IDLE_REV) return;

        lastX = robotX;
        lastY = robotY;

        lastClose = pointInTriangleDebug(robotX, robotY, C_X1, C_Y1, C_X2, C_Y2, C_X3, C_Y3);
        lastFar   = pointInTriangleDebug(robotX, robotY, F_X1, F_Y1, F_X2, F_Y2, F_X3, F_Y3);

        lastInClose = lastClose.inside;
        lastInFar   = lastFar.inside;
        lastInAny   = lastInClose || lastInFar;

        if (lastInAny) updateVelocityControl(robotX, robotY);
        else stop();
    }

    public boolean isInCloseShootingZone(double x, double y) {
        return pointInTriangleDebug(x, y, C_X1, C_Y1, C_X2, C_Y2, C_X3, C_Y3).inside;
    }

    public boolean isInFarShootingZone(double x, double y) {
        return pointInTriangleDebug(x, y, F_X1, F_Y1, F_X2, F_Y2, F_X3, F_Y3).inside;
    }

    public boolean isInAnyShootingZone(double x, double y) {
        return isInCloseShootingZone(x, y) || isInFarShootingZone(x, y);
    }

    private static TriCheck pointInTriangleDebug(
            double px, double py,
            double ax, double ay,
            double bx, double by,
            double cx, double cy
    ) {
        double v0x = cx - ax, v0y = cy - ay;
        double v1x = bx - ax, v1y = by - ay;
        double v2x = px - ax, v2y = py - ay;

        double dot00 = v0x * v0x + v0y * v0y;
        double dot01 = v0x * v1x + v0y * v1y;
        double dot02 = v0x * v2x + v0y * v2y;
        double dot11 = v1x * v1x + v1y * v1y;
        double dot12 = v1x * v2x + v1y * v2y;

        double denom = (dot00 * dot11 - dot01 * dot01);
        if (Math.abs(denom) < 1e-9) return new TriCheck(false, 0.0, 0.0);

        double invDenom = 1.0 / denom;
        double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

        boolean inside = (u >= 0.0) && (v >= 0.0) && (u + v <= 1.0);
        return new TriCheck(inside, u, v);
    }

    public void updateVelocityControl(double robotX, double robotY) {
        measuredVelocity = -right.getVelocity();

        if (robotY < 50) params = farLUT.getFromRobotPose(robotX, robotY);
        else params = closeLUT.getFromRobotPose(robotX, robotY);

        targetVelocity = manualOverride ? manualVel : params.shooterVel;

        velPID.setCoefficients(new PIDFCoefficients(kP, 0.0, kD, 0.0));

        double error = targetVelocity - measuredVelocity;
        velPID.updateError(error);
        double uFb = velPID.run();

        double uFf = kV * targetVelocity + kS * Math.signum(targetVelocity);

        double power = uFb + uFf;
        power = Math.max(-1.0, Math.min(1.0, power));

        left.setPower(-power);
        right.setPower(-power);
    }

    public boolean atSpeed(double tolerance) {
        return Math.abs(targetVelocity - measuredVelocity) <= tolerance;
    }

    public void telemetry() {
        telemetry.addData("ShooterMode", mode);
        telemetry.addData("TargetVel", targetVelocity);
        telemetry.addData("MeasuredVel", measuredVelocity);
        telemetry.addData("RPM", measuredVelocity / 28.0 * 60.0);
        telemetry.addData("ManualOverride", manualOverride);
        telemetry.addData("IdleRevEnabled", ENABLE_IDLE_REV);

        telemetry.addData("ZoneX", lastX);
        telemetry.addData("ZoneY", lastY);

        telemetry.addData("InClose", lastInClose);
        telemetry.addData("Close_u", lastClose.u);
        telemetry.addData("Close_v", lastClose.v);
        telemetry.addData("Close_u+v", lastClose.u + lastClose.v);

        telemetry.addData("InFar", lastInFar);
        telemetry.addData("Far_u", lastFar.u);
        telemetry.addData("Far_v", lastFar.v);
        telemetry.addData("Far_u+v", lastFar.u + lastFar.v);

        telemetry.addData("InAny", lastInAny);
    }
}
