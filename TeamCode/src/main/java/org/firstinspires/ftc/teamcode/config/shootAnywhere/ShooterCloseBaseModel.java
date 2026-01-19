package org.firstinspires.ftc.teamcode.config.shootAnywhere;

public class ShooterCloseBaseModel {

     // v(d) = b0 + b1*d + b2*d^2
//og: 931.615
    public static double b0 = 885.67;
    public static double b1 = 0.6611767;
    public static double b2 = 0.0297967;

    public static double velocityFromDistance(double d) {
        return b0 + b1 * d + b2 * d * d;
    }
    public static double velocitySlope(double d) {
        return b1 + 2.0 * b2 * d;
    }
}
