package org.firstinspires.ftc.teamcode.config.shootAnywhere;

public class ShooterFarBaseModel {

    // v(d) = a*d^2 + b*d + c

    public static final double a = -0.04718;
    public static final double b =  7.42;
    public static final double c =  1348.6;

    public static double velocityFromDistance(double d) {
        return a * d * d + b * d + c;
    }

    public static double velocitySlope(double d) {
        return 2.0 * a * d + b;
    }
}
