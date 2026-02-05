package org.firstinspires.ftc.teamcode.config.shootAnywhere;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class ShooterCloseBaseModel {

    public static final double a = 0.00633009031;
    public static final double b = 4.88793585;
    public static final double c = 786.062238;

    public static double velocityFromDxDy(double dx, double dy) {
        double d = Math.hypot(dx, dy);
        return a * d * d + b * d + c;
    }

    public static double velocityFromRobotPose(double robotX, double robotY) {
        double dx = Math.abs(robotX - RobotConstants.goalX);
        double dy = Math.abs(robotY - RobotConstants.goalY);
        return velocityFromDxDy(dx, dy);
    }

    public static double velocityFromDistance(double d) {
        return a * d * d + b * d + c;
    }
}
