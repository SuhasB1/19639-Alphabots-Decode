package org.firstinspires.ftc.teamcode.config.shootAnywhere;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class ShooterCloseLUT {

    private final double val = -6.7;

    private final InterpolatedLUT2D shooterLUT = new InterpolatedLUT2D();

    public static double kVelocityCoupling = 1.0;

    public ShooterCloseLUT() {
        shooterLUT.addPoint(38, 60, new ShooterParams(1.0, 1125 + val)); // robot (44, 78) vs blue goal (6,138)
        shooterLUT.addPoint(38, 43, new ShooterParams(1.0, 1085+ val)); // robot (44, 95)
        shooterLUT.addPoint(38, 32, new ShooterParams(1.0, 1045+ val)); // robot (44,106)
        shooterLUT.addPoint(38, 17, new ShooterParams(1.0, 1020+ val)); // robot (44,121)
        shooterLUT.addPoint(38,  4, new ShooterParams(1.0, 1000+ val)); // robot (44,134)

        shooterLUT.addPoint(52, 60, new ShooterParams(1.0, 1180+ val)); // robot (58, 78)
        shooterLUT.addPoint(52, 43, new ShooterParams(1.0, 1145+ val)); // robot (58, 95)
        shooterLUT.addPoint(52, 32, new ShooterParams(1.0, 1115+ val)); // robot (58,106)
        shooterLUT.addPoint(52, 17, new ShooterParams(1.0, 1085+ val)); // robot (58,121)
        shooterLUT.addPoint(52,  4, new ShooterParams(1.0, 1035+ val)); // robot (58,134)

        shooterLUT.addPoint(66, 60, new ShooterParams(1.0, 1240+ val)); // robot (72, 78)
        shooterLUT.addPoint(66, 43, new ShooterParams(1.0, 1205+ val)); // robot (72, 95)
        shooterLUT.addPoint(66, 32, new ShooterParams(1.0, 1175+ val)); // robot (72,106)
        shooterLUT.addPoint(66, 17, new ShooterParams(1.0, 1145+ val)); // robot (72,121)
        shooterLUT.addPoint(66,  4, new ShooterParams(1.0, 1125+ val)); // robot (72,134)

        shooterLUT.addPoint(80, 60, new ShooterParams(1.0, 1330+ val)); // robot (86, 78)
        shooterLUT.addPoint(80, 43, new ShooterParams(1.0, 1285+ val)); // robot (86, 95)
        shooterLUT.addPoint(80, 32, new ShooterParams(1.0, 1265+ val)); // robot (86,106)
        shooterLUT.addPoint(80, 17, new ShooterParams(1.0, 1240+ val)); // robot (86,121)
        shooterLUT.addPoint(80,  4, new ShooterParams(1.0, 1220+ val)); // robot (86,134)

        shooterLUT.addPoint(94, 60, new ShooterParams(1.0, 1365+ val)); // robot (100, 78)
        shooterLUT.addPoint(94, 43, new ShooterParams(1.0, 1340+ val)); // robot (100, 95)
        shooterLUT.addPoint(94, 32, new ShooterParams(1.0, 1325+ val)); // robot (100,106)
        shooterLUT.addPoint(94, 17, new ShooterParams(1.0, 1310+ val)); // robot (100,121)
        shooterLUT.addPoint(94,  4, new ShooterParams(1.0, 1305+ val)); // robot (100,134)

        shooterLUT.addPoint(112, 60, new ShooterParams(1.0, 1500+ val)); // robot (118, 78)
        shooterLUT.addPoint(112, 43, new ShooterParams(1.0, 1480+ val)); // robot (118, 95)
        shooterLUT.addPoint(112, 32, new ShooterParams(1.0, 1440+ val)); // robot (118,106)
        shooterLUT.addPoint(112, 17, new ShooterParams(1.0, 1415+ val)); // robot (118,121)
        shooterLUT.addPoint(112,  4, new ShooterParams(1.0, 1420+ val)); // robot (118,134)






    }
    public ShooterParams getFromRobotPose(double robotX, double robotY) {
        double dx = Math.abs(robotX - RobotConstants.goalX);
        double dy = Math.abs(robotY - RobotConstants.goalY);
        return shooterLUT.get(dx, dy);
    }

    public ShooterParams get(double dx, double dy, double actualVelocity, boolean isShooting) {
        ShooterParams p = shooterLUT.get(dx, dy);
        return new ShooterParams(p.hoodPos, p.shooterVel);
    }
}
