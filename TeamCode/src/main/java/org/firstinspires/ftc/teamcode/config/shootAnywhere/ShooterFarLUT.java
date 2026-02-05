package org.firstinspires.ftc.teamcode.config.shootAnywhere;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class ShooterFarLUT {

    private final double val = 41.67;

    private final InterpolatedLUT2D shooterLUT = new InterpolatedLUT2D();
    public static double kVelocityCoupling = 1.0;

    public ShooterFarLUT() {
        shooterLUT.addPoint(38, 128, new ShooterParams(1.0, 1505+ val)); // robot (44, 10) vs blue goal (6,138)
        shooterLUT.addPoint(38, 118, new ShooterParams(1.0, 1430+ val)); // robot (44, 20)
        shooterLUT.addPoint(38, 108, new ShooterParams(1.0, 1390+ val)); // robot (44, 30)

        shooterLUT.addPoint(52, 128, new ShooterParams(1.0, 1510+ val)); // robot (58, 10)
        shooterLUT.addPoint(52, 118, new ShooterParams(1.0, 1460+ val)); // robot (58, 20)
        shooterLUT.addPoint(52, 108, new ShooterParams(1.0, 1410+ val)); // robot (58, 30)

        shooterLUT.addPoint(65, 128, new ShooterParams(1.0, 1553+ val)); // robot (71, 10)
        shooterLUT.addPoint(65, 118, new ShooterParams(1.0, 1503+ val)); // robot (71, 20)
        shooterLUT.addPoint(65, 108, new ShooterParams(1.0, 1453+ val)); // robot (71, 30)

        shooterLUT.addPoint(77, 128, new ShooterParams(1.0, 1590+ val)); // robot (83, 10)
        shooterLUT.addPoint(77, 118, new ShooterParams(1.0, 1550+ val)); // robot (83, 20)
        shooterLUT.addPoint(77, 108, new ShooterParams(1.0, 1500+ val)); // robot (83, 30)

        shooterLUT.addPoint(88, 128, new ShooterParams(1.0, 1630+ val)); // robot (94, 10)
        shooterLUT.addPoint(88, 118, new ShooterParams(1.0, 16100+ val)); // robot (94, 20)
        shooterLUT.addPoint(88, 108, new ShooterParams(1.0, 1560+ val)); // robot (94, 30)
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
