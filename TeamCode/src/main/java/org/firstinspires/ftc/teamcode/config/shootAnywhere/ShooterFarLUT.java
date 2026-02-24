package org.firstinspires.ftc.teamcode.config.shootAnywhere;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
@Config
public class ShooterFarLUT {

    public static double val = 0;

    private final InterpolatedLUT2D shooterLUT = new InterpolatedLUT2D();
    public ShooterFarLUT() {
        shooterLUT.addPoint(38, 128, new ShooterParams(1.0, 1405+ val)); // robot (44, 10) vs blue goal (6,138)
        shooterLUT.addPoint(38, 118, new ShooterParams(1.0, 1325+ val)); // robot (44, 20)
        shooterLUT.addPoint(38, 108, new ShooterParams(1.0, 1295+ val)); // robot (44, 30)

        shooterLUT.addPoint(52, 128, new ShooterParams(1.0, 1415+ val)); // robot (58, 10)
        shooterLUT.addPoint(52, 118, new ShooterParams(1.0, 1360+ val)); // robot (58, 20)
        shooterLUT.addPoint(52, 108, new ShooterParams(1.0, 1315+ val)); // robot (58, 30)

        shooterLUT.addPoint(65, 128, new ShooterParams(1.0, 1470+ val)); // robot (71, 10)
        shooterLUT.addPoint(65, 118, new ShooterParams(1.0, 1425+ val)); // robot (71, 20)
        shooterLUT.addPoint(65, 108, new ShooterParams(1.0, 1380+ val)); // robot (71, 30)

        shooterLUT.addPoint(77, 128, new ShooterParams(1.0, 1530+ val)); // robot (83, 10)
        shooterLUT.addPoint(77, 118, new ShooterParams(1.0, 1470+ val)); // robot (83, 20)
        shooterLUT.addPoint(77, 108, new ShooterParams(1.0, 1420+ val)); // robot (83, 30)

        shooterLUT.addPoint(88, 128, new ShooterParams(1.0, 1545+ val)); // robot (94, 10)
        shooterLUT.addPoint(88, 118, new ShooterParams(1.0, 1525+ val)); // robot (94, 20)
        shooterLUT.addPoint(88, 108, new ShooterParams(1.0, 1475+ val)); // robot (94, 30)
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
