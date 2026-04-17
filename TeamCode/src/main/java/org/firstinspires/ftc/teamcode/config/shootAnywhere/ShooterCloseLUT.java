package org.firstinspires.ftc.teamcode.config.shootAnywhere;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
@Config
public class ShooterCloseLUT {

    public static double val = 3.67;

    private final InterpolatedLUT2D shooterLUT = new InterpolatedLUT2D();

    public ShooterCloseLUT() {
        shooterLUT.addPoint(38, 60, new ShooterParams(1.0, 1015 + val)); // robot (44, 78) vs blue goal (6,138)
        shooterLUT.addPoint(38, 43, new ShooterParams(1.0, 975+ val)); // robot (44, 95)
        shooterLUT.addPoint(38, 32, new ShooterParams(1.0, 935+ val)); // robot (44,106)
        shooterLUT.addPoint(38, 17, new ShooterParams(1.0, 890+ val)); // robot (44,121)
        shooterLUT.addPoint(38,  4, new ShooterParams(1.0, 875+ val)); // robot (44,134)

        shooterLUT.addPoint(52, 60, new ShooterParams(1.0, 1070+ val)); // robot (58, 78)
        shooterLUT.addPoint(52, 43, new ShooterParams(1.0, 1015+ val)); // robot (58, 95)
        shooterLUT.addPoint(52, 32, new ShooterParams(1.0, 995+ val)); // robot (58,106)
        shooterLUT.addPoint(52, 17, new ShooterParams(1.0, 975+ val)); // robot (58,121)
        shooterLUT.addPoint(52,  4, new ShooterParams(1.0, 930+ val)); // robot (58,134)

        shooterLUT.addPoint(66, 60, new ShooterParams(1.0, 1130+ val)); // robot (72, 78)
        shooterLUT.addPoint(66, 43, new ShooterParams(1.0, 1095+ val)); // robot (72, 95)
        shooterLUT.addPoint(66, 32, new ShooterParams(1.0, 1065+ val)); // robot (72,106)
        shooterLUT.addPoint(66, 17, new ShooterParams(1.0, 1035+ val)); // robot (72,121)
        shooterLUT.addPoint(66,  4, new ShooterParams(1.0, 1015+ val)); // robot (72,134)

        shooterLUT.addPoint(80, 60, new ShooterParams(1.0, 1210+ val)); // robot (86, 78)
        shooterLUT.addPoint(80, 43, new ShooterParams(1.0, 1165+ val)); // robot (86, 95)
        shooterLUT.addPoint(80, 32, new ShooterParams(1.0, 1145+ val)); // robot (86,106)
        shooterLUT.addPoint(80, 17, new ShooterParams(1.0, 1120+ val)); // robot (86,121)
        shooterLUT.addPoint(80,  4, new ShooterParams(1.0, 1100+ val)); // robot (86,134)

        shooterLUT.addPoint(94, 60, new ShooterParams(1.0, 1240+ val)); // robot (100, 78)
        shooterLUT.addPoint(94, 43, new ShooterParams(1.0, 1215+ val)); // robot (100, 95)
        shooterLUT.addPoint(94, 32, new ShooterParams(1.0, 1200+ val)); // robot (100,106)
        shooterLUT.addPoint(94, 17, new ShooterParams(1.0, 1185+ val)); // robot (100,121)
        shooterLUT.addPoint(94,  4, new ShooterParams(1.0, 1180+ val)); // robot (100,134)

        shooterLUT.addPoint(112, 60, new ShooterParams(1.0, 1360+ val)); // robot (118, 78)
        shooterLUT.addPoint(112, 43, new ShooterParams(1.0, 1340+ val)); // robot (118, 95)
        shooterLUT.addPoint(112, 32, new ShooterParams(1.0, 1300+ val)); // robot (118,106)
        shooterLUT.addPoint(112, 17, new ShooterParams(1.0, 1275+ val)); // robot (118,121)
        shooterLUT.addPoint(112,  4, new ShooterParams(1.0, 1280+ val)); // robot (118,134)






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
