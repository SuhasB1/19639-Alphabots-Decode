package org.firstinspires.ftc.teamcode.config.shootAnywhere;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class ShooterCloseLUT {

    private final InterpolatedLUT2D shooterLUT = new InterpolatedLUT2D();

    public static double kVelocityCoupling = 1.0;

    public ShooterCloseLUT() {
        addAbsPoint(100,  78, 1375, 1.0);
        addAbsPoint(100, 106, 1300, 1.0);
        addAbsPoint(100, 134, 1250, 1.0);

        addAbsPoint( 72,  78, 1220, 1.0);
        addAbsPoint( 72, 106, 1180, 1.0);
        addAbsPoint( 72, 134, 1100, 1.0);

        addAbsPoint( 44,  78, 1100, 1.0);
        addAbsPoint( 44, 106, 1040, 1.0);
        addAbsPoint( 44, 134, 1000, 1.0);
    }

    private void addAbsPoint(double robotX, double robotY, double vel, double hood) {
        double goalX = RobotConstants.BLUE_CASTLE.getX();
        double goalY = RobotConstants.BLUE_CASTLE.getY();

        double x = robotX - goalX;
        double y = robotY - goalY;

        shooterLUT.addPoint(x, y, new ShooterParams(hood, vel));
    }

    public ShooterParams getFromRobotPose(double robotX, double robotY) {
        double goalX = RobotConstants.BLUE_CASTLE.getX();
        double goalY = RobotConstants.BLUE_CASTLE.getY();

        double x = robotX - goalX;
        double y = robotY - goalY;

        return shooterLUT.get(x, y);
    }

    public ShooterParams get(double x, double y,
                             double actualVelocity,
                             boolean isShooting) {

        ShooterParams lutParams = shooterLUT.get(x, y);

        double hood = lutParams.hoodPos;
        double vel  = lutParams.shooterVel;

        return new ShooterParams(hood, vel);
    }
}
