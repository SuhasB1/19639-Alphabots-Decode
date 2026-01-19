package org.firstinspires.ftc.teamcode.config.shootAnywhere;

import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class ShooterFarLUT {

    private final InterpolatedLUT2D shooterLUT = new InterpolatedLUT2D();
    public static double kVelocityCoupling = 1.0;

    public ShooterFarLUT() {
        addAbsPoint(48,  10, 1495, 1.0);
        addAbsPoint(48, 20, 1430, 1.0);
        addAbsPoint(48, 30, 1370, 1.0);

        addAbsPoint( 71,  10, 1530, 1.0);
        addAbsPoint( 71, 20, 1480, 1.0);
        addAbsPoint( 71, 30, 1425, 1.0);

        addAbsPoint( 94,  10, 1575, 1.0);
        addAbsPoint( 94, 20, 1535, 1.0);
        addAbsPoint( 94, 30, 1495, 1.0);
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
        return new ShooterParams(lutParams.hoodPos, lutParams.shooterVel);
    }
}
