package org.firstinspires.ftc.teamcode.config.paths;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class FarCycles {
    double wiggle = 12.0;


    private final Follower f;

    public Pose start = new Pose(53.850-2.2, 6.6, Math.toRadians(90));

    public Pose shootStart = new Pose(58.000-2.2, 12.000 - 1.5, Math.toRadians(112));
    public Pose shoot = new Pose(58.000-2.2, 12.000 - 1.5, Math.toRadians(112));

    public Pose intake3rdRow = new Pose(11.5-2.2, 37-1.5, Math.toRadians(180));
    public Pose intake3rdRowControl = new Pose(65.2-2.2, 45-1.5);

    public Pose intakeHP1 = new Pose(16.5-5, 14.0 - 4.5, Math.toRadians(200));
    public Pose intakeHP2 = new Pose(16.5-5, 14.00 - 1.5- (wiggle/2), Math.toRadians(180));
    public Pose intakeHP3 = new Pose(16.5-5, 14.00 - 1.5- (wiggle/2), Math.toRadians(180));
    public Pose intakeHP4 = new Pose(16.5-5, 14.00-4.5, Math.toRadians(180));

    public Pose shootControlFromStart = new Pose(55.000-2.2, 13.500-1.5);
    public Pose intakeControlFromShoot = new Pose(34.300-2.2, 9.200-1.5);

    public Pose park = new Pose(34.5, 10, Math.toRadians(90));

    public FarCycles(Robot r, Alliance a) {
        this.f = follower;

        if (a != null && a.equals(Alliance.RED)) {
            start = start.mirror();
            shootStart.mirror();
            shoot = shoot.mirror();

            intakeHP1 = intakeHP1.mirror();
            intakeHP2 = intakeHP2.mirror();
            intakeHP3 = intakeHP3.mirror();
            intakeHP4 = intakeHP4.mirror();

            shootControlFromStart = shootControlFromStart.mirror();
            intakeControlFromShoot = intakeControlFromShoot.mirror();

            park = park.mirror();

            RobotConstants.currentTeam = Alliance.RED;
        } else {
            RobotConstants.currentTeam = Alliance.BLUE;
        }
    }

    public PathChain startToShoot() {
        return f.pathBuilder()
                .addPath(new BezierCurve(start, shootControlFromStart, shootStart))
                .setLinearHeadingInterpolation(start.getHeading(), shootStart.getHeading())
                .build();
    }
    public PathChain shootToIntake3rd() {
        return f.pathBuilder()
                .addPath(new BezierCurve(shoot, intake3rdRowControl, intake3rdRow))
                .setLinearHeadingInterpolation(shootStart.getHeading(), intake3rdRow.getHeading(), 0.2)
                .build();
    }

    public PathChain intake3rdToShoot() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake3rdRow, intake3rdRowControl, shoot))
                .setLinearHeadingInterpolation(intake3rdRow.getHeading(), shoot.getHeading(),0.4)
                .build();
    }

    private PathChain shootToIntakeHP(Pose intakePose) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shoot, intakeControlFromShoot, intakePose))
                .setConstantHeadingInterpolation(intakePose.getHeading())
                .build();
    }

    public PathChain shootToIntakeHP1() { return shootToIntakeHP(intakeHP1); }
    public PathChain shootToIntakeHP2() { return shootToIntakeHP(intakeHP2); }
    public PathChain shootToIntakeHP3() { return shootToIntakeHP(intakeHP3); }
    public PathChain shootToIntakeHP4() { return shootToIntakeHP(intakeHP4); }

    public PathChain intakeWiggle(Pose intakePose) {

        Pose left = new Pose(
                intakePose.getX(),
                intakePose.getY(),
                intakePose.getHeading()
        );

        Pose right = new Pose(
                intakePose.getX()-3.5,
                intakePose.getY() + wiggle,
                intakePose.getHeading()
        );

        Pose control = new Pose(
                intakePose.getX() - 1,
                intakePose.getY()
        );

        return f.pathBuilder()
                .addPath(new BezierCurve(left, control, right))
                .setConstantHeadingInterpolation(intakePose.getHeading())
                .addPath(new BezierCurve(right, control, left))
                .setConstantHeadingInterpolation(intakePose.getHeading())
                .build();
    }
    private PathChain intakeToShoot(Pose intakePose, double shootHeading) {
        return f.pathBuilder()
                .addPath(new BezierLine(intakePose, shoot))
                .setLinearHeadingInterpolation(intakePose.getHeading(), shootHeading)
                .build();
    }

    public PathChain intakeHP1ToShoot() { return intakeToShoot(intakeHP1, shoot.getHeading()); }
    public PathChain intakeHP2ToShoot() { return intakeToShoot(intakeHP2, shoot.getHeading()); }
    public PathChain intakeHP3ToShoot() { return intakeToShoot(intakeHP3, shoot.getHeading()); }
    public PathChain intakeHP4ToShoot() { return intakeToShoot(intakeHP4, shoot.getHeading()); }


    public PathChain parkFromShoot() {
        return f.pathBuilder()
            .addPath(new BezierLine(shoot, park))
            .setLinearHeadingInterpolation(shoot.getHeading(), park.getHeading())
            .build();
    }
}
