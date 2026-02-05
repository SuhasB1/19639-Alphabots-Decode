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

    private final Follower f;

    public Pose start = new Pose(53.850-1.25, 8.100, Math.toRadians(90));
    public Pose shoot = new Pose(58.000-1.25, 12.000, Math.toRadians(111.3));

    public Pose intakeHP1 = new Pose(16-1.25, 14.0-4, Math.toRadians(200));
    public Pose intakeHP2 = new Pose(16-1.25, 14.00-4, Math.toRadians(200));
    public Pose intakeHP3 = new Pose(16 - 1.25, 14.00-4, Math.toRadians(200));

    public Pose shootControlFromStart = new Pose(55.000-1.25, 13.500);
    public Pose intakeControlFromShoot = new Pose(34.300-1.25, 9.200);

    public Pose park = new Pose(40-1.25, 30, Math.toRadians(111.3));

    public double wiggleInches = 1.0;

    public FarCycles(Robot r, Alliance a) {
        this.f = follower;

        if (a != null && a.equals(Alliance.RED)) {
            start = start.mirror();
            shoot = shoot.mirror();

            intakeHP1 = intakeHP1.mirror();
            intakeHP2 = intakeHP2.mirror();
            intakeHP3 = intakeHP3.mirror();

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
                .addPath(new BezierCurve(start, shootControlFromStart, shoot))
                .setLinearHeadingInterpolation(start.getHeading(), shoot.getHeading())
                .build();
    }

    private PathChain shootToIntake(Pose intakePose) {
        return f.pathBuilder()
                .addPath(new BezierCurve(shoot, intakeControlFromShoot, intakePose))
                .setConstantHeadingInterpolation(intakePose.getHeading())
                .build();
    }

    public PathChain shootToIntakeHP1() { return shootToIntake(intakeHP1); }
    public PathChain shootToIntakeHP2() { return shootToIntake(intakeHP2); }
    public PathChain shootToIntakeHP3() { return shootToIntake(intakeHP3); }

    public PathChain intakeWiggle(Pose intakePose) {
        Pose left  = new Pose(intakePose.getX() - wiggleInches, intakePose.getY(), intakePose.getHeading());
        Pose right = new Pose(intakePose.getX() + wiggleInches, intakePose.getY(), intakePose.getHeading());

        return f.pathBuilder()
                .addPath(new BezierLine(intakePose, left))
                .setConstantHeadingInterpolation(intakePose.getHeading())
                .addPath(new BezierLine(left, right))
                .setConstantHeadingInterpolation(intakePose.getHeading())
                .addPath(new BezierLine(right, intakePose))
                .setConstantHeadingInterpolation(intakePose.getHeading())
                .build();
    }

    private PathChain intakeToShoot(Pose intakePose) {
        return f.pathBuilder()
                .addPath(new BezierLine(intakePose, shoot))
                .setLinearHeadingInterpolation(intakePose.getHeading(), shoot.getHeading())
                .build();
    }

    public PathChain intakeHP1ToShoot() { return intakeToShoot(intakeHP1); }
    public PathChain intakeHP2ToShoot() { return intakeToShoot(intakeHP2); }
    public PathChain intakeHP3ToShoot() { return intakeToShoot(intakeHP3); }

    public PathChain parkFromShoot() {
        return f.pathBuilder()
            .addPath(new BezierLine(shoot, park))
            .setLinearHeadingInterpolation(shoot.getHeading(), park.getHeading())
            .build();
    }
}
