package org.firstinspires.ftc.teamcode.config.worldsPaths;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class FarPaths {

    private final Follower f;

    public static double WIGGLE_BACK = 0;

    public Pose start = new Pose(51.65, 8.4, Math.toRadians(90));

    public Pose shootStart = new Pose(55.8, 10.5, Math.toRadians(118.5));
    public Pose shoot = new Pose(55.8, 10.5, Math.toRadians(112));

    public Pose shootControl = new Pose((55.8/2), (10.5*1.35));

    public Pose intake3rdRow = new Pose(6, 36.2, Math.toRadians(180));
    public Pose intake3rdRowControl = new Pose(60.0, 40);

    public Pose intakeHP1 = new Pose(8, 8.2, Math.toRadians(180));
    public Pose intakeHP2 = new Pose(8, 8.2, Math.toRadians(180));
    public Pose intakeHP3 = new Pose(8, 18, Math.toRadians(180));
    public Pose intakeHP4 = new Pose(8, 12, Math.toRadians(180));
    public Pose intakeHP5 = new Pose(8, 8.5, Math.toRadians(180));
    public Pose intakeHP6 = new Pose(8, 8.2, Math.toRadians(180));

    public Pose shootControlFromStart = new Pose(52.8, 12.0);
    public Pose intakeControlFromShoot = new Pose(32.1, 12);

    public Pose park = new Pose(34.5, 10.0, Math.toRadians(90));

    public FarPaths(Robot r, Alliance a) {
        this.f = follower;

        if (a != null && a.equals(Alliance.RED)) {
            start = start.mirror();
            start = new Pose(start.getX()-0.5, start.getY(), start.getHeading());

            shootStart = shootStart.mirror();
            shoot = shoot.mirror();
            shoot = new Pose(shoot.getX(), shoot.getY() + 1.5 + (145.25 - 141.5), shoot.getHeading());
            shootControl = shootControl.mirror();
            shootControl = new Pose(shootControl.getX(), shootControl.getY()+ 1.5 + (145.25 - 141.5), shootControl.getHeading());


            intake3rdRow = intake3rdRow.mirror();
            intake3rdRow = new Pose(intake3rdRow.getX(), intake3rdRow.getY(), intake3rdRow.getHeading());
            intake3rdRowControl = intake3rdRowControl.mirror();
            intake3rdRowControl = new Pose(intake3rdRowControl.getX(), intake3rdRowControl.getY(), intake3rdRowControl.getHeading());


            intakeHP1 = intakeHP1.mirror();
            intakeHP1 = new Pose(intakeHP1.getX(), intakeHP1.getY() + 1, intakeHP1.getHeading());
            intakeHP2 = intakeHP2.mirror();
            intakeHP2 = new Pose(intakeHP2.getX(), intakeHP2.getY() + 0.7, intakeHP2.getHeading());
            intakeHP3 = intakeHP3.mirror();
            intakeHP4 = intakeHP4.mirror();
            intakeHP5 = intakeHP5.mirror();
            intakeHP6 = intakeHP6.mirror();

            shootControlFromStart = shootControlFromStart.mirror();

            intakeControlFromShoot = intakeControlFromShoot.mirror();
            intakeControlFromShoot = new Pose(intakeControlFromShoot.getX(), intakeControlFromShoot.getY() + 1 + (145.25 - 141.5), intakeControlFromShoot.getHeading());

            park = park.mirror();
            park = new Pose(park.getX() + 1 + (145.25 - 141.5), park.getY(), park.getHeading());


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
                .setLinearHeadingInterpolation(intake3rdRow.getHeading(), shoot.getHeading(), 0.4)
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
    public PathChain shootToIntakeHP5() { return shootToIntakeHP(intakeHP5); }
    public PathChain shootToIntakeHP6() { return shootToIntakeHP(intakeHP6); }

    public PathChain intakeHP1ToShoot() { return intakeToShoot(intakeHP1); }
    public PathChain intakeHP2ToShoot() { return intakeToShoot(intakeHP2); }
    public PathChain intakeHP3ToShoot() { return intakeToShoot(intakeHP3); }
    public PathChain intakeHP4ToShoot() { return intakeToShoot(intakeHP4); }
    public PathChain intakeHP5ToShoot() { return intakeToShoot(intakeHP5); }
    public PathChain intakeHP6ToShoot() { return intakeToShoot(intakeHP6); }

    public PathChain intakeHP1Wiggle() { return intakeWiggle(intakeHP1); }
    public PathChain intakeHP2Wiggle() { return intakeWiggle(intakeHP2); }
    public PathChain intakeHP3Wiggle() { return intakeWiggle(intakeHP3); }
    public PathChain intakeHP4Wiggle() { return intakeWiggle(intakeHP4); }
    public PathChain intakeHP5Wiggle() { return intakeWiggle(intakeHP5); }
    public PathChain intakeHP6Wiggle() { return intakeWiggle(intakeHP6); }

    // HP pose is forward-most → go back → return to HP
    public PathChain intakeWiggle(Pose intakePose) {
        Pose back = moveForward(intakePose, -WIGGLE_BACK);

        return f.pathBuilder()
                .addPath(new BezierLine(intakePose, back))
                .setConstantHeadingInterpolation(intakePose.getHeading())

                .addPath(new BezierLine(back, intakePose))
                .setConstantHeadingInterpolation(intakePose.getHeading())

                .build();
    }

    private Pose moveForward(Pose pose, double distance) {
        double x = pose.getX() + distance * Math.cos(pose.getHeading());
        double y = pose.getY() + distance * Math.sin(pose.getHeading());
        return new Pose(x, y, pose.getHeading());
    }

    private PathChain intakeToShoot(Pose intakePose) {
        return f.pathBuilder()
                .addPath(new BezierCurve(intakePose, shootControl, shoot))
                .setLinearHeadingInterpolation(intakePose.getHeading(), shoot.getHeading())
                .build();
    }

    public PathChain parkFromShoot() {
        return f.pathBuilder()
                .addPath(new BezierLine(shoot, park))
                .setLinearHeadingInterpolation(shoot.getHeading(), park.getHeading())
                .build();
    }
}