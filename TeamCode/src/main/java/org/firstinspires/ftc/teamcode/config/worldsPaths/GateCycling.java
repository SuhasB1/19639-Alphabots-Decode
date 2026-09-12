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

public class GateCycling {

    private final Follower f;

    public Pose start = new Pose(28.925, 131.0, Math.toRadians(90));
    public Pose score = new Pose(54, 83, Math.toRadians(133));

    public Pose intake1 = new Pose(13, 83.25, Math.toRadians(180));
    public Pose intake1Control = new Pose(53.0, 82.5);

    public Pose intake2 = new Pose(6.7, 59.8, Math.toRadians(180));
    public Pose intake2Control = new Pose(60.0, 59);
    public Pose scoreControlFromIntake2 = new Pose(40.0, 70.0);

    public Pose secondSetGateHit = new Pose(15.1, 69, Math.toRadians(180));
    public Pose secondSetGateHitControl = new Pose(18, 56);
    public Pose scoreControlFromSecondSetGateHit = new Pose(40, 60);

    public Pose gateIntake = new Pose(8.67, 59.34, Math.toRadians(151));
    public Pose gateIntakeControl = new Pose(30, 59.67);
    public Pose scoreControlFromGate = new Pose(30.0, 64);

    public Pose parkScore = new Pose(55.5, 103.5, Math.toRadians(145));

    public GateCycling(Robot r, Alliance a) {
        this.f = follower;

        if (a != null && a.equals(Alliance.RED)) {
            start = start.mirror();
            start = new Pose(start.getX() + (145.25 - 141.5)-1, start.getY() - 0.5, start.getHeading());
            score = score.mirror();

            intake1 = intake1.mirror();
            intake1Control = intake1Control.mirror();

            intake2 = intake2.mirror();
            intake2Control = intake2Control.mirror();
            scoreControlFromIntake2 = scoreControlFromIntake2.mirror();

            secondSetGateHit = secondSetGateHit.mirror();
            secondSetGateHitControl = secondSetGateHitControl.mirror();
            secondSetGateHit = new Pose(secondSetGateHit.getX()+1.4, secondSetGateHit.getY(), secondSetGateHit.getHeading());
            scoreControlFromSecondSetGateHit = scoreControlFromSecondSetGateHit.mirror();

            gateIntake = gateIntake.mirror();
            gateIntake = new Pose(gateIntake.getX(), gateIntake.getY()-1.49, gateIntake.getHeading());
            gateIntakeControl = gateIntakeControl.mirror();
            scoreControlFromGate = scoreControlFromGate.mirror();

            parkScore = parkScore.mirror();

            RobotConstants.currentTeam = Alliance.RED;
        } else {
            RobotConstants.currentTeam = Alliance.BLUE;
        }
    }

    // ===== PRELOAD =====
    public PathChain scorePreload() {
        return f.pathBuilder()
                .addPath(new BezierLine(start, score))
                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
                .build();
    }

    // ===== INTAKE 1 =====
    public PathChain toIntake1() {
        return f.pathBuilder()
                .addPath(new BezierCurve(score, intake1Control, intake1))
                .setConstantHeadingInterpolation(intake1.getHeading())
                .build();
    }

    public PathChain intake1ToScore() {
        return f.pathBuilder()
                .addPath(new BezierLine(intake1, score))
                .setLinearHeadingInterpolation(intake1.getHeading(), score.getHeading())
                .build();
    }

    // ===== INTAKE 2 =====
    public PathChain toIntake2() {
        return f.pathBuilder()
                .addPath(new BezierCurve(score, intake2Control, intake2))
                .setConstantHeadingInterpolation(intake2.getHeading())
                .build();
    }

    public PathChain intake2ToGateHit() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake2, secondSetGateHitControl, secondSetGateHit))
                .setLinearHeadingInterpolation(intake2.getHeading(), secondSetGateHit.getHeading())
                .build();
    }

    public PathChain gateHitToScore() {
        return f.pathBuilder()
                .addPath(new BezierCurve(secondSetGateHit, scoreControlFromSecondSetGateHit, score))
                .setLinearHeadingInterpolation(secondSetGateHit.getHeading(), score.getHeading())
                .build();
    }

    public PathChain intake2ToScore() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake2, intake2Control, score))
                .setLinearHeadingInterpolation(intake2.getHeading(), score.getHeading())
                .build();
    }

    // ===== GATE CYCLE =====
    public PathChain scoreToGateIntake(double yOffset) {
        Pose g = new Pose(gateIntake.getX(), gateIntake.getY() - yOffset, gateIntake.getHeading());
        Pose c = new Pose(gateIntakeControl.getX(), gateIntakeControl.getY() - yOffset);

        return f.pathBuilder()
                .addPath(new BezierCurve(score, c, g))
                .setLinearHeadingInterpolation(score.getHeading(), g.getHeading())
                .build();
    }

    public PathChain gateIntakeToScore(double yOffset) {
        Pose g = new Pose(gateIntake.getX(), gateIntake.getY() + yOffset, gateIntake.getHeading());
        Pose c = new Pose(scoreControlFromGate.getX(), scoreControlFromGate.getY() + yOffset);

        return f.pathBuilder()
                .addPath(new BezierCurve(g, c, score))
                .setLinearHeadingInterpolation(g.getHeading(), score.getHeading())
                .build();
    }

    public PathChain gateIntakeToParkScore(double yOffset) {
        Pose g = new Pose(gateIntake.getX(), gateIntake.getY() + yOffset, gateIntake.getHeading());

        return f.pathBuilder()
                .addPath(new BezierLine(g, parkScore))
                .setLinearHeadingInterpolation(g.getHeading(), parkScore.getHeading())
                .build();
    }

    public PathChain intake1ToParkScore() {
        return f.pathBuilder()
                .addPath(new BezierLine(intake1, parkScore))
                .setLinearHeadingInterpolation(intake1.getHeading(), parkScore.getHeading())
                .build();
    }
}