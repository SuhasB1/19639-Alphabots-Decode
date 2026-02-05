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

public class Slow12 {

    private final Follower f;

    // You measured assuming 144, real field is 141.5

    private static final double X_OFFSET = 2.5;
    private static final double Y_OFFSET = 3;

    private static Pose p(double x, double y, double h) {
        return new Pose(x - X_OFFSET, y- Y_OFFSET, h);
    }

    private static Pose p(double x, double y) {
        return new Pose(x - X_OFFSET, y - Y_OFFSET);
    }

    // ================== POSES ==================

    public Pose start = new Pose(24 + 6.1 - 1.25, 141.5 - 10.5, Math.toRadians(90));
    public Pose scorePControl = p(55.593, 94.779);
    public Pose score = p(48, 96.0, Math.toRadians(133));

    public Pose intake1 = p(21.25, 85.5, Math.toRadians(180));
    public Pose intake1Control = p(50.000, 81.5);

    public Pose intake2 = p(14, 62.550, Math.toRadians(180));
    public Pose intake2Control = p(67.00 + X_OFFSET, 47.5 + Y_OFFSET);

    public Pose gate = p(16, 73.00, Math.toRadians(180));
    public Pose gateControl = p(30, 73);
    public Pose intake3 = p(14, 35.750, Math.toRadians(180));
    public Pose intake3Control = p(68.5, 20);

    public Pose score3Control = p(67.67, 54);
    public Pose score3 = p(58, 106, Math.toRadians(148.5));

    public Pose intakeCorner = p(6.6+X_OFFSET, 13, Math.toRadians(250));
    public Pose intakeCornerControl = p(6.7+X_OFFSET, 45);

    public Pose scoreToCorner = p(58, 106, Math.toRadians(148.5));
    public Pose scoreCorner = scoreToCorner;

    public Pose park = p(48, 72, Math.toRadians(180));

    public Pose CONTROL_POSE_gateFromIntake1 = p(30, 85);
    public Pose CONTROL_POSE_scoreFromGate1 = p(35, 90);
    public Pose CONTROL_POSE_parkFromScore3 = p(48, 84);
    public Pose CONTROL_POSE_toCornerFromScore2 = p(30, 50);

    // ================== MIRROR (UNCHANGED) ==================

    public Slow12(Robot r, Alliance a) {
        this.f = follower;

        if (a == Alliance.RED) {
            start = start.mirror();
            scorePControl = scorePControl.mirror();
            score = score.mirror();

            intake1 = intake1.mirror();
            intake1Control = intake1Control.mirror();

            intake2 = intake2.mirror();
            intake2Control = intake2Control.mirror();

            gate = gate.mirror();
            gateControl = gateControl.mirror();

            intake3 = intake3.mirror();
            intake3Control = intake3Control.mirror();

            score3Control = score3Control.mirror();
            score3 = score3.mirror();

            intakeCorner = intakeCorner.mirror();
            intakeCornerControl = intakeCornerControl.mirror();

            scoreToCorner = scoreToCorner.mirror();
            scoreCorner = scoreCorner.mirror();

            park = park.mirror();

            CONTROL_POSE_gateFromIntake1 = CONTROL_POSE_gateFromIntake1.mirror();
            CONTROL_POSE_scoreFromGate1 = CONTROL_POSE_scoreFromGate1.mirror();
            CONTROL_POSE_parkFromScore3 = CONTROL_POSE_parkFromScore3.mirror();
            CONTROL_POSE_toCornerFromScore2 = CONTROL_POSE_toCornerFromScore2.mirror();

            RobotConstants.currentTeam = Alliance.RED;
        } else {
            RobotConstants.currentTeam = Alliance.BLUE;
        }
    }

    // ================== PATHS (UNCHANGED) ==================

    public PathChain scoreP() {
        return f.pathBuilder()
                .addPath(new BezierCurve(start, scorePControl, score))
                .setBrakingStrength(1)
                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
                .build();
    }

    public PathChain intake1() {
        return f.pathBuilder()
                .addPath(new BezierCurve(score, intake1Control, intake1))
                .setBrakingStrength(1.5)
                .setLinearHeadingInterpolation(score.getHeading(), intake1.getHeading(), 0.3)
                .build();
    }

    public PathChain score1() {
        return f.pathBuilder()
                .addPath(new BezierLine(intake1, score))
                .setBrakingStrength(1)
                .setLinearHeadingInterpolation(intake1.getHeading(), score.getHeading())
                .build();
    }

    public PathChain gateFromIntake1() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake1, CONTROL_POSE_gateFromIntake1, gate))
                .setBrakingStrength(1.5)
                .setLinearHeadingInterpolation(intake1.getHeading(), gate.getHeading())
                .build();
    }

    public PathChain scoreFromGate1() {
        return f.pathBuilder()
                .addPath(new BezierCurve(gate, CONTROL_POSE_scoreFromGate1, score))
                .setBrakingStrength(1)
                .setLinearHeadingInterpolation(gate.getHeading(), score.getHeading())
                .build();
    }

    public PathChain intake2() {
        return f.pathBuilder()
                .addPath(new BezierCurve(score, intake2Control, intake2))
                .setBrakingStrength(1.5)
                .setConstantHeadingInterpolation(intake2.getHeading())
                .build();
    }

    public PathChain gate() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake2, gateControl, gate))
                .setBrakingStrength(1.5)
                .setLinearHeadingInterpolation(intake2.getHeading(), gate.getHeading())
                .build();
    }

    public PathChain score2() {
        return f.pathBuilder()
                .addPath(new BezierLine(gate, score))
                .setBrakingStrength(1)
                .setLinearHeadingInterpolation(gate.getHeading(), score.getHeading())
                .build();
    }

    public PathChain intake3() {
        return f.pathBuilder()
                .addPath(new BezierCurve(score, intake3Control, intake3))
                .setBrakingStrength(1.5)
                .setConstantHeadingInterpolation(intake3.getHeading())
                .build();
    }

    public PathChain score3() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake3, score3Control, score3))
                .setBrakingStrength(1)
                .setLinearHeadingInterpolation(intake3.getHeading(), score3.getHeading())
                .build();
    }

    public PathChain parkFromScore3() {
        return f.pathBuilder()
                .addPath(new BezierCurve(score3, CONTROL_POSE_parkFromScore3, park))
                .setBrakingStrength(1.5)
                .setLinearHeadingInterpolation(score3.getHeading(), park.getHeading())
                .build();
    }

    public PathChain toCornerFromScore2() {
        return f.pathBuilder()
                .addPath(new BezierCurve(score, CONTROL_POSE_toCornerFromScore2, intakeCorner))
                .setBrakingStrength(1.5)
                .setLinearHeadingInterpolation(score.getHeading(), intakeCorner.getHeading(), 0.3)
                .build();
    }

    public PathChain scoreCorner() {
        return f.pathBuilder()
                .addPath(new BezierLine(intakeCorner, scoreCorner))
                .setBrakingStrength(1)
                .setLinearHeadingInterpolation(intakeCorner.getHeading(), scoreCorner.getHeading(),0.4)
                .build();
    }
}