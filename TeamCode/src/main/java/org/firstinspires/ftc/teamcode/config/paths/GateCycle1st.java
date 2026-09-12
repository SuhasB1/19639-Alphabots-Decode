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

public class GateCycle1st {

    private final Follower f;

    public static double SCORE_DONE_T = 0.95;
    public static double INTAKE_DONE_T = 0.95;

    public Pose start = new Pose(24 + 6.1 - 1.25, 141.5 - 10.5, Math.toRadians(90));
//40,95,129.5
    public Pose scorePControl = new Pose(55.593 - 2.5, 94.779 - 2.5);
    public Pose score = new Pose(48.6, 92.6, Math.toRadians(134.75));

    public Pose intake1 = new Pose(21.25 - 2.5, 85.5 - 2.5, Math.toRadians(180));
    public Pose intake1Control = new Pose(51.25 - 2.5, 81.5 - 2.5);

    public Pose intake2 = new Pose(14 - 2.5, 60.550 - 1.75, Math.toRadians(180));
    public Pose intake2Control = new Pose(69.5 - 2.5, 50 - 2.5);

    public Pose gate = new Pose(16.5 - 1, 71.35, Math.toRadians(180));
    public Pose gateControl = new Pose(20+5, 73 - 10);
    public Pose intake3 = new Pose(14 - 2.5, 37.750 - 2.5, Math.toRadians(180));
    public Pose intake3Control = new Pose(68.5 - 2.5, 20 - 2.5);

    public Pose score3Control = new Pose(67.67 - 2.5, 54 - 2.5);
    public Pose score3 = new Pose(48 - 2.5, 96.0 - 2.5, Math.toRadians(132.5));
    public Pose gateIntake = new Pose(10, 58.35, Math.toRadians(160));
    public Pose gateIntakeThird = new Pose(10, 59, Math.toRadians(159.5));
    public Pose gateIntakeControlFromScore = new Pose(70, 75.0);
    public Pose scoreControlFromGateIntake = new Pose(70, 75.0);
    public Pose parkScore = new Pose(55.5,103.5, Math.toRadians(145));
    public Pose parkScoreControlFromIntake3 = new Pose(67.67-2.5,54-2.5);

    public Pose CONTROL_POSE_gateFromIntake1 = new Pose(30-2.5, 85-3);
    public Pose CONTROL_POSE_scoreFromGate1 = new Pose(35-2.5, 90-3);


    public GateCycle1st(Robot r, Alliance a) {
        this.f = follower;

        if (a != null && a.equals(Alliance.RED)) {
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

            gateIntake = gateIntake.mirror();
            gateIntake = new Pose(gateIntake.getX(), gateIntake.getY() + 1.25, gateIntake.getHeading());
            gateIntakeThird = gateIntakeThird.mirror();
            gateIntakeThird = new Pose(gateIntake.getX(), gateIntake.getY() + 1.25 - 0.65, gateIntake.getHeading());


            gateIntakeControlFromScore = gateIntakeControlFromScore.mirror();
            scoreControlFromGateIntake = scoreControlFromGateIntake.mirror();

            parkScore = parkScore.mirror();
            parkScoreControlFromIntake3 = parkScoreControlFromIntake3.mirror();

            RobotConstants.currentTeam = Alliance.RED;
        } else {
            RobotConstants.currentTeam = Alliance.BLUE;
        }
    }

    public PathChain scorePreload() {
        return f.pathBuilder()
                .addPath(new BezierCurve(start, scorePControl, score))
                .setBrakingStrength(1)
                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
                .build();
    }

    public PathChain toIntake1() {
        return f.pathBuilder()
                .addPath(new BezierCurve(score, intake1Control, intake1))
                .setBrakingStrength(1)
                .setLinearHeadingInterpolation(score.getHeading(), intake1.getHeading(), 0.3)
                .build();
    }

    public PathChain intake1ToScore() {
        return f.pathBuilder()
                .addPath(new BezierLine(intake1, parkScore))
                .setBrakingStrength(1.0)
                .setLinearHeadingInterpolation(intake1.getHeading(), parkScore.getHeading())
                .build();
    }

    public PathChain toIntake2() {
        return f.pathBuilder()
                .addPath(new BezierCurve(score, intake2Control, intake2))
                .setBrakingStrength(1)
                .setConstantHeadingInterpolation(intake2.getHeading())
                .build();
    }

    public PathChain intake2ToGate() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake2, gateControl, gate))
                .setNoDeceleration()
                .setLinearHeadingInterpolation(intake2.getHeading(), gate.getHeading())
                .build();
    }

    public PathChain intake2ToScore() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake2, scoreControlFromGateIntake, score))
                .setNoDeceleration()
                .setLinearHeadingInterpolation(intake2.getHeading(), score.getHeading())
                .build();
    }

    public PathChain gateToScore() {
        return f.pathBuilder()
                .addPath(new BezierLine(gate, score3))
                .setBrakingStrength(1.0)
                .setBrakingStart(0.25)
                .setLinearHeadingInterpolation(gate.getHeading(), score3.getHeading())
                .build();
    }

    public PathChain scoreToGateIntake() {
        return f.pathBuilder()
                .addPath(new BezierCurve(score, gateIntakeControlFromScore, gateIntake))
                .setBrakingStrength(1)
                .setLinearHeadingInterpolation(score.getHeading(), gateIntake.getHeading(), 0.3)
                .build();
    }
    public PathChain scoreToGateIntakeThird() {
        return f.pathBuilder()
                .addPath(new BezierCurve(score, gateIntakeControlFromScore, gateIntakeThird))
                .setBrakingStrength(1)
                .setLinearHeadingInterpolation(score.getHeading(), gateIntakeThird.getHeading(), 0.3)
                .build();
    }

    public PathChain gateIntakeToScore() {
        return f.pathBuilder()
                .addPath(new BezierCurve(gateIntake, scoreControlFromGateIntake, score3))
                .setBrakingStrength(1)
                .setLinearHeadingInterpolation(gateIntake.getHeading(), score3.getHeading(), 0.3)
                .build();
    }

    public PathChain gateIntakeToScorePark() {
        return f.pathBuilder()
                .addPath(new BezierCurve(gateIntake, scoreControlFromGateIntake, parkScore))
                .setBrakingStrength(1)
                .setLinearHeadingInterpolation(gateIntake.getHeading(), parkScore.getHeading(), 0.3)
                .build();
    }
    public PathChain gateIntakeThirdToScorePark() {
        return f.pathBuilder()
                .addPath(new BezierCurve(gateIntakeThird, scoreControlFromGateIntake, parkScore))
                .setBrakingStrength(1)
                .setLinearHeadingInterpolation(gateIntakeThird.getHeading(), parkScore.getHeading(), 0.3)
                .build();
    }

    public PathChain toIntake3() {
        return f.pathBuilder()
                .addPath(new BezierCurve(score, intake3Control, intake3))
                .setBrakingStrength(1)
                .setConstantHeadingInterpolation(intake3.getHeading())
                .build();
    }

    public PathChain intake3ToParkScore() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake3, parkScoreControlFromIntake3, parkScore))
                .setBrakingStrength(1)
                .setLinearHeadingInterpolation(intake3.getHeading(), parkScore.getHeading(), 0.3)
                .build();
    }

    public PathChain gateFromIntake1() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake1, CONTROL_POSE_gateFromIntake1, gate))
                .setBrakingStrength(1)
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
}