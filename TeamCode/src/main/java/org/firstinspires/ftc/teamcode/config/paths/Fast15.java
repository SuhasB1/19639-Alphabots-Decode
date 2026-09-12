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

public class Fast15 {

    private final Follower f;

    public Pose start          = new Pose(24 + 6.1 - 1.25, 141.5 - 10.5, Math.toRadians(90));
    public Pose scorePControl  = new Pose(55.593-2.5, 94.779-2.5);
    public Pose score          = new Pose(48-2.5, 96.0-2.5, Math.toRadians(133));
    public Pose intake1 = new Pose(21.25-2.5,85.5-2.5, Math.toRadians(180));
    public Pose intake1Control = new Pose(51.25-2.5, 81.5-2.5);
    public Pose intake2 = new Pose(14-2.5, 60.550-2.5, Math.toRadians(180));
    public Pose intake2Control = new Pose(69.5-2.5, 50-2.5);

    public Pose gate = new Pose(16.5-2.5, 73.00-2.5, Math.toRadians(180));
    public Pose gateControl = new Pose(30-2.5, 73-2.5);
    public Pose intake3 = new Pose(14-2.5, 37.750-2.5, Math.toRadians(180));
    public Pose intake3Control = new Pose(68.5-2.5, 20-2.5);
    public Pose score3Control = new Pose(67.67-2.5,54-2.5);
    public Pose score3          = new Pose(48-2.5, 96.0-2.5, Math.toRadians(132.67));
    public Pose intakeCorner = new Pose(6.6,13-2.5, Math.toRadians(250));
    public Pose intakeCornerControl = new Pose(6.7,45-2.5);
    public Pose scoreCorner = new Pose(58-2.5,106-2.5, Math.toRadians(148.5));
    public Fast15(Robot r, Alliance a) {
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

            intakeCorner = intakeCorner.mirror();
            intakeCornerControl = intakeCornerControl.mirror();

            scoreCorner = scoreCorner.mirror();

            RobotConstants.currentTeam = Alliance.RED;
        }else{
            RobotConstants.currentTeam = Alliance.BLUE;
        }
    }

    public PathChain scoreP() {
        return f.pathBuilder()
                .addPath(new BezierCurve(start, scorePControl, score))
                .setBrakingStrength(1.5)
                .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
                .build();
    }

    public PathChain intake1() {
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                score,
                                intake1Control,
                                intake1
                        )
                )
                .setBrakingStrength(1.5)
                .setLinearHeadingInterpolation(score.getHeading(), intake1.getHeading(), 0.3)
                .build();
    }

    public PathChain score1() {
        return f.pathBuilder()
                .addPath(new BezierLine(intake1, score))
                .setBrakingStrength(1)
                .setLinearHeadingInterpolation(gate.getHeading(), score.getHeading())
                .build();
    }

    public PathChain intake2() {
        return f.pathBuilder()
                .addPath(
                        new BezierCurve(
                                score,
                                intake2Control,
                                intake2
                        )
                )
                .setBrakingStrength(1.5)
                .setConstantHeadingInterpolation(intake2.getHeading())
                .build();
    }

    public PathChain score2() {
        return f.pathBuilder()
                .addPath(new BezierLine(gate, score))
                .setBrakingStrength(1)
                .setLinearHeadingInterpolation(gate.getHeading(), score.getHeading())
                .build();
    }

    public PathChain gate() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake2, gateControl, gate))
                .setNoDeceleration()
                .setLinearHeadingInterpolation(intake2.getHeading(), gate.getHeading())
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
    public PathChain intakeCorner() {
        return f.pathBuilder()
                .addPath(new BezierCurve(score3, intakeCornerControl, intakeCorner))
                .setBrakingStrength(1.5)
                .setLinearHeadingInterpolation(score3.getHeading(),intakeCorner.getHeading(),0.3)
                .build();
    }
/*
    public PathChain intakeCornerSpin() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intakeCorner, intakeCornerSpin))
                .setLinearHeadingInterpolation(intakeCorner.getHeading(),intakeCornerSpin.getHeading())
                .build();
    }

 */

    public PathChain scoreCorner() {
        return f.pathBuilder()
                .addPath(new BezierLine(intakeCorner, scoreCorner))
                .setNoDeceleration()
                .setLinearHeadingInterpolation(intakeCorner.getHeading(), scoreCorner.getHeading(),0.4)
                .build();
    }
}