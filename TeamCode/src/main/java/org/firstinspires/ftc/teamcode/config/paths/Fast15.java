package org.firstinspires.ftc.teamcode.config.paths;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.Robot;
import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.RobotConstants;

public class Fast15 {

    private final Follower f;

    public Pose start          = new Pose(24 + 6.1, 144 - 9.3, Math.toRadians(90));
    public Pose scorePControl  = new Pose(55.593, 94.779);
    public Pose score          = new Pose(48, 96.0, Math.toRadians(133));
    public Pose intake1 = new Pose(19,83.5, Math.toRadians(180));
    public Pose intake1Control = new Pose(50.000, 87.5);
    public Pose intake2 = new Pose(11, 60.050, Math.toRadians(180));
    public Pose intake2Control = new Pose(65.400, 65);
    public Pose gate = new Pose(15.75, 72.500, Math.toRadians(180));
    public Pose gateControl = new Pose(30, 73);
    public Pose intake3 = new Pose(10, 39.750-3.5, Math.toRadians(180));
    public Pose intake3Control = new Pose(75, intake3.getY()-5);
    public Pose score3Control = new Pose(67.67,54);
    public Pose score3          = new Pose(48, 96.0, Math.toRadians(131.67));
    public Pose intakeCorner = new Pose(10,20 , Math.toRadians(270));
    public Pose intakeCornerControl = new Pose(10,35);
    public Pose scoreToCorner = score.withHeading(Math.toRadians(-135));
    public Pose scoreCorner = score;
    public Pose park = new Pose(48, 72, Math.toRadians(180));

    // ---------------- Constructor ----------------
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

            scoreToCorner = scoreToCorner.mirror();
            scoreCorner = scoreCorner.mirror();

            park = park.mirror();

            RobotConstants.currentTeam = Alliance.RED;
        }else{
            RobotConstants.currentTeam = Alliance.BLUE;
        }
    }

    public PathChain scoreP() {
        return f.pathBuilder()
                .addPath(new BezierCurve(start, scorePControl, score))
                .setBrakingStrength(2.4)
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
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(score.getHeading(), intake1.getHeading(), 0.3)
                .build();
    }

    public PathChain score1() {
        return f.pathBuilder()
                .addPath(new BezierLine(intake1, score))
                .setBrakingStrength(2.1)
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
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(score.getHeading(), intake2.getHeading(), 0.5)
                .build();
    }

    public PathChain score2() {
        return f.pathBuilder()
                .addPath(new BezierLine(gate, score))
                .setBrakingStrength(2.1)
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

//    public PathChain scoreGate() {
//        return f.pathBuilder()
//                .addPath(new BezierCurve(gate, gateControl, score))
//                .setLinearHeadingInterpolation(gate.getHeading(), score.getHeading())
//                .build();
//    }

    public PathChain intake3() {
        return f.pathBuilder()
                .addPath(new BezierCurve(score, intake3Control, intake3))
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(score.getHeading(), intake3.getHeading(), 0.7)
                .build();
    }

    public PathChain score3() {
        return f.pathBuilder()
                .addPath(new BezierCurve(intake3, score3Control, score3))
                .setBrakingStrength(2.167)
                .setLinearHeadingInterpolation(intake3.getHeading(), score3.getHeading())
                .build();
    }
    public PathChain intakeCorner() {
        return f.pathBuilder()
                .addPath(new BezierCurve(score3, intakeCornerControl, intakeCorner))
                .setBrakingStrength(2)
                .setLinearHeadingInterpolation(score3.getHeading(),intakeCorner.getHeading(),0.3)
                .build();
    }

    public PathChain scoreCorner() {
        return f.pathBuilder()
                .addPath(new BezierLine(intakeCorner, scoreCorner))
                .setNoDeceleration()
                .setLinearHeadingInterpolation(intakeCorner.getHeading(), scoreCorner.getHeading())
                .build();
    }

    public PathChain park() {
        return f.pathBuilder()
                .addPath(new BezierLine(scoreCorner, park))
                .setLinearHeadingInterpolation(scoreCorner.getHeading(), park.getHeading())
                .build();
    }
}
