package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.geometry.Pose;

public class AprilTagRelocalizer {

    public static class TagInfo {
        public final int id;
        public final double fieldX;
        public final double fieldY;
        public final double heightIn;

        public TagInfo(int id, double fieldX, double fieldY, double heightIn) {
            this.id = id;
            this.fieldX = fieldX;
            this.fieldY = fieldY;
            this.heightIn = heightIn;
        }
    }

    private final TagInfo blueGoalTag;
    private final TagInfo redGoalTag;

    private final double camHeightIn;
    private final double camPitchRad;
    private final double camYawRad;

    private final double camForwardIn;
    private final double camLeftIn;

    public AprilTagRelocalizer(
            TagInfo blueGoalTag,
            TagInfo redGoalTag,
            double camHeightIn,
            double camPitchDeg,
            double camYawDeg,
            double camForwardIn,
            double camLeftIn
    ) {
        this.blueGoalTag = blueGoalTag;
        this.redGoalTag = redGoalTag;
        this.camHeightIn = camHeightIn;
        this.camPitchRad = Math.toRadians(camPitchDeg);
        this.camYawRad = Math.toRadians(camYawDeg);
        this.camForwardIn = camForwardIn;
        this.camLeftIn = camLeftIn;
    }

    public Pose estimateRobotPoseFromTagAngles(
            boolean isBlueAlliance,
            double robotHeadingRad,
            double txDeg,
            double tyDeg
    ) {
        TagInfo tag = isBlueAlliance ? blueGoalTag : redGoalTag;

        double txRad = Math.toRadians(txDeg);
        double tyRad = Math.toRadians(tyDeg);

        double pitchToTag = camPitchRad + tyRad;

        double dz = (tag.heightIn - camHeightIn);

        double forwardCamToTag;
        if (Math.abs(Math.tan(pitchToTag)) < 1e-6) {
            forwardCamToTag = 1e9;
        } else {
            forwardCamToTag = dz / Math.tan(pitchToTag);
        }

        if (forwardCamToTag < 0) forwardCamToTag = -forwardCamToTag;

        double rightCamToTag = forwardCamToTag * Math.tan(txRad);

        double rangeCamToTag = Math.hypot(forwardCamToTag, rightCamToTag);

        double bearingCamToTag = robotHeadingRad + camYawRad + Math.atan2(rightCamToTag, forwardCamToTag);

        double camFieldX = tag.fieldX - rangeCamToTag * Math.cos(bearingCamToTag);
        double camFieldY = tag.fieldY - rangeCamToTag * Math.sin(bearingCamToTag);

        double cosH = Math.cos(robotHeadingRad);
        double sinH = Math.sin(robotHeadingRad);

        double camOffsetFieldX = camForwardIn * cosH - camLeftIn * sinH;
        double camOffsetFieldY = camForwardIn * sinH + camLeftIn * cosH;

        double robotFieldX = camFieldX - camOffsetFieldX;
        double robotFieldY = camFieldY - camOffsetFieldY;

        return new Pose(robotFieldX, robotFieldY, robotHeadingRad);
    }
}
