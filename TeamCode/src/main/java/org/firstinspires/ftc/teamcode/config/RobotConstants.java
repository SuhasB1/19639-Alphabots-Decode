package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class RobotConstants {

    public static Pose BLUE_CASTLE = new Pose(6.7, 141.5 - 6.7);
    public static Pose RED_CASTLE = new Pose(141.5 - 6.7, 141.5 - 6.7);

    public static Pose currentPose = new Pose(5,6.5);
    public static Alliance currentTeam = Alliance.BLUE;

    public static double rgb = 1;

    public static double goalX = 0;

    public static double goalY = 0;

    public static boolean software_are_bums = true;

    public static boolean actionBusy = false;


}
