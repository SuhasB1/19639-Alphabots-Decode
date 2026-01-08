package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class RobotConstants {

    // Shooter

    // Intake
    public static double intakeIntake = 1;
    public static double intakeIdle = 0;
    public static double intakeOuttake = -1;
    public static double gateOpen = 0;
    public static double gateClose = 0.25;

    // Outtake
    public static double hoodMin = 0;
    public static double hoodMax = 1;

    //Field

    public static Pose BLUE_CASTLE = new Pose(1, 143);
    public static Pose RED_CASTLE = new Pose(143, 143);

    //Variable Storage

    public static Pose currentPose = new Pose();
    public static Team currentTeam = Team.RED;
    public enum Team {
        BLUE, RED
    }
    public static boolean software_are_bums = true;

}
