package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class DriveSubsystem {
    // Define motors
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;

    // Constructor
    public DriveSubsystem(HardwareMap hardwareMap) {
        // Initialize motors
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "rf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");

        // Set motor directions (adjust based on your robot's configuration)
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(currentPose);

        // Set all motors to zero power
        stop();
    }

    // Mecanum drive logic
    public void drive() {

    }

    // Stop all motors
    public void stop() {

    }
}
