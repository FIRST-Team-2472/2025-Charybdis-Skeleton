package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class RobotStatus {

    public static double pigeonYaw = 0;
    public static double pigeonPitch = 0;
    public static double pigeonRoll = 0;

    public static Pose2d odometryBotPose;

    public static Pose2d[] LimeLightBotPoses;
    public static double[] LimeLightConfidences;

    public static Pose2d filteredBotPose;

}