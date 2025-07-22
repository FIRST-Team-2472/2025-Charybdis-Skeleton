package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RunSwerve;
import frc.robot.commands.RunSwerveWithAccelLimiter;
import frc.robot.commands.SwerveDriveToPointCmd;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;


public class CommandSequences {

    public CommandSequences() {
    }
    public Command TestCommandOne(SwerveSubsystem swerveSubsystem){ // 10 feet forward
        return new SequentialCommandGroup(new SwerveDriveToPointCmd(swerveSubsystem, new Pose2d(2, 0, new Rotation2d())));
    }
    public Command DriveMediumSpeedForward(SwerveSubsystem swerveSubsystem){
        return new SequentialCommandGroup(new RunSwerve(swerveSubsystem, .25, 0, 0, 6));
    }
    public Command DriveMediumSpeedForwardWithAccelLimiter(SwerveSubsystem swerveSubsystem){
        return new SequentialCommandGroup(new RunSwerveWithAccelLimiter(swerveSubsystem, .5, 0, 0, 6));
    }

    public Pose2d simplePose(double x, double y, double angleDegrees) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(angleDegrees));
    }
}