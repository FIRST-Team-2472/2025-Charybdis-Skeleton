package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveDriveToPointCmd;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;


public class CommandSequences {

    public CommandSequences() {
    }
    public Command TestCommandOne(SwerveSubsystem swerveSubsystem){ // 10 feet forward
        return new SequentialCommandGroup(new SwerveDriveToPointCmd(swerveSubsystem, new Pose2d(3.048, 0, new Rotation2d())));
    }

    public Pose2d simplePose(double x, double y, double angleDegrees) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(angleDegrees));
    }
}