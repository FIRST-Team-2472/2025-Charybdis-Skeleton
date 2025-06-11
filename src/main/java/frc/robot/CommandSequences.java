package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.defaultCommands.SwerveDriveToPointCmd;
import frc.robot.commands.SwerveFollowTransitionCmd;
import frc.robot.extras.PosPose2d;
import frc.robot.subsystems.SwerveSubsystem;

public class CommandSequences {

    public CommandSequences() {
    }
    public Command TestCommandOne(){
        return null;
    }

    public PosPose2d simplePose(double x, double y, double angleDegrees) {
        return new PosPose2d(x, y, Rotation2d.fromDegrees(angleDegrees));
    }
}