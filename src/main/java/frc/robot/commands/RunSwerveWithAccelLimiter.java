package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class RunSwerveWithAccelLimiter extends Command{
    SwerveSubsystem swerveSubsystem;

    double xPower, yPower, durationSeconds, turningSpeed;
    private Timer timer;
    
    public RunSwerveWithAccelLimiter(SwerveSubsystem swerveSubsystem, double xPower, double yPower, double turningSpeed, double durationSeconds) {
        this.swerveSubsystem = swerveSubsystem;

        this.xPower = xPower;
        this.yPower = yPower;
        this.durationSeconds = durationSeconds;
        this.turningSpeed = turningSpeed;
        timer = new Timer();

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){
        timer.restart();
    }

    @Override
    public void execute() {
            swerveSubsystem.runModulesFieldRelative(xPower, yPower, turningSpeed);
    }

    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(durationSeconds);
    }
}