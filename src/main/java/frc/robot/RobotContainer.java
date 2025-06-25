// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.defaultCommands.SwerveJoystickCmd;

import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private String m_autoSelected;
  private final String
    testAutoOne = "Test Auto One";


  private final SendableChooser<String> autoChooser = new SendableChooser<>();
  private final SendableChooser<String> testChooser = new SendableChooser<>();
  private final CommandSequences commandSequences = new CommandSequences();

  // Add subsystems below this comment
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  //Add subsystems below this comment

  // Make sure this xbox controller is correct and add driver sticks
  CommandXboxController xboxController = new CommandXboxController(OperatorConstants.kXboxControllerPort);

  public static Joystick leftJoystick = new Joystick(OperatorConstants.kLeftJoystickPort);
  public static Joystick rightJoystick = new Joystick(OperatorConstants.kRightJoystickPort);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(swerveSubsystem,
        () -> -leftJoystick.getX(), // negative because we get the inverse value
        () -> -leftJoystick.getY(), // negative because we get the inverse value
        () -> -rightJoystick.getX(),
        () -> rightJoystick.getRawButton(1),
        () -> rightJoystick.getRawButton(4)));

    autoChooser.addOption(testAutoOne, testAutoOne);


    if (DriverStation.isFMSAttached() == true) {
      ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
      driverBoard.add("Auto choices", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    } else {
      ShuffleboardTab driverBoard = Shuffleboard.getTab("Driver Board");
      driverBoard.add("Auto choices", autoChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

      ShuffleboardTab autoTestingBoard = Shuffleboard.getTab("Auto Testing");
      autoTestingBoard.add("Auto choices", testChooser).withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    ShuffleboardTab autoTestingBoard = Shuffleboard.getTab("Auto Testing");
    autoTestingBoard.add("Auto choices - in testing", testChooser).withWidget(BuiltInWidgets.kComboBoxChooser);

  }

  public Command getAutonomousCommand() {
    m_autoSelected = autoChooser.getSelected();

    if (m_autoSelected != null) {
      switch (m_autoSelected) {
        case testAutoOne:
          return new SequentialCommandGroup(
              commandSequences.TestCommandOne(swerveSubsystem));
      }
    }

    return null;
  }
}
