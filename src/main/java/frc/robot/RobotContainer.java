// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.drive.Drive;
import frc.robot.drive.SwerveConstants;
import frc.robot.drive.Drive.DriveState;
import frc.robot.drive.Gyro.GyroHardware;
import frc.robot.drive.SwerveModule.SwerveModule;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.VisionHardware;

public class RobotContainer {
  public CommandXboxController driveController = new CommandXboxController(0);
  public Drive drive;

  public RobotContainer() {
    drive = new Drive(
      new SwerveModule[]{
        new SwerveModule(SwerveConstants.frontLeft), 
        new SwerveModule(SwerveConstants.frontRight), 
        new SwerveModule(SwerveConstants.backLeft), 
        new SwerveModule(SwerveConstants.backRight)}, 
        new GyroHardware(),
        new Vision(new VisionHardware[] {
          new VisionHardware(VisionConstants.kExampleCamName, VisionConstants.kExampleTransform)
        }));
    
    drive.acceptJoystickInputs(
      () -> driveController.getLeftY(), 
      () -> driveController.getLeftX(), 
      () -> driveController.getRightX());
    
    configureTriggers();
    configureBindings();
  }

  private void configureTriggers() {
    new Trigger(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> drive.resetAllEncoders()));
  }
  
  private void configureBindings() {
    driveController.x().onTrue(Commands.runOnce(() -> {drive.resetGyro();}));

    driveController.povUp().onTrue(drive.setDriveStateCommandContinued(DriveState.SNIPER_UP)).onFalse(drive.setDriveStateCommand(DriveState.TELEOP));
        
    driveController.povRight().onTrue(drive.setDriveStateCommandContinued(DriveState.SNIPER_RIGHT)).onFalse(drive.setDriveStateCommand(DriveState.TELEOP));

    driveController.povDown().onTrue(drive.setDriveStateCommandContinued(DriveState.SNIPER_DOWN)).onFalse(drive.setDriveStateCommand(DriveState.TELEOP));

    driveController.povLeft().onTrue(drive.setDriveStateCommandContinued(DriveState.SNIPER_LEFT)).onFalse(drive.setDriveStateCommand(DriveState.TELEOP));
  }



  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}