package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveCommand extends Command {

  private final DriveSubsystem m_driveSubsystem;
  private final CommandXboxController m_driverController;

  public ArcadeDriveCommand(DriveSubsystem driveSubsystem, CommandXboxController driverController) {
    this.m_driveSubsystem = driveSubsystem;
    this.m_driverController = driverController;
    addRequirements(m_driveSubsystem);
  }

  @Override
  public void initialize() {
    // No initialization needed
  }

  @Override
  public void execute() {
    double turnSpeed = -m_driverController.getLeftX();
    double driveSpeed = -m_driverController.getLeftY();

    m_driveSubsystem.drive(driveSpeed, turnSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0);
  }

  @Override
  public boolean isFinished() {
    return false; // Runs until interrupted
  }
}
