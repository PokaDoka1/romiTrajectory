// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Balance extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RomiDrivetrain m_subsystem;

  private final PIDController m_pidAngle = new PIDController(0.05, 0, 0);
  private final PIDController m_pidStraight = new PIDController(0.01, 0, 0);

  private final double limit = 0.5;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Balance(RomiDrivetrain subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double outputAngle = -m_pidAngle.calculate(m_subsystem.m_gyro.getAngleY(), 0);
    double outputStraight = -m_pidStraight.calculate(m_subsystem.m_gyro.getAngleZ(), 0);
    m_subsystem.arcadeDrive((Math.abs(outputAngle) > limit) ?  limit * Math.signum(outputAngle) : outputAngle, 0);
    //m_subsystem.arcadeDrive(-outputAngle, -outputStraight);
    
    //SmartDashboard.putNumber("gyro Y angle" , m_subsystem.m_gyro.getAngleY());
    //SmartDashboard.putNumber("gyro Z angle" , m_subsystem.m_gyro.getAngleZ());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
