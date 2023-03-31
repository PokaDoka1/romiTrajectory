// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class dog extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final RomiDrivetrain m_db;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private boolean chargingStationDetected = false;
  private boolean chargingStationBalanced = false;
  private boolean crosslineCheck = false;
  private int backward = 1;
  private double angle = 5;
  private double maxMeters = 15;
  private int negative = 1;
  private final double limit = 0.5;
  private final PIDController m_pidAngle = new PIDController(0.2, 0.01, 0);
  private final PIDController m_pidStraight = new PIDController(0.02, 0.01, 0);



  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public dog(RomiDrivetrain db, int goBackward) {
    m_db = db;
    if (goBackward < 0){
      backward = goBackward;
    }
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(db);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_db.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentAngle = m_db.m_gyro.getAngleY();

    //keep going forward until you reach the 12 degrees
    if ((currentAngle < angle)){
      m_db.arcadeDrive(0.5 * backward, 0);
      if(m_db.getAverageDistanceInch() > 12){
        // crosslineCheck = true;
      }
    }
    //once ur tilted at the correct angle, the command ends
    else{
      chargingStationDetected = true;
    }

    if(chargingStationDetected){
      double outputAngle = MathUtil.clamp( -backward * m_pidAngle.calculate(m_db.m_gyro.getAngleY(), 0), -0.5, 0.5);
      m_db.arcadeDrive(outputAngle, -m_pidStraight.calculate(m_db.m_gyro.getAngleZ(), 0));
      if(m_pidAngle.atSetpoint()){
        chargingStationBalanced = true;
      }
    
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_db.arcadeDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ( crosslineCheck || chargingStationBalanced );
  }
}