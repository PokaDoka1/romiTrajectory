// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.sensor.RomiGyro;
import edu.wpi.first.math.util.Units;

public class RomiDrivetrain extends SubsystemBase {
  private Field2d m_field = new Field2d();

  public DifferentialDriveKinematics m_kinematics = Constants.DriveConstants.kDriveKinematics;

  private static final double kCountsPerRevolution = 1440.0;
  public static double yAngle = 0;
  private static final double kWheelDiameterInch = Units.inchesToMeters(2.75591); // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  public final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  public final RomiGyro m_gyro = new RomiGyro();

  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new RomiDrivetrain. */
  public RomiDrivetrain() {
    // Use inches as unit for encoder distances
    SmartDashboard.putData("ok",m_field);
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();
    m_gyro.reset();

    //parameters
    //angle, distance measured by left encoder, distance measured by right encoder
    m_odometry =
        new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), getLeftDistanceInch(), getRightDistanceInch());

    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch(){
    return ( (getLeftDistanceInch() + getRightDistanceInch()) / 2.0);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveKinematics getKinematics(){
    return m_kinematics;
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), getLeftDistanceInch(), getRightDistanceInch(), pose);
  }
  
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public static void reverseAlphabet(String input){
    //kevins pain
    String first = "";
    String[] arr = input.split("");
    String temp;
    for(int i = 1; i<arr.length; i++){
      for(int j = 0; j<arr.length-1; j++){
        if(arr[j].compareTo(arr[j+1]) <= 0){
          temp = arr[j];
          arr[j] = arr[j+1];
          arr[j+1] = temp;
        }
      }
    }
    String output = String.join("",arr);
    SmartDashboard.putString("reverse", output);
    System.out.println(output);
  }
  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts);
    m_diffDrive.feed();
  }

  public void setMaxOutput(double maxOutput) {
    m_diffDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  @Override
  public void periodic() {
    yAngle = m_gyro.getAngleY();
    m_odometry.update(
      m_gyro.getRotation2d(), getLeftDistanceInch(), getRightDistanceInch());
    m_field.setRobotPose(m_odometry.getPoseMeters());
    SmartDashboard.putNumber("y angle", m_gyro.getAngleY());
    SmartDashboard.putNumber("help me ", m_leftMotor.get());
    //SmartDashboard.putNumber("left distance inch", getLeftDistanceInch());
    //SmartDashboard.putNumber("right distance inch", getRightDistanceInch());
 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
