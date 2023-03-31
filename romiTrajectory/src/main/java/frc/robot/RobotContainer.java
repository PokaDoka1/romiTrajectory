// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

//import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Balance;
import frc.robot.commands.dog;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.RomiDrivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final RomiDrivetrain m_romiDrivetrain = new RomiDrivetrain();
  public final dog m_dog = new dog(m_romiDrivetrain, 1);
  public static final PS4Controller controller = new PS4Controller(0);
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_romiDrivetrain);
  private final Balance m_balance = new Balance(m_romiDrivetrain);
  PathPlannerTrajectory examplePath = PathPlanner.loadPath("jacksonLarry", new PathConstraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));
  //PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1.2);


  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              m_romiDrivetrain.resetOdometry(traj.getInitialPose());
          }
        }),
        new PPRamseteCommand(
            traj, 
            m_romiDrivetrain::getPose, // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts, Constants.DriveConstants.kvVoltSecondsPerMeter, Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
            m_romiDrivetrain.getKinematics(), // DifferentialDriveKinematics
            m_romiDrivetrain::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, Constants.DriveConstants.kIDriveVel, Constants.DriveConstants.kDDriveVel), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, Constants.DriveConstants.kIDriveVel, Constants.DriveConstants.kDDriveVel), // Right controller (usually the same values as left controller)
            m_romiDrivetrain::tankDriveVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            m_romiDrivetrain // Requires this drive subsystem
        )
    );
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_romiDrivetrain.setDefaultCommand(new ExampleCommand(m_romiDrivetrain));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return followTrajectoryCommand(examplePath, true);
    //return m_dog;

  }
}
