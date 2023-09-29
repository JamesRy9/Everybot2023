// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

//import com.pathplanner.lib.PathPlannerTrajectory;
//import com.pathplanner.lib.commands.PPRamseteCommand;

//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.RamseteController;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants; 
import frc.robot.subsystems.gyro.GyroSubsystem;

public class DriveBaseSubsystem extends SubsystemBase {
  /** Creates a new DriveBaseSubsystem2. */
  private SwerveModule[] swerveModules;
  private SwerveDriveKinematics m_kinematics;
  private SwerveDriveOdometry m_odometry;
  private GyroSubsystem gyro;
  private SwerveModulePosition[] positions;
  

  public DriveBaseSubsystem(GyroSubsystem gyro) {
    swerveModules = new SwerveModule[] {
    new SwerveModule(SwerveConstants.swerve0.turnMotorID, SwerveConstants.swerve0.speedMotorID, SwerveConstants.swerve0.turnEncoderID),
    new SwerveModule(SwerveConstants.swerve1.turnMotorID, SwerveConstants.swerve1.speedMotorID, SwerveConstants.swerve1.turnEncoderID),
    new SwerveModule(SwerveConstants.swerve2.turnMotorID, SwerveConstants.swerve2.speedMotorID, SwerveConstants.swerve2.turnEncoderID),
    new SwerveModule(SwerveConstants.swerve3.turnMotorID, SwerveConstants.swerve3.speedMotorID, SwerveConstants.swerve3.turnEncoderID),
  };
  positions = new SwerveModulePosition[4];
  positions[0] = getSwerveModule(0).getPosition();
  positions[1] = getSwerveModule(1).getPosition();
  positions[2] = getSwerveModule(2).getPosition();
  positions[3] = getSwerveModule(3).getPosition();
  //TODO: we need the gyro and the module pos for odometry
  this.gyro = gyro;
  m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d(gyro.getAngle()), positions, new Pose2d(0, 0, new Rotation2d(0)));
  m_kinematics = new SwerveDriveKinematics(SwerveConstants.swerve0.location, SwerveConstants.swerve1.location, SwerveConstants.swerve2.location, SwerveConstants.swerve3.location); }
  public SwerveModule getSwerveModule(int index) {
    return swerveModules[index];
  }
  public SwerveDriveKinematics getSwerveDriveKinematics() {
    return m_kinematics;
  }

  public SwerveModule[] getSwerveModules() {
    return swerveModules;
  }
  //call odometry update in this periodic
  @Override
  public void periodic() {
    positions[0] = getSwerveModule(0).getPosition();
    positions[1] = getSwerveModule(1).getPosition();
    positions[2] = getSwerveModule(2).getPosition();
    positions[3] = getSwerveModule(3).getPosition();
    m_odometry.update(new Rotation2d(gyro.getAngle()), positions);
  }
  /*
  // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        // Reset odometry for the first path you run during auto
        if(isFirstPath){
            this.resetOdometry(traj.getInitialPose());
        }
      }),
      new PPRamseteCommand(
          traj, 
          this::getPose, // Pose supplier
          new RamseteController(),
          new SimpleMotorFeedforward(KS, KV, KA),
          this.m_kinematics, // DifferentialDriveKinematics
          this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
          new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
          new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
          this::outputVolts, // Voltage biconsumer
          true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          this // Requires this drive subsystem
      )
    );
  }*/
}
