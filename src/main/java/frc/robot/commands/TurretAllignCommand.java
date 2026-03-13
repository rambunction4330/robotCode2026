// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretAllignCommand extends Command {
  /** Creates a new TurretAllignCommand. */

  private TurretSubsystem turretSubsystem;
  private CommandSwerveDrivetrain drivetrain; 
  private Translation2d hubVec_Blue = new Translation2d(182.11*.0254, 158.84*.0254);
  private Translation2d hubVec_Red = new Translation2d(469.11*.0254, 158.84*.0254);
  private Translation2d centerToTurret = new Translation2d(-5.197*.0254, -5*.0254);
  private Translation2d hubVec;
  private Supplier<ChassisSpeeds> m_robotSpeedsSup;


  public TurretAllignCommand(TurretSubsystem turret, CommandSwerveDrivetrain drivetrain, Supplier<ChassisSpeeds> robotSpeedsSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turretSubsystem = turret; 
    this.drivetrain = drivetrain; 
    this.m_robotSpeedsSup = robotSpeedsSup;
    super.addRequirements(turretSubsystem);

    hubVec = hubVec_Blue;
    
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      hubVec = hubVec_Red;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      hubVec = hubVec_Red;
    }

    Rotation2d currentRobotRot = drivetrain.getState().Pose.getRotation(); 
    Translation2d currentRobotVec = drivetrain.getState().Pose.getTranslation();

    Translation2d adjustedTurretVec = centerToTurret.rotateBy(currentRobotRot);
    adjustedTurretVec =  adjustedTurretVec.plus(currentRobotVec);
    Translation2d distanceVec = hubVec.minus(adjustedTurretVec);

    // Minus Robot Rotation from vector math angle to get the turret angle setpoint - Peine
    Rotation2d targetRot = distanceVec.getAngle().minus(drivetrain.getState().Pose.getRotation()); 

  SmartDashboard.putNumber("TargetRotTurret", targetRot.getRotations());
  //SmartDashboard.putNumber("currentTurretPosition", turretSubsystem.getTurret().getPosition().magnitude());
    turretSubsystem.setTurretPosition(Rotations.of(-targetRot.getRotations()-.5), .12*m_robotSpeedsSup.get().omegaRadiansPerSecond);

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
