// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Constants.driveConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AllignToPoseCommand extends Command {

  private CommandSwerveDrivetrain drivetrain;
  private CommandXboxController driveController;

  private FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
      .withRotationalDeadband(RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private Translation2d hubPoseBlueAlliance = new Translation2d(4, 4.5);
  private Translation2d hubPoseRedAlliance = new Translation2d(12, 4);

  /** Creates a new AllignToPoseCommand. */
  public AllignToPoseCommand(CommandSwerveDrivetrain drivetrain, CommandXboxController driveController) {
    this.drivetrain = drivetrain;
    this.driveController = driveController;
    super.addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AllianceStationID allianceStationID = DriverStationSim.getAllianceStationId();
    // ^ the above should be replaced with Alliance allianceStationID =
    // DriverStation.getAlliance().get();
    Translation2d hubVec;
    // DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    if (allianceStationID == AllianceStationID.Blue1 || allianceStationID == AllianceStationID.Blue2
        || allianceStationID == AllianceStationID.Blue3) {
      hubVec = this.hubPoseBlueAlliance;
    } else {
      hubVec = this.hubPoseRedAlliance;
    }

    Translation2d currentPoseVec = drivetrain.getState().Pose.getTranslation();
    Translation2d distanceVec = hubVec.minus(currentPoseVec);
    Rotation2d targetRot = distanceVec.getAngle();

    double newRot = drivetrain.getThetaController().calculate(
        drivetrain.getState().Pose.getRotation().getRadians(),
        targetRot.getRadians());

    drivetrain.setControl(drive.withVelocityX(-driveController.getLeftY() * driveConstants.MaxSpeed)
        .withVelocityY(-driveController.getLeftX() * driveConstants.MaxSpeed)
        .withRotationalRate(newRot));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
