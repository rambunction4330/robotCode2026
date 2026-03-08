// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Lib.SparkMaxPositionController;
import frc.robot.Lib.TalonFXPositionController;
import frc.robot.Lib.TalonFXVelocityController;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  /// Arm: Falcon 500 motor
  /// rollers: Falcon 500 motor

  private SparkMaxPositionController arm;
  private TalonFXVelocityController rollers;
  private CommandXboxController controller;

  public IntakeSubsystem(CommandXboxController controller) {
    this.arm = Constants.OperatorConstants.intakeArm;
    this.rollers = Constants.OperatorConstants.intakeRollers;
    this.controller = controller;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setArmPosition(Angle pos) {
    arm.setPosition(pos);
  }

  public void setArmPosition() {
    double target = controller.getRightY() / 50;
    arm.setPosition(Rotations.of(target));
  }

  public void setRollersVelocity(double velocity_RotPerMin) {
    rollers.setVelocity(RotationsPerSecond.of(velocity_RotPerMin));
  }

  public Command intakeCommand(Angle pos, double velocity_RotPerMin) {
    return new RunCommand(
        () -> {
          setArmPosition(pos);
          setRollersVelocity(velocity_RotPerMin);
        }, this);
  }
}
