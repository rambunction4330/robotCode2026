// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Lib.SparkMaxVelocityController;
import frc.robot.Lib.TalonFXVelocityController;

public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new IndexerSubsystem. */
  private SparkMaxVelocityController indexer;
  private TalonFXVelocityController kicker;
  private CommandXboxController controller;

  public IndexerSubsystem() {
    this.indexer = Constants.OperatorConstants.indexer;
    this.kicker = Constants.OperatorConstants.kicker;
    // this.controller = controller;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIndexterVelocity(AngularVelocity velocity) {
    indexer.setVelocity(velocity);
  }

  public void setKickerVelocity(AngularVelocity velocity) {
    kicker.setVelocity(velocity);
  }

  // Combine these next commands later
  // public Command kickCommand(AngularVelocity velocity) {
  //   return new RunCommand(() -> {
  //     setKickerVelocity(velocity);
  //   }, this);
  // }

  // public Command kickVoltage(double voltage) {
  //   return new RunCommand(() -> {
  //     kicker.getMotor().setVoltage(voltage);
  //   }, this);
  // }

  public void setIndexVol(double voltage) {
      indexer.getMotor().setVoltage(voltage);
  }

  public Command kickAndIndex(AngularVelocity kickVel, double indexerVol){
  return new RunCommand(
    ()-> {
      setKickerVelocity(kickVel);
      setIndexVol(indexerVol);
    }
  );
  }

  public Command Stop() {
    return new RunCommand(() -> {
      indexer.stop();
      kicker.stop();
    }, this);
  }

}
