// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Lib.SparkMaxPositionController;
import frc.robot.Lib.TalonFXVelocityController;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  private TalonFXVelocityController shooter;
  private SparkMaxPositionController hood;
  private CommandXboxController controller;
  private AngularVelocity targetShootVelocity;

  public ShooterSubsystem(CommandXboxController controller) {
    this.shooter = Constants.OperatorConstants.shooter;
    this.hood = Constants.OperatorConstants.hood;
    this.controller = controller;
    Constants.hoodMapFunc();
    Constants.shootMapFunc();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShootVelocity(AngularVelocity velocity) {
    this.targetShootVelocity = velocity;
    shooter.setVelocity(velocity);
  }

  public AngularVelocity getCurrentVelocity() {
    return shooter.getVelocity();
  }

  public boolean isShooterAtTargetVel() {
    return getCurrentVelocity().isNear(targetShootVelocity, 0.005);
  }

  public void setHoodPosition(Angle pos) {
    hood.setPosition(pos);
  }

  public void setHoodPosition() {
    double target = controller.getLeftY() / 50;
    hood.setPosition(Rotations.of(target));
  }

  public Command rezeroHood() {
    return new RunCommand(() -> {
      hood.setEncoderPosition(Rotations.of(0.0));
    }, this);
  }

  public SparkMaxPositionController getHood() {
    return hood;
  }

  public TalonFXVelocityController getShooter() {
    return shooter;
  }

  public Command Stop(){
    return new RunCommand(()->{hood.stop();
    shooter.stop();}, this);
  }

  public Command shootCommand(AngularVelocity velocity){ //, Angle position) {
    return new RunCommand(() -> {
      setShootVelocity(velocity);
      // setHoodPosition(position);
    }, this);

  }

  public Command shootCommand2(DoubleSupplier distance){
   return new RunCommand(()->{
    setShootVelocity(RotationsPerSecond.of(Constants.velocityMap.get(distance.getAsDouble())));
    setHoodPosition(Rotations.of(Constants.hoodMap.get(distance.getAsDouble())));

   }, this);
  }

}
