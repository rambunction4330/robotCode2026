// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Lib.SparkMaxPositionController;
import frc.robot.Lib.SparkMaxVelocityController;
import frc.robot.subsystems.Constants.turretConstants;

public class TurretSubsystem extends SubsystemBase {
  /** Creates a new TurretSubsystem. */

  private SparkMaxPositionController turret;

  private CommandXboxController controller;

  public TurretSubsystem(CommandXboxController controller) {
    this.turret = Constants.OperatorConstants.turret;
    this.controller = controller;
    turret.setEncoderPosition(Rotations.of(0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("currentEncoderReading", turret.getPosition().in(Rotations));
    SmartDashboard.putNumber("Current Setpoint", Rotations.of(turret.sparkMax.getClosedLoopController().getSetpoint()).magnitude());
    //SmartDashboard.putNumber("Gear Ratio", turret.getGearRatio());
  }

  // need to insert some logic to get closest angle due to turret overlap
  // we are planning to have a -200 + to 200 range of shooter or 400 degeress
  // overall - Peine
  public void setTurretPosition(Angle pos, double robotRotSpeed_RadPerSec) {
    // ArrayList<Double> angles = new ArrayList<Double>();
    // ArrayList<Double> diffs = new ArrayList<Double>();

SmartDashboard.putNumber("Pos",pos.in(Rotations));

    double smallestDiff = 10000.0;
    double targetSmallestPos = 0.0;

    for (int i = -2; i < 3; i++) {
      double testAngle = pos.in(Rotations) + i*1.0;
      //if ((turretConstants.kMinAngle < testAngle) && (testAngle < turretConstants.kMaxAngle)) {
        // angles.add(testAngle);
        double currentTargetDiff = Math.abs(turret.getPosition().in(Rotations) - testAngle);
        if (currentTargetDiff < smallestDiff) {
          smallestDiff = currentTargetDiff;
          targetSmallestPos = testAngle;
        }
        // diffs.add(currentTargetDiff);
     // }
    }

    SmartDashboard.putNumber("Smallest Angle", targetSmallestPos);
    // int indexOfMin = diffs.indexOf(Collections.min(diffs));

    // Add feedforeward of inversion of robot rotation speed(rad/s??)
    
    turret.setPosition(Rotations.of(targetSmallestPos));//, -.12 * turretConstants.kGearRatio * robotRotSpeed_RadPerSec);
  }

  public void setTurretPosOrginal(Angle pos){
    turret.setPosition(pos);
  }

  public Command setTurretCommand(Angle pos){
    return new RunCommand(()-> {setTurretPosOrginal(pos);}, this);
  }

  public void setTurretPosition() {
    double target = controller.getRightX() / 50;
    turret.setPosition(Rotations.of(target));
  }

  public SparkMaxPositionController getTurret() {
    return turret;
  }

}
