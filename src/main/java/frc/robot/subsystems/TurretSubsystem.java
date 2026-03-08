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
  }

  // need to insert some logic to get closest angle due to turret overlap
  // we are planning to have a -200 + to 200 range of shooter or 400 degeress
  // overall - Peine
  public void setTurretPosition(Angle pos, double robotRotSpeed_RadPerSec) {
    // ArrayList<Double> angles = new ArrayList<Double>();
    // ArrayList<Double> diffs = new ArrayList<Double>();

    double smallestDiff = 2000000000;
    double targetSmallestPos = 45;

    for (int i = -2; i < 3; i++) {
      double testAngle = pos.in(Degrees) + (360 * i);
      if ((turretConstants.kMinAngle <= testAngle) && (testAngle <= turretConstants.kMaxAngle)) {
        // angles.add(testAngle);
        double currentTargetDiff = Math.abs(turret.getPosition().in(Degrees) - testAngle);
        if (currentTargetDiff < smallestDiff) {
          smallestDiff = currentTargetDiff;
          targetSmallestPos = testAngle;
        }
        // diffs.add(currentTargetDiff);
      }
    }
    // int indexOfMin = diffs.indexOf(Collections.min(diffs));

    // Add feedforeward of inversion of robot rotation speed(rad/s??)
    SmartDashboard.putNumber("targetSmallestPos",targetSmallestPos);
    turret.setPosition(Degrees.of(targetSmallestPos));//, -.12 * turretConstants.kGearRatio * robotRotSpeed_RadPerSec);
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
