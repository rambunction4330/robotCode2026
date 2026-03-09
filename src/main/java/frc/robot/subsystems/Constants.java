package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Lib.TalonFXPositionController;
import frc.robot.Lib.TalonFXPositionController.talonPcreateInfo;
import frc.robot.Lib.TalonFXPositionController.talonPfeedBack;
import frc.robot.Lib.TalonFXPositionController.talonPlimitsConfig;
import frc.robot.Lib.TalonFXPositionController.talonPmotorConfig;
import frc.robot.Lib.TalonFXPositionController.talonPpidConfig;
import frc.robot.Lib.TalonFXPositionController.talonPrange;
import frc.robot.Lib.TalonFXPositionController.talonPbrakeMode;
import frc.robot.Lib.TalonFXPositionController.talonPisInverted;

import frc.robot.Lib.TalonFXVelocityController;
import frc.robot.Lib.TalonFXVelocityController.talonVcreateInfo;
import frc.robot.Lib.TalonFXVelocityController.talonVfeedBack;
import frc.robot.Lib.TalonFXVelocityController.talonVlimitsConfig;
import frc.robot.Lib.TalonFXVelocityController.talonVmotorConfig;
import frc.robot.Lib.TalonFXVelocityController.talonVpidConfig;
import frc.robot.generated.TunerConstants;
import frc.robot.Lib.TalonFXVelocityController.talonVbrakeMode;
import frc.robot.Lib.TalonFXVelocityController.talonVisInverted;

import frc.robot.Lib.SparkMaxPositionController;
import frc.robot.Lib.SparkMaxPositionController.sparkPcreateInfo;
import frc.robot.Lib.SparkMaxPositionController.sparkPfeedBack;
import frc.robot.Lib.SparkMaxPositionController.sparkPMotorConfig;
import frc.robot.Lib.SparkMaxPositionController.sparkPpidConfig;
import frc.robot.Lib.SparkMaxPositionController.sparkPprofiling;
import frc.robot.Lib.SparkMaxPositionController.sparkPrange;

import frc.robot.Lib.SparkMaxVelocityController;
import frc.robot.Lib.SparkMaxVelocityController.sparkVcreateInfo;
import frc.robot.Lib.SparkMaxVelocityController.sparkVfeedBack;
import frc.robot.Lib.SparkMaxVelocityController.sparkVmotorConfig;
import frc.robot.Lib.SparkMaxVelocityController.sparkVpidConfig;
import frc.robot.Lib.SparkMaxVelocityController.sparkVprofiling;

public final class Constants {
  public static class OperatorConstants {

    /// Intake Constants
    ///arm starts upward, use cod for calculations of kG value in line with most torque power applied to the motor
    static SparkMaxPositionController intakeArm = new SparkMaxPositionController(new sparkPcreateInfo(
        new sparkPMotorConfig(50, SparkLowLevel.MotorType.kBrushless , true, 0),
        new sparkPpidConfig(2, 0, 0, 0, 0, 40, -40),
        null,
        new sparkPrange(true, Rotations.of(0), Rotations.of(8)),
        new sparkPfeedBack(FeedbackSensor.kPrimaryEncoder, 0.02777),
        new sparkPprofiling(false,  RotationsPerSecond.of(0), RotationsPerSecondPerSecond.of(0))));

    static TalonFXVelocityController intakeRollers = new TalonFXVelocityController(new talonVcreateInfo(
        new talonVmotorConfig(51, talonVisInverted.NOT_INVERTED, talonVbrakeMode.NEUTRAL, -1, 40),
        new talonVpidConfig(1, 0, 0, 0, 0, 0, 0),
        new talonVfeedBack(2), new talonVlimitsConfig(true, Amps.of(40), true, Amp.of(20))));

    /// Indexer Constants
    static SparkMaxVelocityController indexer = new SparkMaxVelocityController(new sparkVcreateInfo(
        new sparkVmotorConfig(52, SparkLowLevel.MotorType.kBrushless, false, 0),
        new sparkVpidConfig(1, 0, 0, 0, 0, 40, -40),
        null,
        new sparkVfeedBack(FeedbackSensor.kPrimaryEncoder, 1),
        new sparkVprofiling(false,  RotationsPerSecond.of(0), RotationsPerSecondPerSecond.of(0))));

    static TalonFXVelocityController kicker = new TalonFXVelocityController(new talonVcreateInfo(
        new talonVmotorConfig(53, talonVisInverted.INVERTED, talonVbrakeMode.NEUTRAL, -1, 1),
        new talonVpidConfig(0.5, 0, 0, 0, 0, 0, 0),
        new talonVfeedBack(1), new talonVlimitsConfig(true, Amps.of(40), true, Amps.of(20))));

    /// Shooter Constants
    static TalonFXVelocityController shooter = new TalonFXVelocityController(new talonVcreateInfo(
        new talonVmotorConfig(54, talonVisInverted.INVERTED, talonVbrakeMode.NEUTRAL, -1,1),
        new talonVpidConfig(0.5, 0, 0, 0, 0, 0, 0),
        new talonVfeedBack(1), new talonVlimitsConfig(true, Amps.of(100), true, Amps.of(40))));

    static SparkMaxPositionController hood = new SparkMaxPositionController(new sparkPcreateInfo(
        new sparkPMotorConfig(55, SparkLowLevel.MotorType.kBrushless , false, 0),
        new sparkPpidConfig(50, 0, 0, 0, 0, 1, -1),
        null,
        new sparkPrange(false, Rotations.of(0), Rotations.of(10)),
        new sparkPfeedBack(FeedbackSensor.kPrimaryEncoder, (double)(1/((48/14)*(170/15)))),
        new sparkPprofiling(false, RotationsPerSecond.of(0), RotationsPerSecondPerSecond.of(0))));

    /// Turret Constants
    static SparkMaxPositionController turret = new SparkMaxPositionController(new sparkPcreateInfo(
        new sparkPMotorConfig(56, SparkLowLevel.MotorType.kBrushless, false, 0), 
        new sparkPpidConfig(0.5, 0, 0, 0, 0, 1, -1), 
        null, 
        new sparkPrange(false, Rotations.of(-100), Rotations.of(100)), 
        new sparkPfeedBack(FeedbackSensor.kPrimaryEncoder, turretConstants.kGearRatio),
        new sparkPprofiling(false, RotationsPerSecond.of(0), RotationsPerSecondPerSecond.of(0))));

  }

  public static class driveConstants {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  }

  public static class turretConstants {
    public static double kGearRatio =1;  //(100/10 * 3/1);
    public static double kMinAngle = -200;
    public static double kMaxAngle = 200;
  }

  public static class intakeConstants {
    public static double kArmOut = 1; //in radians
  }
}
