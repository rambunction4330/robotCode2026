package frc.robot.Lib;


import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Objects;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class SparkMaxVelocityController {
    private  SparkMax sparkMax;

    private final ControlType controlType;

    private final sparkVcreateInfo info;

    public record  sparkVmotorConfig(int id, SparkLowLevel.MotorType type, boolean inverted,
            int maxCurrent) {
    }

    public record  sparkVpidConfig(double p, double i, double d, double iZone,
            double iMaxAccumulator, double maxOutput, double minOutput) {
    }

    public record  sparkVfeedBack(FeedbackSensor encoderType, double gearRatio) {
    }

    public record  sparkVprofiling(boolean usingMaxMotion, AngularVelocity maxVelocity, AngularAcceleration maxAcceleration) {
    }

    public record  sparkVcreateInfo( sparkVmotorConfig motorConfig,  sparkVpidConfig pidConfig,
            SparkMaxVelocityController leader,  sparkVfeedBack feedBack,  sparkVprofiling profileConfig) {
    }

    public SparkMaxVelocityController( sparkVcreateInfo info) {

        this.info = info;
        SparkMaxConfig config = new SparkMaxConfig();
        sparkMax = new SparkMax(info.motorConfig.id, info.motorConfig.type);

        config.smartCurrentLimit(info.motorConfig.maxCurrent);
        config.inverted(info.motorConfig.inverted);
        //config.encoder.countsPerRevolution(42);

        config.closedLoop.feedbackSensor(info.feedBack.encoderType);
        config.closedLoop.pid(info.pidConfig.p, info.pidConfig.i, info.pidConfig.d);
        config.closedLoop.iZone(info.pidConfig.iZone);
        config.closedLoop.iMaxAccum(info.pidConfig.iMaxAccumulator);
        config.closedLoop.maxOutput(info.pidConfig.maxOutput);
        config.closedLoop.minOutput(info.pidConfig.minOutput);


        if (Objects.isNull(info.feedBack.gearRatio) || info.feedBack.gearRatio != 0.0) {
            config.encoder.velocityConversionFactor(info.feedBack.gearRatio);
        }

        if (info.profileConfig.usingMaxMotion) {
            controlType = SparkBase.ControlType.kMAXMotionVelocityControl;
            config.closedLoop.maxMotion
                    .maxVelocity(info.profileConfig.maxVelocity.magnitude());
            config.closedLoop.maxMotion
                    .maxAcceleration(info.profileConfig.maxAcceleration.magnitude() );
        } else {
            controlType = SparkBase.ControlType.kVelocity;
        }

        //config.follow(info.leader.getMotor(), info.leader.info.motorConfig.inverted);

        sparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setVelocity(AngularVelocity velocity) {
        AngularVelocity targetVelocity = AngularVelocity.ofBaseUnits(velocity.magnitude(), RotationsPerSecond);
        sparkMax.getClosedLoopController().setSetpoint(targetVelocity.magnitude(), controlType);
    }

    public AngularVelocity getVelocity(){
       return RotationsPerSecond.of(sparkMax.getEncoder().getVelocity()); 
    }

    public void setPower(double power) {
        sparkMax.set(power);
    }

    public void disable() {
        sparkMax.disable();
    }

    public void stop() {
        sparkMax.stopMotor();
    }

    public SparkMax getMotor() {
        return sparkMax;
    }
}
