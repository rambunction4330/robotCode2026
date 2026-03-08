package frc.robot.Lib;

import static edu.wpi.first.units.Units.Rotations;

import java.util.Objects;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class SparkMaxPositionController {

    private final SparkMax sparkMax;

    private final ControlType controlType;

    private final sparkPcreateInfo info;

    public record sparkPMotorConfig(int id, SparkLowLevel.MotorType type, boolean inverted,
            int maxCurrent) {
    }

    public record sparkPpidConfig(double p, double i, double d, double iZone,
            double iMaxAccumulator, double maxOutput, double minOutput) {
    }

    public record sparkPrange(boolean isContinuous, Angle minPosition, Angle maxPosition) {
    }

    public record sparkPfeedBack(FeedbackSensor encoderType, double gearRatio) {
    }

    public record sparkPprofiling(boolean usingMaxMotion, AngularVelocity maxVelocity,
            AngularAcceleration maxAcceleration) {
    }

    public record sparkPcreateInfo(sparkPMotorConfig motorConfig, sparkPpidConfig pidConfig,
            SparkMaxPositionController leadController, sparkPrange range, sparkPfeedBack feedBack,
            sparkPprofiling profileConfig) {
    }

    public SparkMaxPositionController(final sparkPcreateInfo info) {

        this.info = info;
        SparkMaxConfig config = new SparkMaxConfig();
        sparkMax = new SparkMax(info.motorConfig.id, info.motorConfig.type);

        config.smartCurrentLimit(info.motorConfig.maxCurrent);
        config.inverted(info.motorConfig.inverted);
        // config.encoder.countsPerRevolution(42);

        // config.closedLoop.feedbackSensor(info.feedBack.encoderType);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(info.pidConfig.p, info.pidConfig.i, info.pidConfig.d);
        config.closedLoop.iZone(info.pidConfig.iZone);
        config.closedLoop.iMaxAccum(info.pidConfig.iMaxAccumulator);
        config.closedLoop.maxOutput(info.pidConfig.maxOutput);
        config.closedLoop.minOutput(info.pidConfig.minOutput);

        if (!Objects.isNull(info.feedBack.gearRatio) && info.feedBack.gearRatio !=
        0.0) {
        config.encoder.positionConversionFactor(info.feedBack.gearRatio);
        }
        // config.encoder.positionConversionFactor(info.feedBack.gearRatio);

        if (info.range.isContinuous) {
            config.closedLoop.positionWrappingEnabled(true);

            if (!Objects.isNull(info.range.minPosition.magnitude())
                    && !Objects.isNull(info.range.maxPosition.magnitude())) {
                config.closedLoop.positionWrappingInputRange(info.range.minPosition.magnitude(),
                        info.range.maxPosition.magnitude());
            }
        }

        if (info.profileConfig.usingMaxMotion) {
            controlType = SparkBase.ControlType.kMAXMotionPositionControl;
            config.closedLoop.maxMotion
                    .maxVelocity(info.profileConfig.maxVelocity.magnitude());
            config.closedLoop.maxMotion
                    .maxAcceleration(info.profileConfig.maxAcceleration.magnitude());
        } else {
            controlType = SparkBase.ControlType.kPosition;
        }

        // config.follow(info.leadController.getMotor(),
        // info.leadController.info.motorConfig.inverted);

        sparkMax.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setPosition(Angle position, double ff) {
        Angle targetPos;

        if (!Objects.isNull(info.range.minPosition.magnitude())
                && !Objects.isNull(info.range.maxPosition.magnitude())) {

            targetPos = Angle.ofBaseUnits(MathUtil.clamp(position.magnitude(), info.range.minPosition.magnitude(),
                    info.range.maxPosition.magnitude()), Rotations);
        } else {
            targetPos = position;
        }
        sparkMax.getClosedLoopController().setSetpoint(targetPos.magnitude(), controlType,
                ClosedLoopSlot.kSlot0, ff);

    }

    public void setPosition(Angle position) {
        Angle targetPos;

        if (!Objects.isNull(info.range.minPosition.magnitude())
                && !Objects.isNull(info.range.maxPosition.magnitude())) {

            targetPos = Angle.ofBaseUnits(MathUtil.clamp(position.magnitude(), info.range.minPosition.magnitude(),
                    info.range.maxPosition.magnitude()), Rotations);
        } else {
            targetPos = position;
        }

        sparkMax.getClosedLoopController().setSetpoint(targetPos.magnitude(), controlType,
                ClosedLoopSlot.kSlot0);

    }

    public Angle getPosition() {

        return Angle.ofBaseUnits(sparkMax.getEncoder().getPosition(), Rotations);
    }

    public void setEncoderPosition(Angle position) {
        sparkMax.getEncoder().setPosition(position.magnitude());
    }

    public void setPower(double power) {
        sparkMax.set(power);
    }

    public double getPower() {
        return sparkMax.get();
    }

    public Angle getMaxPos() {
        return info.range.maxPosition;
    }

    public Angle getMinPos() {
        return info.range.minPosition;
    }

    public void stop() {
        sparkMax.stopMotor();
    }

    public void disable() {
        sparkMax.disable();
    }

    public SparkMax getMotor() {
        return sparkMax;
    }

}
