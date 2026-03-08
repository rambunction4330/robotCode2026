package frc.robot.Lib;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

public class TalonFXPositionController {

   public enum talonPisInverted {
        NOT_INVERTED,
        INVERTED
    }

    public enum talonPbrakeMode {
        BRAKE,
        NEUTRAL
    }

    private TalonFX talonMotorController; 
    

    public record talonPmotorConfig(int id, talonPisInverted inverted,
           talonPbrakeMode mode, double minOutput, double maxOutput) {
    }

    public record talonPpidConfig(double p, double i, double d, double kS, double kV, double kA, double kG) {
    }

    public record talonPlimitsConfig(boolean statorCurrentEnable, Current statorCurrentLimit, boolean supplyCurrentEnable,
            Current supplyCurrentLimit) {
    }

    // Gear Ratio is labeled sensorToMech ratio in configuration
    public record talonPfeedBack(double gearRatio) {
    }

    public record talonPrange(boolean isContinuous, Angle minPos, Angle maxPos) {
    }

    public record talonPcreateInfo(talonPmotorConfig motorConfig, talonPpidConfig pidConfig, talonPlimitsConfig currentLimits, talonPrange range,
            talonPfeedBack feedBack) {
    }

    private final talonPcreateInfo info;

    public TalonFXPositionController(talonPcreateInfo info) {
        this.info = info;

        TalonFXConfiguration config = new TalonFXConfiguration();
        talonMotorController = new TalonFX(info.motorConfig.id);

        config.MotorOutput.Inverted = InvertedValue.valueOf(info.motorConfig.inverted.ordinal());
        config.MotorOutput.NeutralMode = NeutralModeValue.valueOf(info.motorConfig.mode.ordinal());
        config.MotorOutput.PeakReverseDutyCycle = info.motorConfig.minOutput;
        config.MotorOutput.PeakForwardDutyCycle = info.motorConfig.maxOutput;

        config.Slot0.kP = info.pidConfig.p;
        config.Slot0.kI = info.pidConfig.i;
        config.Slot0.kD = info.pidConfig.d;
        config.Slot0.kS = info.pidConfig.kS;
        config.Slot0.kV = info.pidConfig.kV; 
        config.Slot0.kA = info.pidConfig.kA; 
        config.Slot0.kG = info.pidConfig.kG;

        // Refer to these links when implementing any sort of Current Limits
        // https://www.chiefdelphi.com/t/current-limiting-talonfx-values/374780/4 and
        // https://www.chiefdelphi.com/t/current-limiting-on-swerve/454392/2

        config.CurrentLimits.StatorCurrentLimitEnable = info.currentLimits.statorCurrentEnable;
        config.CurrentLimits.StatorCurrentLimit = info.currentLimits.statorCurrentLimit.magnitude();
        config.CurrentLimits.SupplyCurrentLimitEnable = info.currentLimits.supplyCurrentEnable;
        config.CurrentLimits.SupplyCurrentLimit = info.currentLimits.supplyCurrentLimit.magnitude();

        config.Feedback.SensorToMechanismRatio = info.feedBack.gearRatio;
        config.ClosedLoopGeneral.ContinuousWrap = info.range.isContinuous;

        talonMotorController.getConfigurator().apply(config);
    }

    public void setPosition(Angle position) {
        Angle targetPos = Angle.ofBaseUnits(MathUtil.clamp(position.magnitude(), info.range.maxPos.magnitude(),
                info.range.maxPos.magnitude()), Rotations);

        talonMotorController.setControl(new PositionVoltage(targetPos));

    }

    public Angle getPosition() {

        StatusSignal<Angle> signal = talonMotorController.getRotorPosition(true);

        return signal.getValue();

    }

    public void setPower(double power) {
        talonMotorController.set(power); 
    }

    public double getPower(){
        return talonMotorController.get(); 
    }

    public void setEncoderPosition() {
        
    }

    public void disable() {
        talonMotorController.disable();
    }

    public void stop() {
        talonMotorController.stopMotor();
    }

    public void follow(TalonFXPositionController followController) {

        MotorAlignmentValue invert; 

        if(followController.info.motorConfig.inverted.ordinal() == 0){ invert = MotorAlignmentValue.Aligned ; }
        else{ invert = MotorAlignmentValue.Opposed;  }

        talonMotorController.setControl(new Follower(followController.info.motorConfig.id, invert));
    }

    public TalonFX getMotor(){
        return talonMotorController; 
    }
}
