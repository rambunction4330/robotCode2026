package frc.robot.Lib;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class TalonFXVelocityController {
    public enum talonVisInverted {
        NOT_INVERTED,
        INVERTED
    }

    public enum talonVbrakeMode {
        BRAKE,
        NEUTRAL
    }

    private TalonFX talonMotorController;

    public record talonVmotorConfig(int id, talonVisInverted inverted,
            talonVbrakeMode mode, double minOutput, double maxOutput) {
    }

    public record talonVpidConfig(double p, double i, double d, double kS, double kV, double kA, double kG) {
    }

    public record talonVlimitsConfig(boolean statorCurrentEnable, Current statorCurrentLimit, boolean supplyCurrentEnable,
            Current supplyCurrentLimit) {
    }

    // Gear Ratio is labeled sensorToMech ratio in configuration
    public record talonVfeedBack(double gearRatio) {
    }

    

    public record talonVcreateInfo(talonVmotorConfig motorConfig, talonVpidConfig pidConfig,  
            talonVfeedBack feedBack, talonVlimitsConfig currentLimits) {
    }

    private final talonVcreateInfo info;

    public TalonFXVelocityController(talonVcreateInfo info) {
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

        config.CurrentLimits.StatorCurrentLimitEnable = info.currentLimits.statorCurrentEnable;
        
        if(!(info.currentLimits.statorCurrentLimit == null)) {
        config.CurrentLimits.StatorCurrentLimit = info.currentLimits.statorCurrentLimit.magnitude();
        }

        config.CurrentLimits.SupplyCurrentLimitEnable = info.currentLimits.supplyCurrentEnable;
        
        if(!(info.currentLimits.supplyCurrentLimit == null)){
        config.CurrentLimits.SupplyCurrentLimit = info.currentLimits.supplyCurrentLimit.magnitude();
        }

        config.Feedback.SensorToMechanismRatio = info.feedBack.gearRatio;

        talonMotorController.getConfigurator().apply(config);

    }

    public void setVelocity(AngularVelocity velocity){

        talonMotorController.setControl(new VelocityVoltage(velocity)); 

    }

    public AngularVelocity getVelocity(){
        StatusSignal<AngularVelocity> signal = talonMotorController.getVelocity(true);
        return signal.getValue(); 
    }

    public void setPower(double power){

        talonMotorController.set(power);
    }

    public double getPower(){
        return talonMotorController.get(); 
    }

    public void stop(){
        talonMotorController.stopMotor();
    }

    public void disable(){
        talonMotorController.disable();
    }


    public void follow(TalonFXVelocityController followController) {

        MotorAlignmentValue invert; 

        if(followController.info.motorConfig.inverted.ordinal() == 0){ invert = MotorAlignmentValue.Aligned; }
        else{ invert = MotorAlignmentValue.Opposed;  }

        talonMotorController.setControl(new Follower(followController.info.motorConfig.id, invert));
    }

    public TalonFX getMotor(){
        return talonMotorController; 
    }
}
