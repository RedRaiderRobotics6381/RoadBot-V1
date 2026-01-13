package frc.robot.subsystems.Secondary;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Climber extends SubsystemBase {
    public TalonFX climbMtrLdr;
    public TalonFX climbMtrFlw;
    public TalonFXConfiguration  climbConfigLdr;
    public MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);

    private double kP = 0.15; 
    private double kD = 0.075;
    public DigitalInput limitSw;
    private boolean climberInitialized;

    public Climber() {
        climbMtrLdr = new TalonFX(ClimberConstants.LEFT_CLIMBER_MOTOR_PORT);
        climbMtrFlw = new TalonFX(ClimberConstants.RIGHT_CLIMBER_MOTOR_PORT);
        climbMtrFlw.setControl(new Follower(ClimberConstants.LEFT_CLIMBER_MOTOR_PORT, false));
        climbConfigLdr = new TalonFXConfiguration();
        limitSw = new DigitalInput(9);

        climbConfigLdr.Slot0.kP = kP;
        climbConfigLdr.Slot0.kD = kD;

        climbConfigLdr.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbConfigLdr.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        climbConfigLdr.CurrentLimits.SupplyCurrentLimitEnable = true;
        climbConfigLdr.CurrentLimits.StatorCurrentLimitEnable = true;
        climbConfigLdr.CurrentLimits.SupplyCurrentLimit = 30.0;
        climbConfigLdr.CurrentLimits.StatorCurrentLimit = 50.0;

        climbConfigLdr.MotionMagic.MotionMagicAcceleration = ClimberConstants.CLIMBER_ACCELERATION_CONSTRAINT;
        climbConfigLdr.MotionMagic.MotionMagicCruiseVelocity = ClimberConstants.CLIMBER_VELOCITY_CONSTRAINT;

        climbMtrLdr.getConfigurator().apply(climbConfigLdr);
        climbMtrFlw.getConfigurator().apply(climbConfigLdr); 
    }

    public void setClimberHeight(double pos) {
        climbMtrLdr.setControl(motionMagicVoltage.withPosition(pos));
    }

    public FunctionalCommand ClimberHeightCmd(double height) {
        return new FunctionalCommand(() -> {},
            () -> setClimberHeight(height),
            interrupted -> {},
            () -> Math.abs(height - climbMtrLdr.getPosition().getValueAsDouble()) <= 0.5,
            this);
    }
            
    public FunctionalCommand ClimberInitCmd() {
        return new FunctionalCommand(() -> climberInitialized = false,
                                     () -> {
                                        if(limitSw.get()){
                                            climbMtrLdr.set(-.125);
                                        } else if(!limitSw.get()) {
                                            climbMtrLdr.set(0);
                                            climbMtrLdr.setPosition(0);
                                            climberInitialized = true;
                                        }},
                                     interrupted -> climbMtrLdr.set(0),
                                     () -> climberInitialized,
                                     this);
    }
}