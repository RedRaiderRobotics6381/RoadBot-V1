package frc.robot.subsystems.Secondary;

import frc.robot.Robot;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class RotateSubsystem extends SubsystemBase {
    
    public TalonFX armAngMtr;
    private MotionMagicVoltage motionMagicVoltage;

    private double angkP = 0.010, angkI = 0.0, angkD = 0.15;// p was 0.002
    private double angkFF = 0.0; // 0.0075
    private double angOutputMin = -0.5;
    private double angOutputMax = 1.0;
    public boolean close;


    public RotateSubsystem() {
        armAngMtr = new TalonFX(RotateConstants.ROTATE_MOTOR_PORT);
        TalonFXConfiguration armAngMtrCfg = new TalonFXConfiguration();
        motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);

        armAngMtrCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armAngMtrCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        armAngMtrCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        armAngMtrCfg.CurrentLimits.StatorCurrentLimitEnable = true;

        armAngMtrCfg.CurrentLimits.SupplyCurrentLimit = 30.0;
        armAngMtrCfg.CurrentLimits.StatorCurrentLimit = 50.0;

        armAngMtrCfg.Slot0.kP = 5.0;
        armAngMtrCfg.Slot0.kI = 0;
        armAngMtrCfg.Slot0.kD = 0;

        armAngMtr.getConfigurator().apply(armAngMtrCfg);

       
    }

    public FunctionalCommand setRotateAngleCmd(double pos) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setRotateAngle(pos), interrupted -> {
                },
                () -> (Math.abs(pos - armAngMtr.getPosition().getValueAsDouble()) <= 2.0 || (Math.abs(pos - armAngMtr.getPosition().getValueAsDouble()) <= 4.0 && Math.abs(armAngMtr.getVelocity().getValueAsDouble()) <= 5.0)),
                this);
    }
    public void setRotateAngle(double angle) {
        armAngMtr.setControl(motionMagicVoltage.withPosition(angle));
    }

    public void periodic() {
        // This method will be called once per scheduler run
    
            SmartDashboard.putNumber("Fuel Arm Position", armAngMtr.getPosition().getValueAsDouble());
            SmartDashboard.putNumber("Fuel Arm Speed", armAngMtr.getVelocity().getValueAsDouble());
    
    }
}

