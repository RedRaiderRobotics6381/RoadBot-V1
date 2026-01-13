package frc.robot.subsystems.Secondary;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
//import frc.robot.Constants.OuttakeConstants;
import frc.robot.Robot;

public class Indexer extends SubsystemBase {

  private final VoltageOut voltageCntrl;
  private final TalonFX indexMtrLdr;
  private final TalonFX indexMtrFlw;


 
  
  public DigitalInput coralSensor;

  public Indexer() {

    voltageCntrl = new VoltageOut(0.0);
    TalonFXConfiguration indexMtrLdrCon = new TalonFXConfiguration();

    indexMtrLdr = new TalonFX(IntakeConstants.INTAKE_MOTOR_PORT);
    indexMtrFlw = new TalonFX(IntakeConstants.INTAKE_MOTOR_PORT);
    indexMtrFlw.setControl(new Follower(IntakeConstants.INTAKE_MOTOR_PORT, false));
    

    indexMtrLdrCon.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    indexMtrLdrCon.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    indexMtrLdrCon.CurrentLimits.SupplyCurrentLimitEnable = true;
    indexMtrLdrCon.CurrentLimits.StatorCurrentLimitEnable = true;

    indexMtrLdrCon.CurrentLimits.SupplyCurrentLimit = 30.0;
    indexMtrLdrCon.CurrentLimits.StatorCurrentLimit = 50.0;

    indexMtrLdrCon.Slot0.kP = 5.0;
    indexMtrLdrCon.Slot0.kI = 0;
    indexMtrLdrCon.Slot0.kD = 0;

    indexMtrLdr.getConfigurator().apply(indexMtrLdrCon);
    indexMtrFlw.getConfigurator().apply(indexMtrLdrCon);
    
    

    //coralSensor = new DigitalInput(CoralConstants.BEAM_BREAK_SENSOR_PORT);

  }



 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
      SmartDashboard.putNumber("Outtake Speed", indexMtrLdr.getVelocity().getValueAsDouble());
      SmartDashboard.putBoolean("CoralSensor", coralSensor.get());
  }

  public void setVoltage(double volt) {
    indexMtrLdr.setControl(voltageCntrl.withOutput(volt));
}
}