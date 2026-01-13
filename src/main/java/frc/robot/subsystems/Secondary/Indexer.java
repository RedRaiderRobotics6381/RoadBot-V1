package frc.robot.subsystems.Secondary;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Indexer extends SubsystemBase {

  private final VoltageOut voltageCntrl;
  public final TalonFX indexMtrLdr;
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