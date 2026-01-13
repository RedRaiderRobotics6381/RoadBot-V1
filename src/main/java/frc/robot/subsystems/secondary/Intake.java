package frc.robot.subsystems.Secondary;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final VoltageOut voltageCntrl;
    private final TalonFX intVelMtr;
//     private final TalonFX armVelSim;
//     private TalonFX armVelEncSim;

    private double velkP = 1.00, velkI = 0.0, velkD = 0.00;// p was 0.002
    private double velkFF = 0.0; // 0.0075
    private double velOutputMin = 250;
    private double velOutputMax = 700;
    public boolean close;

public Intake() {

    voltageCntrl = new VoltageOut(0.0);
    TalonFXConfiguration intVelMtrCfg = new TalonFXConfiguration();

    intVelMtr = new TalonFX(IntakeConstants.INTAKE_MOTOR_PORT);

    intVelMtrCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intVelMtrCfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intVelMtrCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
    intVelMtrCfg.CurrentLimits.StatorCurrentLimitEnable = true;

    intVelMtrCfg.CurrentLimits.SupplyCurrentLimit = 30.0;
    intVelMtrCfg.CurrentLimits.StatorCurrentLimit = 50.0;

    intVelMtrCfg.Slot0.kP = 5.0;
    intVelMtrCfg.Slot0.kI = 0;
    intVelMtrCfg.Slot0.kD = 0;

    intVelMtr.getConfigurator().apply(intVelMtrCfg);

}


    public void setVoltage(double volt) {
        intVelMtr.setControl(voltageCntrl.withOutput(volt));
}



}