package frc.robot.subsystems.Secondary;

import frc.robot.Robot;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    
    private SparkFlex armAngMtr;
    public AbsoluteEncoder armAngEnc;
    public SparkClosedLoopController armAngPID;
    private SparkFlexSim armAngMtrSim;
    private SparkAbsoluteEncoderSim armAngEncSim;
    private SparkFlexConfig armAngMtrCfg;

    private double angkP = 0.010, angkI = 0.0, angkD = 0.15;// p was 0.002
    private double angkFF = 0.0; // 0.0075
    private double angOutputMin = -0.5;
    private double angOutputMax = 1.0;
    public boolean close;


    public RotateSubsystem() {
        armAngMtr = new SparkFlex(RotateConstants.ROTATE_MOTOR_PORT, MotorType.kBrushless);
        armAngMtrCfg = new SparkFlexConfig();

        armAngPID = armAngMtr.getClosedLoopController();
        armAngEnc = armAngMtr.getAbsoluteEncoder();

        armAngMtrCfg
                .inverted(false)
                .voltageCompensation(12.0)
                .smartCurrentLimit(50)
                .idleMode(IdleMode.kBrake);
        armAngMtrCfg.absoluteEncoder
                .positionConversionFactor(360)
                .inverted(false)
                .zeroOffset(0.34722222);
        armAngMtrCfg.softLimit
                .forwardSoftLimit(200.0)
                .reverseSoftLimit(60.0)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);
        armAngMtrCfg.closedLoop
                .pid(angkP, angkI, angkD)
                .outputRange(angOutputMin, angOutputMax)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        armAngMtr.configure(armAngMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Add motors to the simulation
        if (Robot.isSimulation()) {
            armAngMtrSim = new SparkFlexSim(armAngMtr, DCMotor.getNEO(1));
            armAngEncSim = new SparkAbsoluteEncoderSim(armAngMtr);
            armAngMtrSim.setPosition(190);
            armAngEncSim.setPosition(190);
            armAngMtrSim.setVelocity(0);
            armAngEncSim.setVelocity(0);
        }
    }

    public FunctionalCommand setRotateAngleCmd(double pos) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setRotateAngle(pos), interrupted -> {
                },
                () -> (Math.abs(pos - armAngEnc.getPosition()) <= 2.0 || (Math.abs(pos - armAngEnc.getPosition()) <= 4.0 && Math.abs(armAngEnc.getVelocity()) <= 5.0)),
                this);
    }
    public void setRotateAngle(double angle) {
        armAngPID.setReference(angle, SparkMax.ControlType.kPosition);

        armAngPID.setReference(angle,
        SparkMax.ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        angkFF * Math.abs(Math.cos(Math.toRadians(angle - 115))),
        ArbFFUnits.kPercentOut);
    }

    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        armAngEncSim.setPosition(armAngMtrSim.getPosition());
        armAngMtrSim.iterate(armAngEncSim.getPosition(), armAngMtrSim.getBusVoltage(), .005);
    }
    public void periodic() {
        // This method will be called once per scheduler run
        if (Robot.isSimulation()) {
            SmartDashboard.putNumber("Coarl Arm Position", armAngEncSim.getPosition());
        } else {
            SmartDashboard.putNumber("Coral Arm Position", armAngEnc.getPosition());
            SmartDashboard.putNumber("Coral Arm Speed", armAngEnc.getVelocity());
            // double distance = canrange.getDistance().getValueAsDouble();
            // close = distance < .43 && distance > .35;
            // SmartDashboard.putBoolean("canrange", close);
            // SmartDashboard.putNumber("canrange distance", distance);
        }
    }
}
}
