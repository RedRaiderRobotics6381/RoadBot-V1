package frc.robot.subsystems.secondary;
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

public class Intake extends SubsystemBase {
    private SparkFlex armVelMtr;
    public AbsoluteEncoder armVelEnc;
    public SparkClosedLoopController armVelPID;
    private SparkFlexSim armVelSim;
    private SparkAbsoluteEncoderSim armVelEncSim;
    private SparkFlexConfig armVelMtrCfg;

    private double velkP = 1.00, velkI = 0.0, velkD = 0.00;// p was 0.002
    private double velkFF = 0.0; // 0.0075
    private double velOutputMin = 250;
    private double velOutputMax = 700;
    public boolean close;

public Intake() {
    armVelMtr = new SparkFlex(IntakeConstants.INTAKE_MOTOR_PORT, MotorType.kBrushless);
    armVelMtrCfg = new SparkFlexConfig();

    armVelPID = armVelMtr.getClosedLoopController();
    armVelEnc = armVelMtr.getAbsoluteEncoder();

    armVelMtrCfg
            .inverted(false)
            .voltageCompensation(12.0)
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kCoast);
    armVelMtrCfg.absoluteEncoder
            .inverted(false);
    armVelMtrCfg.closedLoop
            .pid(velkP, velkI, velkD)
            .outputRange(velOutputMin, velOutputMax)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    armVelMtr.configure(armVelMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

}

public FunctionalCommand setIntakeCmd(double vel) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setVel(vel), interrupted -> {
                },
                () -> (Math.abs(vel - armVelEnc.getVelocity()) <= 50),
                this);
    }

    public void setVel(double vel) {
        armVelPID.setReference(vel, SparkMax.ControlType.kVelocity);

}
}
