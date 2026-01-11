
package frc.robot.subsystems.Secondary;

//import frc.robot.Robot;
//import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.sim.SparkAbsoluteEncoderSim;
//import com.revrobotics.sim.SparkFlexSim;
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

public class Turret extends SubsystemBase{
    
private SparkFlex turretAngMtr;
public AbsoluteEncoder turretAngEnc;
public SparkClosedLoopController turretAngPID;
//private SparkFlexSim turretAngMtrSim;
//private SparkAbsoluteEncoderSim turretAngEncSim;
private SparkFlexConfig turretAngMtrCfg;

private double angkP = 0.010, angkI = 0.0, angkD = 0.15;// p was 0.002
private double angkFF = 0.0; // 0.0075
private double angOutputMin = -0.5;
private double angOutputMax = 1.0;
public boolean close;

public Turret() {

turretAngMtr = new SparkFlex(TurretConstants.TURRET_CONSTANT, MotorType.kBrushless);
turretAngMtrCfg = new SparkFlexConfig();

turretAngPID = turretAngMtr.getClosedLoopController();
 
turretAngEnc = turretAngMtr.getAbsoluteEncoder();

     turretAngMtrCfg
                .inverted(false)
                .voltageCompensation(12.0)
                .smartCurrentLimit(50)
                .idleMode(IdleMode.kBrake);
        turretAngMtrCfg.absoluteEncoder
                .positionConversionFactor(360)
                .inverted(false)
                .zeroOffset(0.34722222);
        turretAngMtrCfg.softLimit
                .forwardSoftLimit(200.0)
                .reverseSoftLimit(60.0)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);
        turretAngMtrCfg.closedLoop
                .pid(angkP, angkI, angkD)
                .outputRange(angOutputMin, angOutputMax)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        turretAngMtr.configure(turretAngMtrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    }

    // public void runIntakeCmd(double speed) {
    // coralIndexMtr.set(speed);
    // }

    /**
     * Creates a FunctionalCommand to initialize the slider mechanism.
     * 
     * The command performs the following actions:
     * 1. Initializes the sliderInitialized flag to false.
     * 2. Continuously checks the state of the limit switch:
     * - If the limit switch is not triggered, the coralSliderMtr motor is set to
     * move the slider.
     * - If the limit switch is triggered, the coralSliderMtr motor is stopped and
     * the sliderInitialized flag is set to true.
     * 3. Stops the coralSliderMtr motor if the command is interrupted.
     * 4. Ends the command when the sliderInitialized flag is true.
     * 
     * @return A new FunctionalCommand instance for initializing the slider.
     */
    

    /**
     * Creates a new FunctionalCommand for the intake mechanism.
     * 
     * The command performs the following actions:
     * - Initializes with an empty lambda function.
     * - Executes the intake mechanism with a speed of 0.1.
     * - Ends the intake mechanism by stopping it (setting speed to 0) when
     * interrupted.
     * - Checks if the beam break sensor is triggered to determine if the command is
     * finished.
     * 
     * @return A new instance of FunctionalCommand for controlling the intake
     *         mechanism.
     */

    // public FunctionalCommand algaeOuttakeCmd() {
    // return new FunctionalCommand(() ->{},
    // () -> indexMtrLdr.set(-0.06),
    // interrupted -> indexMtrLdr.set(0),
    // () -> {},
    // this);
    // }

    public FunctionalCommand setTurretCmd(double pos) {
        return new FunctionalCommand(
                () -> {
                },
                () -> setTurret(pos), interrupted -> {
                },
                () -> (Math.abs(pos - turretAngEnc.getPosition()) <= 2.0 || (Math.abs(pos - turretAngEnc.getPosition()) <= 4.0 && Math.abs(turretAngEnc.getVelocity()) <= 5.0)),
                this);
    }

    public void setTurret(double angle) {
        turretAngPID.setReference(angle, SparkMax.ControlType.kPosition);

        // From Minibot
        // This is an arbitrary feedforward value that is multiplied by the positon of
        // the arm to account
        // for the reduction in force needed to hold the arm vertical instead of
        // hortizontal. The .abs
        // ensures the value is always positive. The .cos function uses radians instead
        // of degrees,
        // so the .toRadians converts from degrees to radians.
        // Use this to add a feed forward value to the arm to hold it horizontal - test
        // test test!
        // Increase angkFF with the arm horizontal until just before it starts to drift
        // upward
       

        // if (Robot.isSimulation()) {
        // coralRotatePID.setReference(angle, SparkMax.ControlType.kPosition);
        // }
    }

    // public void extend() {
    // pusherServo.set(1);
    // }

    





}







