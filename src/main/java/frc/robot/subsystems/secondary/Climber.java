package frc.robot.subsystems.Secondary;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Robot;

public class Climber extends SubsystemBase {
   public SparkFlex elevMtrLdr;
    public SparkFlex elevMtrFlw;
    private SparkFlexConfig ldrCfg;
    private SparkFlexConfig flwCfg;
    public RelativeEncoder elevEncLdr;
    public RelativeEncoder elevEncFlw;
    public SparkClosedLoopController  elevPIDLdr;
    public SparkClosedLoopController  elevPIDFlw;
    private SparkFlexSim elevMtrLdrSim;
    private SparkFlexSim elevMtrFlwSim;
    private SparkRelativeEncoderSim elevEncLdrSim;
    private SparkRelativeEncoderSim elevEncFlwSim;
    private double kP = 0.15; //start p = 0.0005
    private double kD = 0.075;
    private double kOutput = 1.0;
    public DigitalInput limitSw;
    private boolean elevatorInitialized;

    public Climber() {
        elevMtrLdr = new SparkFlex(ClimberConstants.LEFT_CLIMBER_MOTOR_PORT, MotorType.kBrushless);
        elevMtrFlw = new SparkFlex(ClimberConstants.RIGHT_CLIMBER_MOTOR_PORT, MotorType.kBrushless);
     limitSw = new DigitalInput(9);

        ldrCfg = new SparkFlexConfig();
        flwCfg = new SparkFlexConfig();

        elevPIDLdr = elevMtrLdr.getClosedLoopController();
        elevPIDFlw = elevMtrFlw.getClosedLoopController();

        elevEncLdr = elevMtrLdr.getEncoder();
        elevEncFlw = elevMtrFlw.getEncoder();

        ldrCfg
            .inverted(true)
            .voltageCompensation(12.0)
            .smartCurrentLimit(80)
            .idleMode(IdleMode.kBrake);
        ldrCfg
            .encoder
                .positionConversionFactor(0.225); //confirm conversion factor
        ldrCfg
            .softLimit
                .forwardSoftLimit(24.0) 
                .reverseSoftLimit(-1.0)
                .forwardSoftLimitEnabled(true)
                .reverseSoftLimitEnabled(true);
        ldrCfg
            .limitSwitch
            .reverseLimitSwitchType(Type.kNormallyOpen)
            .reverseLimitSwitchEnabled(true);
        ldrCfg
            .closedLoop
                // .pidf(kLdrP, kLdrI, kLdrD, kLdrFF)
                .p(kP)
                .d(kD)
                .outputRange(-kOutput, kOutput)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder); 
                
                elevMtrLdr.configure(ldrCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

                flwCfg
                    .follow(elevMtrLdr, false)
                    .voltageCompensation(12.0)
                    .smartCurrentLimit(80)
                    .idleMode(IdleMode.kBrake);
                elevMtrFlw.configure(flwCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
                // Add motors to the simulation
                if (Robot.isSimulation()) {
                    elevMtrLdrSim = new SparkFlexSim(elevMtrLdr, DCMotor.getNEO(1));
                    elevMtrFlwSim = new SparkFlexSim(elevMtrFlw, DCMotor.getNEO(1));
                    elevEncLdrSim = new SparkRelativeEncoderSim(elevMtrLdr);
                    elevEncFlwSim = new SparkRelativeEncoderSim(elevMtrFlw);
                    elevMtrLdrSim.setPosition(0);
                    elevMtrFlwSim.setPosition(0);
                    elevEncLdrSim.setVelocity(0);
                    elevEncFlwSim.setVelocity(0);
                }
            }
            public void setElevatorHeight(double pos) {
                elevPIDLdr.setReference(pos, SparkMax.ControlType.kPosition);}
                public FunctionalCommand ElevatorHeightCmd(double height) {
                    return new FunctionalCommand(() -> {},
                        () -> setElevatorHeight(height),
                        interrupted -> {},
                        () -> Math.abs(height - elevEncLdr.getPosition()) <= 0.5,
                        this);
                }
            
                public FunctionalCommand ElevatorInitCmd() {
                    return new FunctionalCommand(() -> elevatorInitialized = false,
                                                    () -> {if(limitSw.get()){
                                                            elevMtrLdr.set(-.125);
                                                        } else if(!limitSw.get()) {
                                                            elevMtrLdr.set(0);
                                                            elevEncLdr.setPosition(0);
                                                            elevatorInitialized = true;
                                                        }},
                                                    interrupted -> elevMtrLdr.set(0),
                                                    () -> elevatorInitialized,
                                                    this);
                }
    }

