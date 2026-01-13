package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.subsystems.Secondary.Indexer;
import frc.robot.subsystems.Secondary.Outtake;
import frc.robot.subsystems.Secondary.RotateSubsystem;
import frc.robot.subsystems.Secondary.Turret;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class Autoshooter extends Command {


    private final Indexer indexer;
    private final RotateSubsystem rotateSubsystem;
    private final Outtake outtake;
    private final CommandSwerveDrivetrain drivetrain;
    private final Turret turret;


    public double rotateAngle = 0;
    public double yaw = 0;
    public int velocity = 0;

    public double distance;

    public Autoshooter(Indexer indexer, RotateSubsystem rotateSubsystem, Outtake outtake, CommandSwerveDrivetrain drivetrain, Turret turret) {

        this.drivetrain = drivetrain;
        this.rotateSubsystem = rotateSubsystem;
        this.indexer = indexer;
        this.outtake = outtake;
        this.turret = turret;

    }
    @Override
    public void initialize() {
        outtake.setVelocity(velocity);
    }


    @Override
    public void execute() {
        distance = Math.sqrt(Math.pow(182.1 - drivetrain.getState().Pose.getX(), 2) + Math.pow(158.5 - drivetrain.getState().Pose.getY(), 2));
        rotateAngle = Math.atan((72 - PhysicalConstants.SHOOTER_HEIGHT) / distance);
        yaw = Math.atan((158.5 - drivetrain.getState().Pose.getY()) / (182.1 - drivetrain.getState().Pose.getX()));
        turret.setTurretCmd(yaw);
        rotateSubsystem.setRotateAngleCmd(rotateAngle);
        if (Math.abs(rotateSubsystem.armAngEnc.getPosition() - rotateAngle) <= .5 && Math.abs(outtake.wheelSpeedEnc.getVelocity() - velocity) <=30) {
            indexer.indexMtrLdr.set(1);
        }
    }

    @Override
    public void end(boolean interrupted) {
        outtake.setVelocity(0);
        indexer.indexMtrLdr.set(0);
    }
}
