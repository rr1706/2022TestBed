package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ZeroHood extends CommandBase {
    private Shooter m_shooter;


    public ZeroHood(Shooter shooter){
        m_shooter = shooter;
        addRequirements(m_shooter);
    }
    @Override
    public void initialize(){
        m_shooter.setHood(-0.040);
    }

    @Override
    public void execute(){
        if(m_shooter.getHoodLimit()){
            m_shooter.setHoodZero();
            m_shooter.setHood(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
      m_shooter.setHoodAngle(0);
      m_shooter.setHood(0.0);
    }
}
