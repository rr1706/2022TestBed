package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Intake extends SubsystemBase {
    private final CANSparkMax m_motor = new CANSparkMax(6,MotorType.kBrushless);
    private final RelativeEncoder m_encoder = m_motor.getEncoder();
    private final SparkMaxPIDController m_PID = m_motor.getPIDController();
    private double m_RPM = 1000.0;
    private final DoubleSolenoid m_actuator = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 0, 1);

    public Intake(){
        m_motor.setSmartCurrentLimit(25);
        m_motor.enableVoltageCompensation(12.0);
        m_motor.setIdleMode(IdleMode.kCoast);
        m_PID.setOutputRange(-1.0, 1.0);
        m_PID.setP(0.00005);
        m_PID.setI(0.0);
        m_PID.setD(0.0);
        m_PID.setFF(0.000091);
        m_motor.burnFlash();

        SmartDashboard.putNumber("IntakeP", m_PID.getP());
        SmartDashboard.putNumber("IntakeI", m_PID.getI());
        SmartDashboard.putNumber("IntakeD", m_PID.getD());
        SmartDashboard.putNumber("IntakeFF", m_PID.getFF());
        SmartDashboard.putNumber("SetIntakeRPM", m_RPM);
        SmartDashboard.putBoolean("IntakeBurnFlash", false);
    }

    public void run() {
        m_PID.setReference(m_RPM, ControlType.kVelocity);
    }

    public void extend(){
        m_actuator.set(Value.kForward);
    }
    public void retract(){
        m_actuator.set(Value.kReverse);
    }
    public void stop(){
        m_motor.stopMotor();
        m_actuator.set(Value.kReverse);
    }

    public double getCurrent(){
       return m_motor.getOutputCurrent();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Current", m_motor.getOutputCurrent());
        SmartDashboard.putNumber("Intake Motor Temp", m_motor.getMotorTemperature());
        SmartDashboard.putNumber("Intake RPM", m_encoder.getVelocity());
        m_RPM = SmartDashboard.getNumber("SetIntakeRPM", 1000.0);
        final double P = SmartDashboard.getNumber("IntakeP", 0.000175);
        final double I = SmartDashboard.getNumber("IntakeI", 0.0);
        final double D = SmartDashboard.getNumber("IntakeD", 0.0);
        final double V = SmartDashboard.getNumber("IntakeFF", 0.000091);
        SmartDashboard.putNumber("SetIntakeRPM", m_RPM);
        SmartDashboard.putNumber("IntakeP", P);
        SmartDashboard.putNumber("IntakeI", I);
        SmartDashboard.putNumber("IntakeD", D);
        SmartDashboard.putNumber("IntakeFF", V);

        final boolean change = SmartDashboard.getBoolean("IntakeBurnFlash", false);
        SmartDashboard.putBoolean("IntakeBurnFlash", change);
        if(change){
            if(m_PID.getP()!=P){
                m_PID.setP(P);
            }
            if(m_PID.getI()!=I){
                m_PID.setP(I);
            }
            if(m_PID.getD()!=D){
                m_PID.setD(D);
            }
            if(m_PID.getFF() != V ){
                m_PID.setFF(V);
            }
                m_motor.burnFlash();
                SmartDashboard.putBoolean("IntakeBurnFlash", false);
        }
    }

}
