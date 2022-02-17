package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private final CANSparkMax m_motor1 = new CANSparkMax(3,MotorType.kBrushless);
    private final CANSparkMax m_motor2 = new CANSparkMax(4,MotorType.kBrushless);
    private final CANSparkMax m_hoodMotor = new CANSparkMax(5,MotorType.kBrushless);
    private final RelativeEncoder m_encoder1 = m_motor1.getEncoder();
    private final RelativeEncoder m_encoder2 = m_motor2.getEncoder();
    private final RelativeEncoder m_hoodEncoder = m_hoodMotor.getEncoder();
    private final PIDController m_PID = new PIDController(0.00005, 0.0003, 0.0);
    private SimpleMotorFeedforward m_FF = new SimpleMotorFeedforward(0.018, 0.0001635);
    private final SparkMaxPIDController m_hoodPID = m_hoodMotor.getPIDController();
    
    private final DigitalInput m_hoodLimit = new DigitalInput(0);

    private double m_RPM = 3000.0;
    private double m_hoodAngle = 15.0;


    public Shooter() {
        m_motor1.setSmartCurrentLimit(40);
        m_motor2.setSmartCurrentLimit(40);
        m_hoodMotor.setSmartCurrentLimit(20);
        m_motor1.enableVoltageCompensation(12.6);
        m_motor2.enableVoltageCompensation(12.6);
        m_hoodMotor.enableVoltageCompensation(12.6);
        m_motor1.setIdleMode(IdleMode.kCoast);
        m_motor2.setIdleMode(IdleMode.kCoast);
        m_hoodMotor.setIdleMode(IdleMode.kBrake);
        
        m_hoodEncoder.setPositionConversionFactor(1.333333);

        m_hoodPID.setP(0.25);
        m_hoodPID.setI(0.0);
        m_hoodPID.setIMaxAccum(0.0003, 0);
        m_hoodPID.setD(0.0);
        m_hoodPID.setFF(0.00);

        m_hoodPID.setOutputRange(-0.33, 1.0);

        m_motor1.setInverted(false);
        m_hoodMotor.setInverted(false);
        m_motor2.follow(m_motor1,true);

        m_motor1.burnFlash();
        m_motor2.burnFlash();
        m_hoodMotor.burnFlash();

        m_PID.setTolerance(50.0);
        m_PID.setIntegratorRange(-0.01, 0.01);

        SmartDashboard.putNumber("SetShotRPM", m_RPM);
        SmartDashboard.putNumber("SetHoodAngle", m_hoodAngle);
        SmartDashboard.putBoolean("Hood Limit", getHoodLimit());
        SmartDashboard.putBoolean("Shooter PID Reset", false);
        SmartDashboard.putNumber("Shooter P", m_PID.getP());
        SmartDashboard.putNumber("Shooter I", m_PID.getI());
        SmartDashboard.putNumber("Shooter FF", m_FF.kv);
        SmartDashboard.putNumber("Shooter Static", m_FF.ks);
        SmartDashboard.putNumber("Shooter Max I Power", 0.02);

    }

    public void run() {
        double outputPID = m_PID.calculate(m_encoder1.getVelocity(), m_RPM);
        double outputFF = m_FF.calculate(m_RPM);
        m_motor1.set(outputPID+outputFF);
        m_hoodPID.setReference(m_hoodAngle, ControlType.kPosition);
    }

    public void setHood(double speed){
        m_hoodMotor.set(speed);
    }

    public void setHoodAngle(double angle){
        m_hoodAngle = angle;
        if(m_hoodAngle < 0.0){
            m_hoodAngle = 0.0;
        }
        else if(m_hoodAngle > 26.0){
            m_hoodAngle = 26.0;
        }
    }

    public void setHoodZero(){
        m_hoodEncoder.setPosition(-0.5);
        m_hoodMotor.stopMotor();
    }

    public boolean getHoodLimit(){
        return m_hoodLimit.get();
    }

    public double getHoodAngle(){
        return m_hoodEncoder.getPosition();
    }

    public void stop() {
        m_motor1.stopMotor();
        m_motor2.stopMotor();
        m_hoodMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", m_encoder1.getVelocity());
        SmartDashboard.putNumber("Shooter RPM2", m_encoder2.getVelocity());
        SmartDashboard.putNumber("Shooter Current", m_motor1.getOutputCurrent()+m_motor2.getOutputCurrent());
        SmartDashboard.putNumber("HoodAngle", m_hoodEncoder.getPosition());
        m_RPM = SmartDashboard.getNumber("SetShotRPM", 3000.0);
        setHoodAngle(SmartDashboard.getNumber("SetHoodAngle", 15.0));

        if(SmartDashboard.getBoolean("Shooter PID Reset", false)){
            double kv = SmartDashboard.getNumber("Shooter FF", 0.00016);
            double ks = SmartDashboard.getNumber("Shooter Static", 0.02);
            double integratePower = SmartDashboard.getNumber("Shooter Max I Power", 0.02);
            m_PID.setP(SmartDashboard.getNumber("Shooter P", 0.0001));
            m_PID.setI(SmartDashboard.getNumber("Shooter I", 0.0001));
            m_PID.setIntegratorRange(-integratePower, integratePower);
            m_FF = new SimpleMotorFeedforward(ks, kv);
            SmartDashboard.putBoolean("Shooter PID Reset", false);
        }




    }

    public boolean atSetpoint() {
        return m_PID.atSetpoint();
    }
    
}
