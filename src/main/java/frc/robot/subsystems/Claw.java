package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Centimeters;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class Claw extends SubsystemIO {
    public enum AlgaeModes {
        STOP(0),
        HOLD(20),
        INTAKE(40),
        OUTTAKE(-80);

        public final double current;

        private AlgaeModes(double current) {
            this.current = current;
        }
    }

    public enum CoralModes {
        STOP(0),
        HOLD(20),
        INTAKE(60),
        OUTTAKE(-40),
        OUTTAKE_SLOW(-.08); 

        public final double current;

        private CoralModes(double current) {
            this.current = current;
        }
    }

    private TalonFX m_Coral;
    private TalonFX m_Algae;

    private CANrange m_CoralSensor;
    private CANrange m_AlgaeSensor;

    private final TorqueCurrentFOC m_CurrentRequest = new TorqueCurrentFOC(0);
    private final DutyCycleOut m_OutputRequest = new DutyCycleOut(0);

    public Claw() {
        m_CoralSensor = new CANrange(ClawConstants.kCoralSensorId, RobotConstants.kCanivoreBusName);

        CANrangeConfiguration coralSensorConfig = new CANrangeConfiguration();

        coralSensorConfig.FovParams.FOVCenterX = 0;
        coralSensorConfig.FovParams.FOVCenterY = 0;
        coralSensorConfig.FovParams.FOVRangeX = 20;
        coralSensorConfig.FovParams.FOVRangeY = 10;
        coralSensorConfig.ProximityParams.ProximityThreshold = .11;

        m_CoralSensor.getConfigurator().apply(coralSensorConfig);

        m_AlgaeSensor = new CANrange(ClawConstants.kAlgaeSensorId, RobotConstants.kCanivoreBusName);

        CANrangeConfiguration algaeSensorConfig = new CANrangeConfiguration();

        algaeSensorConfig.FovParams.FOVCenterX = 1;
        algaeSensorConfig.FovParams.FOVCenterY = 1;
        algaeSensorConfig.FovParams.FOVRangeX = 8;
        algaeSensorConfig.FovParams.FOVRangeY = 8;
        algaeSensorConfig.ProximityParams.ProximityThreshold = .2;

        m_AlgaeSensor.getConfigurator().apply(algaeSensorConfig);

        m_Algae = new TalonFX(ClawConstants.kAlgaeId, RobotConstants.kCanivoreBusName);

        TalonFXConfiguration algaeConfigs = new TalonFXConfiguration();

        algaeConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        algaeConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        m_Algae.getConfigurator().apply(algaeConfigs);

        m_Coral = new TalonFX(ClawConstants.kCoralId, RobotConstants.kCanivoreBusName);

        TalonFXConfiguration coralConfigs = new TalonFXConfiguration();

        coralConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        coralConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_Coral.getConfigurator().apply(coralConfigs);

        setAlgaeMode(AlgaeModes.HOLD);
        setCoralMode(CoralModes.HOLD);
    }

    public static class PeriodicIO {
        public CoralModes coralMode = CoralModes.STOP;

        public double firstSeenCoral = Double.MIN_VALUE;
        public double lastSeenCoral = Double.MIN_VALUE;
        public boolean hasCoral = false;
        public boolean latchCoralMode = false;
        public double timeoutCoral = Double.MIN_VALUE;
        public boolean longTimeoutCoral = false;
        public int tryAgainCoral = 0;

        public AlgaeModes algaeMode = AlgaeModes.STOP;

        public double firstSeenAlgae = Double.MIN_VALUE;
        public double lastSeenAlgae = Double.MIN_VALUE;
        public boolean hasAlgae = false;
        public boolean seeAlgae = false;
        public boolean latchAlgaeMode = false;
        public double timeoutAlgae = Double.MIN_VALUE;
        public boolean longTimeoutAlgae = false;
        public int tryAgainAlgae = 0;
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    public void setCoralMode(CoralModes mode) {
        setCoralMode(mode, false);
    }

    public void setCoralMode(CoralModes mode, boolean longTimeout) {
        m_PeriodicIO.coralMode = mode;
        m_PeriodicIO.longTimeoutCoral = longTimeout;
        m_PeriodicIO.timeoutCoral = Timer.getFPGATimestamp();

        if (!m_PeriodicIO.latchCoralMode) {
            m_PeriodicIO.firstSeenCoral = Double.MIN_VALUE;
            m_PeriodicIO.lastSeenCoral = Double.MIN_VALUE;
        }

        if (mode == CoralModes.INTAKE && m_PeriodicIO.hasCoral
                && Timer.getFPGATimestamp() - m_PeriodicIO.firstSeenCoral > 1.5) {
            m_PeriodicIO.firstSeenCoral = Double.MIN_VALUE;
        }
    }

    public void setAlgaeMode(AlgaeModes mode) {
        setAlgaeMode(mode, false);
    }

    public void setAlgaeMode(AlgaeModes mode, boolean longTimeout) {
        m_PeriodicIO.algaeMode = mode;
        m_PeriodicIO.longTimeoutAlgae = longTimeout;
        m_PeriodicIO.timeoutAlgae = Timer.getFPGATimestamp();

        if (!m_PeriodicIO.latchAlgaeMode) {
            m_PeriodicIO.firstSeenAlgae = Double.MIN_VALUE;
            m_PeriodicIO.lastSeenAlgae = Double.MIN_VALUE;
        }

        if (mode == AlgaeModes.INTAKE) {
            m_PeriodicIO.latchAlgaeMode = true;
        }

        if (mode == AlgaeModes.INTAKE && m_PeriodicIO.hasAlgae
                && Timer.getFPGATimestamp() - m_PeriodicIO.firstSeenAlgae > 1.5) {
            m_PeriodicIO.firstSeenAlgae = Double.MIN_VALUE;
        }
    }

    public boolean hasAlgae() {
        return m_PeriodicIO.hasAlgae;
    }

    public boolean seeAlgae() {
        return m_PeriodicIO.seeAlgae;
    }

    public boolean hasCoral() {
        return m_PeriodicIO.hasCoral;
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        throw new UnsupportedOperationException("Unimplemented method 'checkSystem'");
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Claw/SeeAlgae", m_PeriodicIO.seeAlgae);
        SmartDashboard.putBoolean("Claw/HasAlgae", m_PeriodicIO.hasAlgae);
        SmartDashboard.putBoolean("Claw/HasCoral", m_PeriodicIO.hasCoral);
        SmartDashboard.putString("Claw/AlgaeMode", m_PeriodicIO.algaeMode.name());
        SmartDashboard.putString("Claw/CoralMode", m_PeriodicIO.coralMode.name());
        SmartDashboard.putNumber("Claw/CoralTryAgain", m_PeriodicIO.tryAgainCoral);
        SmartDashboard.putNumber("Claw/AlgaeTryAgain", m_PeriodicIO.tryAgainAlgae);

        SignalLogger.writeBoolean("Claw/SeeAlgae", m_PeriodicIO.seeAlgae);
        SignalLogger.writeBoolean("Claw/HasAlgae", m_PeriodicIO.hasAlgae);
        SignalLogger.writeBoolean("Claw/HasCoral", m_PeriodicIO.hasCoral);
        SignalLogger.writeString("Claw/AlgaeMode", m_PeriodicIO.algaeMode.name());
        SignalLogger.writeString("Claw/CoralMode", m_PeriodicIO.coralMode.name());
        SignalLogger.writeDouble("Claw/CoralTryAgain", m_PeriodicIO.tryAgainCoral);
        SignalLogger.writeDouble("Claw/AlgaeTryAgain", m_PeriodicIO.tryAgainAlgae);
    }

    @Override
    public void readPeriodicInputs() {
        m_PeriodicIO.hasCoral = m_CoralSensor.getDistance().getValue()
                .in(Centimeters) < ClawConstants.kCoralDistanceThreshold && m_CoralSensor.getIsDetected().getValue();
        if (m_PeriodicIO.hasCoral && m_PeriodicIO.firstSeenCoral == Double.MIN_VALUE) {
            m_PeriodicIO.firstSeenCoral = Timer.getFPGATimestamp();
            m_PeriodicIO.lastSeenCoral = Double.MIN_VALUE;
        } else if (!m_PeriodicIO.hasCoral && m_PeriodicIO.firstSeenCoral != Double.MIN_VALUE) {
            m_PeriodicIO.firstSeenCoral = Double.MIN_VALUE;
            m_PeriodicIO.lastSeenCoral = Timer.getFPGATimestamp();
        }
        
        m_PeriodicIO.seeAlgae = m_AlgaeSensor.getIsDetected().getValue();
        m_PeriodicIO.hasAlgae = m_AlgaeSensor.getDistance().getValue()
                .in(Centimeters) < ClawConstants.kAlgaeDistanceThreshold && m_PeriodicIO.seeAlgae;
                
        if (m_PeriodicIO.hasAlgae && m_PeriodicIO.firstSeenAlgae == Double.MIN_VALUE) {
            m_PeriodicIO.firstSeenAlgae = Timer.getFPGATimestamp();
            m_PeriodicIO.lastSeenAlgae = Double.MIN_VALUE;
        } else if (!m_PeriodicIO.hasAlgae && m_PeriodicIO.firstSeenAlgae != Double.MIN_VALUE) {
            m_PeriodicIO.firstSeenAlgae = Double.MIN_VALUE;
            m_PeriodicIO.lastSeenAlgae = Timer.getFPGATimestamp();
        }
    }

    @Override
    public void writePeriodicOutputs() {

        // Algae logic
        if (m_PeriodicIO.algaeMode != AlgaeModes.STOP || m_PeriodicIO.hasAlgae) {
            double timeout = .5;

            if (m_PeriodicIO.algaeMode == AlgaeModes.INTAKE) {
                if (m_PeriodicIO.longTimeoutAlgae) {
                    timeout = 1;
                }
                if (hasAlgae() && Timer.getFPGATimestamp() - m_PeriodicIO.firstSeenAlgae > 1) {
                    m_PeriodicIO.algaeMode = AlgaeModes.HOLD;
                } else if (!m_PeriodicIO.latchAlgaeMode
                        && Timer.getFPGATimestamp() - m_PeriodicIO.timeoutAlgae > timeout) {
                    m_PeriodicIO.algaeMode = AlgaeModes.STOP;
                }
            } else if (m_PeriodicIO.algaeMode == AlgaeModes.HOLD) {
                if (!hasAlgae()) {
                    if (Timer.getFPGATimestamp() - m_PeriodicIO.timeoutAlgae > timeout) {
                        if (m_PeriodicIO.tryAgainAlgae < 5) {
                            m_PeriodicIO.timeoutAlgae = Timer.getFPGATimestamp();
                            m_PeriodicIO.longTimeoutAlgae = true;
                            m_PeriodicIO.algaeMode = AlgaeModes.INTAKE;
                            m_PeriodicIO.tryAgainAlgae++;
                        } else {
                            m_PeriodicIO.tryAgainAlgae = 0;
                            m_PeriodicIO.algaeMode = AlgaeModes.STOP;
                        }
                    } else {
                        m_PeriodicIO.algaeMode = AlgaeModes.STOP;
                    }
                }
                else {
                    m_PeriodicIO.tryAgainAlgae = 0;
                }
            } else if (m_PeriodicIO.algaeMode == AlgaeModes.OUTTAKE && !hasAlgae() && !m_PeriodicIO.latchAlgaeMode) {
                if (Timer.getFPGATimestamp() - m_PeriodicIO.timeoutAlgae > timeout) {
                    m_PeriodicIO.algaeMode = AlgaeModes.STOP;
                }
            } else if (m_PeriodicIO.algaeMode == AlgaeModes.STOP && hasAlgae()) {
                m_PeriodicIO.algaeMode = AlgaeModes.HOLD;
            }

            if(m_PeriodicIO.algaeMode != AlgaeModes.OUTTAKE && m_PeriodicIO.seeAlgae) {
                if(m_PeriodicIO.hasAlgae) {
                    m_PeriodicIO.algaeMode = AlgaeModes.HOLD;
                } else {
                    m_PeriodicIO.algaeMode = AlgaeModes.INTAKE;
                }
            }

            m_Algae.setControl(m_CurrentRequest.withOutput(m_PeriodicIO.algaeMode.current));
        } else {
            if(m_PeriodicIO.seeAlgae) {
                if(m_PeriodicIO.hasAlgae) {
                    m_PeriodicIO.algaeMode = AlgaeModes.HOLD;
                } else {
                    m_PeriodicIO.algaeMode = AlgaeModes.INTAKE;
                }
                m_Algae.setControl(m_CurrentRequest.withOutput(m_PeriodicIO.algaeMode.current));
            }
            else {
                m_Algae.stopMotor();
            }
        }

        if (m_PeriodicIO.latchAlgaeMode) {
            m_PeriodicIO.latchAlgaeMode = false;
        }

        // Coral logic
        if (m_PeriodicIO.coralMode != CoralModes.STOP || m_PeriodicIO.hasCoral) {
            double timeout = .5;

            if (m_PeriodicIO.coralMode == CoralModes.INTAKE) {
                if (m_PeriodicIO.longTimeoutAlgae) {
                    timeout = 1;
                }
                if (hasCoral() && Timer.getFPGATimestamp() - m_PeriodicIO.firstSeenCoral > .5) {
                    m_PeriodicIO.coralMode = CoralModes.HOLD;
                } else if (!m_PeriodicIO.latchCoralMode
                        && Timer.getFPGATimestamp() - m_PeriodicIO.timeoutCoral > timeout) {
                    m_PeriodicIO.coralMode = CoralModes.STOP;
                }
            } else if (m_PeriodicIO.coralMode == CoralModes.HOLD && !hasCoral()) {
                if (Timer.getFPGATimestamp() - m_PeriodicIO.timeoutCoral > timeout) {
                    if (m_PeriodicIO.tryAgainCoral < 5) {
                        m_PeriodicIO.timeoutCoral = Timer.getFPGATimestamp();
                        m_PeriodicIO.coralMode = CoralModes.INTAKE;
                        m_PeriodicIO.longTimeoutCoral = true;
                        m_PeriodicIO.tryAgainCoral++;
                    } else {
                        m_PeriodicIO.tryAgainCoral = 0;
                        m_PeriodicIO.coralMode = CoralModes.STOP;
                    }
                } else {
                    m_PeriodicIO.coralMode = CoralModes.STOP;
                }
            } else if ((m_PeriodicIO.coralMode == CoralModes.OUTTAKE || m_PeriodicIO.coralMode == CoralModes.OUTTAKE_SLOW) && !hasCoral() && !m_PeriodicIO.latchCoralMode) {
                if (Timer.getFPGATimestamp() - m_PeriodicIO.timeoutCoral > timeout) {
                    m_PeriodicIO.coralMode = CoralModes.STOP;
                }
            } else if (m_PeriodicIO.coralMode == CoralModes.STOP && hasCoral()) {
                m_PeriodicIO.coralMode = CoralModes.HOLD;
            }

            if(m_PeriodicIO.coralMode == CoralModes.OUTTAKE_SLOW) {
                m_Coral.setControl(m_OutputRequest.withOutput(m_PeriodicIO.coralMode.current));
            }
            else{
                m_Coral.setControl(m_CurrentRequest.withOutput(m_PeriodicIO.coralMode.current));
            }
        } else {
            m_Coral.stopMotor();
        }

        if (m_PeriodicIO.latchCoralMode) {
            m_PeriodicIO.latchCoralMode = false;
        }
    }
}
