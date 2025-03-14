package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import frc.robot.Robot;
import frc.robot.RobotContainer;

import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix6.SignalLogger;

public class Candle extends SubsystemIO {
    private final CANdle m_candle = new CANdle(CandleConst.kId, RobotConstants.kCanivoreBusName);
    private final int LedCount = 99;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll
    }

    public enum State {
        DEFAULT, STROBE, SHOOTING, 
    }

    public static class PeriodicIO {
        State requestedState = State.DEFAULT;
        State lastRequState = null;
    }

    private final PeriodicIO m_PeriodicIO = new PeriodicIO();

    // private AnimationTypes m_animation = AnimationTypes.Twinkle;

    private Animation m_StrobeAnimation = new StrobeAnimation(0, 100, 0, 0, .00001, 60);
    // private Animation noah_Animation = new ColorFlowAnimation(255, 0, 255, 0, .1,
    // 60, null);

    

    public Candle() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }

    private void DefaultColor() {
        m_candle.setLEDs((int) (1 * 255),
                (int) (0 * 255),
                (int) (1 * 255));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // if(m_toAnimate == null) {
        // DefaultColor();
        // }
        // else {
        // m_candle.animate(m_toAnimation);
        // }

        // if (ShooterConst.Shooter.hasNote() == true) {
        // m_candle.animate(m_toAnimation);
        // } else {
        // m_candle.clearAnimation(0);
        // DefaultColor();
        // }
        // m_candle.modulateVBatOutput(0);
    }

    @Override
    public void writePeriodicOutputs() {
        // if (Claw.PeriodicIO.hasCoral() == true) {
        //     if(ShooterConst.Shooter.isShootRequested() == true){
        //         m_PeriodicIO.requestedState = State.SHOOTING;
        //     }
        //     else{
        //         m_PeriodicIO.requestedState = State.STROBE;
        //     }
        // } else {
        //     m_PeriodicIO.requestedState = State.DEFAULT;
        // }

        // //when click the shoot, solid green
        // //when we are not seeing limelight change quarter of the led

        // if (m_PeriodicIO.lastRequState != m_PeriodicIO.requestedState) {

        //     switch (m_PeriodicIO.requestedState) {
        //         case DEFAULT:    
        //             m_candle.clearAnimation(0);
        //             DefaultColor();
        //             // m_candle.animate(noah_Animation);
        //             break;
        //         case STROBE:
        //             // m_candle.clearAnimation(0);
        //             m_candle.animate(m_StrobeAnimation, 0);
        //             break;
        //         case SHOOTING:
        //             if(RobotConstants.kPracticeBot) {
        //                 if(ShooterConst.Shooter.isAtRotation()){
                            
        //                     m_candle.setLEDs(0, 255, 0, 0, 0, 1);
        //                 }
        //                 else{
        //                     m_candle.setLEDs(255, 0, 0, 0, 0, 1);
        //                 }

        //                 if(ShooterConst.Shooter.isPivotAt()){
        //                     m_candle.setLEDs(0, 255, 0, 0, 1, 1);
        //                 }
        //                 else{
        //                     m_candle.setLEDs(255, 255, 0, 0, 1, 1);
        //                 }
                        
        //                 if(ShooterConst.Shooter.isAtVelocity()){
        //                     m_candle.setLEDs(0, 255, 0, 0, 2, 1);
        //                 }
        //                 else {
        //                     m_candle.setLEDs(0, 0, 255, 0, 2, 1);
        //                 }
                        
        //                 m_candle.setLEDs(0, 255, 0, 0, 3, 3);
        //             }

        //             if(ShooterConst.Shooter.isAtRotation()){    
        //                 m_candle.setLEDs(0, 255, 0, 0, 1, 15);
        //             }
        //             else{
        //                 m_candle.setLEDs(255, 0, 0, 0, 1, 15);
        //             }

        //             if(ShooterConst.Shooter.isPivotAt()){
        //                 m_candle.setLEDs(0, 255, 0, 0, 15, 15);
        //             }
        //             else{
        //                 m_candle.setLEDs(255, 255, 0, 0, 15, 15);
        //             }
                            
        //             if(ShooterConst.Shooter.isAtVelocity()){
        //                 m_candle.setLEDs(0, 255, 0, 0, 30, 15);
        //             }
        //             else {
        //                 m_candle.setLEDs(0, 0, 255, 0, 30, 15);
        //             }
                            
        //             m_candle.setLEDs(0, 255, 0, 0, 45, 60);
        //             break;
        //     }
        //     m_PeriodicIO.lastRequState = m_PeriodicIO.requestedState;
        // }
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method
        // 'checkSystem'");
        return true;
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method
        // 'outputTelemetry'");

        SignalLogger.writeString("Candle/State", m_PeriodicIO.requestedState.name());
    }
}
