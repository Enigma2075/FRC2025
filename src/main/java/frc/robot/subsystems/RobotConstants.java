package frc.robot.subsystems;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

public class RobotConstants {
    
    public static final String kPracticeBotMACAddress = "";
    public static final boolean kPracticeBot = getMACAddress().equals(kPracticeBotMACAddress);
    public static final double kSecondsPerPeriodic = .02;
    public static final String kCanivoreBusName = "canivore";
    
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                System.out.println("NIS: " + nis.getDisplayName());
                if (nis != null && "etho".equals(nis.getDisplayName())) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null){
                        for (int i = 0; i <mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length -1) ? ":" : ""));
                        }
                        String addr = ret.toString();
                        System.out.println("NIS " + nis.getDisplayName() + " addr: " + addr);
                        return addr;
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Skipping adaptor: " + nis.getDisplayName());
                }
            }
        } catch (SocketException | NullPointerException e) {
            e.printStackTrace();
        }
        return "";
    }
}
