package frc.robot.subsystems;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotConstants {
    
    public static final String kPracticeBotMACAddress = "00:80:2F:33:D0:6E";
    public static final boolean kPracticeBot = getMACAddress().equals(kPracticeBotMACAddress);
    public static final double kSecondsPerPeriodic = .02;
    public static final String kCanivoreBusName = "canivore";
    
    public static String getMACAddress() {
    
            try {
               NetworkInterface myNI = NetworkInterface.networkInterfaces().filter(it -> {
                 try {
                   byte[] MA = it.getHardwareAddress();
                   return null != MA;
                 } catch (Exception e) {
                   e.printStackTrace();
                 }
                 return false;
              }).findFirst().orElse(NetworkInterface.networkInterfaces().findFirst().get());
              byte[] MAC_ADDRESS = myNI.getHardwareAddress();
              final List<Byte> macList = new ArrayList<>();
              if (null != MAC_ADDRESS) {
                for (byte b : MAC_ADDRESS) {
                  macList.add(b);
                }
              }
              String macString = macList.stream().map(it -> String.format("%02X", it))
                  .collect(Collectors.joining(":"));
              SmartDashboard.putString("MAC", macString);
              return macString;
            } catch (SocketException e) {
              e.printStackTrace();
            }
            SmartDashboard.putString("MAC", "");
            return "";
          
        // try {
        //     Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
        //     StringBuilder ret = new StringBuilder();
        //     while (nwInterface.hasMoreElements()) {
        //         NetworkInterface nis = nwInterface.nextElement();
        //         System.out.println("NIS: " + nis.getDisplayName());
        //         if (nis != null && "etho".equals(nis.getDisplayName())) {
        //             byte[] mac = nis.getHardwareAddress();
        //             if (mac != null){
        //                 for (int i = 0; i <mac.length; i++) {
        //                     ret.append(String.format("%02X%s", mac[i], (i < mac.length -1) ? ":" : ""));
        //                 }
        //                 String addr = ret.toString();
        //                 System.out.println("NIS " + nis.getDisplayName() + " addr: " + addr);

        //                 SmartDashboard.putString("MAC", addr);
        //                 return addr;
        //             } else {
        //                 System.out.println("Address doesn't exist or is not accessible");
        //             }
        //         } else {
        //             System.out.println("Skipping adaptor: " + nis.getDisplayName());
        //         }
        //     }
        // } catch (SocketException | NullPointerException e) {
        //     e.printStackTrace();
        // }

        // SmartDashboard.putString("MAC", "");                
        // return "";
    }
}
