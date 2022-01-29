package frc.robot;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotType {

  public enum Type {
    COMPETITION,
    SKILLSBOT,
    PRACTICE
  }

  private static String kPracticeBotMAC = "00:80:2F:17:89:85";

  //TODO:  Need accurate MAC address from skillsbot
  private static String kSkillsBotMAC = "00:80:2F:00:00:00";

  public static Type getRobotType() {
    String mac = getMACAddress();
    if (mac != null && mac.equals(kPracticeBotMAC)) {
      return Type.PRACTICE;
    } else if (mac != null && mac.equals(kSkillsBotMAC)) {
      return Type.SKILLSBOT;
    }
    return Type.COMPETITION;
  }

  private static String getMACAddress() {
    try {
      NetworkInterface network = NetworkInterface.getByInetAddress(InetAddress.getLocalHost());
      byte[] address = network.getHardwareAddress();
      StringBuilder sb = new StringBuilder();
      for (int i = 0; i < address.length; i++) {
        sb.append(String.format("%02X%s", address[i], (i < address.length - 1) ? ":" : ""));
      }

      // Remove once validated.
      SmartDashboard.putString("Mac-Address", sb.toString());
      return sb.toString();
    } catch (UnknownHostException | SocketException e) {
      e.printStackTrace();
    }
    return null;
  }
}
