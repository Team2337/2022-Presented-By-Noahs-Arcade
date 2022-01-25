package frc.robot;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;

public class RobotType {

  public enum Type {
    COMPETITION,
    PRACTICE
  }

  private static String kPracticeBotMAC = "";

  public static Type getRobotType() {
    String mac = getMACAddress();
    if (mac != null && mac.equals(kPracticeBotMAC)) {
      return Type.PRACTICE;
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
      return sb.toString();
    } catch (UnknownHostException | SocketException e) {
      e.printStackTrace();
    }
    return null;
  }
}
