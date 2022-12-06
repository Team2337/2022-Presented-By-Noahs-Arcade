package frc.robot.subsystems.hardware;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;

import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import org.apache.commons.io.IOIndexedException;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase  {
  
  private PhotonCamera aprilTagCamera = new PhotonCamera("aprilTagCamera");
  private PhotonPipelineResult targets;
  private PhotonTrackedTarget target;
  private boolean hasTarget = false;
  private ArrayList<AprilTag> aprilTags;

  public PhotonVision() {
  }

  public static class AprilTag {
    public int id;
    public double yaw;
    public double area;

    public AprilTag(int id, double yaw, double area) {
      this.id = id;
      this.yaw = yaw;
      this.area = area;
    }
  }



  @Override
  public void periodic() {
    targets = aprilTagCamera.getLatestResult();
    if (targets.hasTargets()) {
      aprilTags.clear();
      for (int i = 0; i < targets.getTargets().size(); i++){
        target = targets.getTargets().get(i);
        int id = target.getFiducialId();
        double yaw = target.getYaw();
        double area = target.getArea();
        AprilTag tag = new AprilTag(id, yaw, area);
        aprilTags.add(tag);
      }
    }
  }

  public double[] readAprilTagDataFromDatabase(int id){
    /*So Jackson is a way to read JSON files in Java, and it is already in WPILib, and we will probably need to use
    a database to store AprilTag position data, so why not use it, as JSON is easy to use? *
    Exact techinique taken from https://stackoverflow.com/a/21760537 */

    ObjectMapper mapper = new ObjectMapper();
    InputStream in = null;
    try {
      //Java works on FileInputStreams, so we need to convert the file into an input stream
      in = new FileInputStream(new File("src//main//java//frc//robot//subsystems//hardware//apriltags.json"));
    } catch (FileNotFoundException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    //BufferedReaders are more efficient ways to read through data, so that is what now can read the JSON File
    //(Copied from my homework)
    BufferedReader br = new BufferedReader(new InputStreamReader(in));
    Map<String, Map<String, Double>> map = null;
    try {
      //Java has a way to create {key,value} dictionaries with maps, so this is converting the JSON into this dictionary object
      map = mapper.readValue(br, Map.class);
    } catch (JsonParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (JsonMappingException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    //And this returns all the data in a nice neat array.
    double[] output = {map.get(String.valueOf(id)).get("x"),map.get(String.valueOf(id)).get("y"),map.get(String.valueOf(id)).get("height")};
    return output;
  }
  
  public ArrayList<AprilTag> getAprilTags(){
    return aprilTags;
  }

  public boolean hasTarget() {
    return hasTarget;
  }

  public void changePipeline(int pipeline) {
    aprilTagCamera.setPipelineIndex(pipeline);
  }

  public double getFrameWidth() {
    //Find based off of PhotonVision output settings
    return 320;
  }

  public double getFrameCenter() {
    return getFrameWidth() / 2;
  }

}
