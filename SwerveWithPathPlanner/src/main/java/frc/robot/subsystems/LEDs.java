
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;

public class LEDs extends SubsystemBase {
  private final HttpClient client;

  public LEDs() {
    // Build the HTTP client once when the robot boots. 
    // We enforce a strict 500ms timeout so it can NEVER freeze the robot.
    client = HttpClient.newBuilder()
        .connectTimeout(Duration.ofMillis(500))
        .build();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Helper method to trigger the URL completely in the background
  private void sendRequest(String urlString) {
    try {
      HttpRequest request = HttpRequest.newBuilder()
          .uri(URI.create(urlString))
          .timeout(Duration.ofMillis(500)) // Abort if ESP32 takes longer than 500ms
          .GET()
          .build();

      // sendAsync triggers the request in the background and instantly returns.
      // BodyHandlers.discarding() tells it we don't care about reading the response,
      // completely eliminating the readLine() freeze!
      client.sendAsync(request, HttpResponse.BodyHandlers.discarding());
      
    } catch (Exception e) {
      // Ignore errors (like malformed URLs) to keep the robot running safely
    }
  }

  public void Default() {
    var alliance = DriverStation.getAlliance();
    
    if (alliance.isPresent() && alliance.get() == Alliance.Blue) {
      sendRequest("http://10.22.83.100/win&PL=1");
    } else if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      sendRequest("http://10.22.83.100/win&PL=2");
    }
  }

  public void Feed() {
    sendRequest("http://10.22.83.100/win&PL=3");
  }

  public void Idle() {
    sendRequest("http://10.22.83.100/win&PL=4");
  }

  public void RTF() {
    sendRequest("http://10.22.83.100/win&PL=5");
  }
}