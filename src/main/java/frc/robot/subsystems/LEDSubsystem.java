// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  AddressableLED m_led = new AddressableLED(9); //45
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(17);
  int m_rainbowFirstPixelHue = 0; //this is the starting point for the rainbow
  public LEDSubsystem() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setOneColor()
  { //test to make sure nothing has gone catastrophically wrong, also known as setting all of the LEDs to red.
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 255, 0, 0);
   }
  }
  
  public void colorFlash() //flashes a color three times. Later perhaps I will make the color passed in through the function.
  {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      //flashes a color
      m_ledBuffer.setRGB(i, 255, 0, 255);
   }
   for (int i = 0; i < m_ledBuffer.getLength(); i++) {
    // turns off
    m_ledBuffer.setRGB(i, 0, 0, 0);
 }
 for (int i = 0; i < m_ledBuffer.getLength(); i++) {
  //flashes a color
  m_ledBuffer.setRGB(i, 255, 0, 255);
}
for (int i = 0; i < m_ledBuffer.getLength(); i++) {
// turns off
m_ledBuffer.setRGB(i, 0, 0, 0);
}
for (int i = 0; i < m_ledBuffer.getLength(); i++) {
  //flashes a color
  m_ledBuffer.setRGB(i, 255, 0, 255);
}
for (int i = 0; i < m_ledBuffer.getLength(); i++) {
// turns off
m_ledBuffer.setRGB(i, 0, 0, 0);
}
  }
  void unmovingRainbow() //makes a rainbow that does not move - mostly for testing purposes - will remove later.
  {
      // For every pixel
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final int hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
        // Set the value
        m_ledBuffer.setHSV(i, hue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      m_rainbowFirstPixelHue += 3;
      // Check bounds
      m_rainbowFirstPixelHue %= 180;
      m_led.setData(m_ledBuffer);
  }
  void movingRainbow(int movements) //does that thing with the unmoving rainbow but moves it. 
  {
    for(int j = 0; j<movements; j++)
    {
      // For every pixel
      for (int i = 0; i < m_ledBuffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final int hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
        // Set the value
        m_ledBuffer.setHSV(i, hue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      m_rainbowFirstPixelHue += 3;
      // Check bounds
      m_rainbowFirstPixelHue %= 180;
      m_led.setData(m_ledBuffer);
    }
   
   
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
