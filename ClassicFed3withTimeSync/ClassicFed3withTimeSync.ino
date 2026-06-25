/*
  Feeding experimentation device 3 (FED3)
  Classic FED3 script with Time Sync (FNT compatible)
  
  This script merges the original FED3 Classic script with time synchronization
  and dynamic control capabilities for the Feeding Network Tracker (FNT).
  
  Original Author: 
  alexxai@wustl.edu
  December, 2020

  FNT Modifications:
  samuel.garmany@colorado.edu
  June, 2026
*/

#include <FED3.h>                //Include the FED3 library 
#include <RTClib.h>              //Added for time sync
String sketch = "Classic";       //Unique identifier text for each sketch
FED3 fed3 (sketch);              //Start the FED3 object
extern RTC_PCF8523 rtc;          //Connect to the clock

//variables for PR tasks
int poke_num = 0;                                      // this variable is the number of pokes since last pellet
int pokes_required = 1;                                // increase the number of pokes required each time a pellet is received using an exponential equation

//variables for Timeout task (FNT mode 12)
unsigned long lastPelletTime = 0;
unsigned long timeoutDuration = 0;                     // in milliseconds

// variables for tracking
int lastLeftCount = 0;
int lastRightCount = 0;
int lastPelletCount = 0;

// Helper to draw the animated mouse at a specific X position
void drawMouse(int x) {
  fed3.display.fillRoundRect(x + 25, 82, 15, 10, 6, BLACK);    //head
  fed3.display.fillRoundRect(x + 22, 80, 8, 5, 3, BLACK);      //ear
  fed3.display.fillRoundRect(x + 30, 84, 1, 1, 1, WHITE);      //eye
  
  if ((x / 10) % 2 == 0) {
    fed3.display.fillRoundRect(x, 84, 32, 17, 10, BLACK);      //body
    fed3.display.drawFastHLine(x - 8, 85, 18, BLACK);           //tail
    fed3.display.drawFastHLine(x - 8, 86, 18, BLACK);
    fed3.display.drawFastHLine(x - 14, 84, 8, BLACK);
    fed3.display.drawFastHLine(x - 14, 85, 8, BLACK);
    fed3.display.fillRoundRect(x + 22, 99, 8, 4, 3, BLACK);    //front foot
    fed3.display.fillRoundRect(x, 97, 8, 6, 3, BLACK);          //back foot
  } else {
    fed3.display.fillRoundRect(x + 2, 82, 30, 17, 10, BLACK);  //body
    fed3.display.drawFastHLine(x - 6, 91, 18, BLACK);           //tail
    fed3.display.drawFastHLine(x - 6, 90, 18, BLACK);
    fed3.display.drawFastHLine(x - 12, 92, 8, BLACK);
    fed3.display.drawFastHLine(x - 12, 91, 8, BLACK);
    fed3.display.fillRoundRect(x + 15, 99, 8, 4, 3, BLACK);    //foot
    fed3.display.fillRoundRect(x + 8, 97, 8, 6, 3, BLACK);     //back foot
  }
}

void setup() {
  Serial.begin(115200);                                // start listening on the serial port
  Serial.setTimeout(10);                               // Set serial timeout to 10ms to make reads non-blocking
  randomSeed(analogRead(0));                           // Seed the random number generator
  fed3.ClassicFED3 = false;                            // Bypasses the boot menu
  fed3.begin();                                        //Setup the FED3 hardware
  if (fed3.FED < 1) {
    fed3.FED = 1;
  }
  fed3.disableSleep();                                 //disable sleep for time sync purposes, this will hurt battery but sync requires a wired connection anyways
  fed3.FEDmode = 1;                                    //Default to Fixed Ratio (FR)
  fed3.FR = 1;                                         //Default to FR1
  fed3.sessiontype = "FR1";
  fed3.DisplayPokes = true;
  fed3.DisplayTimed = false;
 
  // Custom Device ID configuration menu on boot (modeled after the original FED3 Classic menu)
  int initialFED = fed3.FED;
  bool displayNeedsUpdate = true;
  int mouseX = -50;
  
  // Set up screen elements before animation loop
  fed3.display.clearDisplay();
  fed3.display.setTextColor(BLACK);
  fed3.display.setTextSize(1);
  
  // Top header text
  fed3.display.setCursor(10, 40);
  fed3.display.print("Set Device ID:");
  
  // Filename at the bottom
  fed3.display.setCursor(1, 135);
  fed3.display.print(fed3.filename);
  
  // Only display the interactive boot menu if USB Serial is NOT active.
  // This bypasses boot delay when FNT scanner pings the board, preventing SAMD21 USB controller hangs.
  if (!Serial) {
    // The mouse animation acts as a loading bar timeout. Resetting mouseX resets the timer.
    while (mouseX < 200) {
      if (displayNeedsUpdate) {
        // Erase previous ID text area
        fed3.display.fillRect(10, 48, 140, 20, WHITE);
        
        // Draw selected ID text
        fed3.display.setCursor(10, 60);
        if (fed3.FED < 100 && fed3.FED >= 10) {
          fed3.display.print("0");
        }
        if (fed3.FED < 10) {
          fed3.display.print("00");
        }
        fed3.display.print(fed3.FED);
        
        fed3.display.refresh();
        displayNeedsUpdate = false;
      }
      
      // Draw the running mouse animation frame by frame
      drawMouse(mouseX);
      
      fed3.display.refresh();
      delay(80);
      
      // Erase the mouse area
      fed3.display.fillRect (mouseX - 25, 73, 95, 33, WHITE);
      
      // Update mouse position (loading bar progress, original mouse speed)
      mouseX += 15;
      
      // Poll the buttons directly (digitalRead == LOW is pressed)
      // Left Poke -> Decrement Device ID
      if (digitalRead(LEFT_POKE) == LOW) {
        if (fed3.FED > 1) {
          fed3.FED--;
          // Play tones & neopixel animations like in SelectMode()
          tone(BUZZER, 2500, 200);
          fed3.colorWipe(fed3.strip.Color(2, 0, 2), 40); // Purple wipe
          fed3.colorWipe(fed3.strip.Color(0, 0, 0), 20); // OFF
          mouseX = -50; // Reset loading bar/timeout
          displayNeedsUpdate = true;
        }
      }
      
      // Right Poke -> Increment Device ID
      if (digitalRead(RIGHT_POKE) == LOW) {
        if (fed3.FED < 999) {
          fed3.FED++;
          // Play tones & neopixel animations like in SelectMode()
          tone(BUZZER, 2500, 200);
          fed3.colorWipe(fed3.strip.Color(2, 2, 0), 40); // Yellow/Green wipe
          fed3.colorWipe(fed3.strip.Color(0, 0, 0), 20); // OFF
          mouseX = -50; // Reset loading bar/timeout
          displayNeedsUpdate = true;
        }
      }
    }
  }
  
  // If the device ID was changed, recreate the datafile with the new ID
  if (fed3.FED != initialFED) {
    // Print "...Selected!" like in the original program selection menu
    fed3.display.setCursor(10, 100);
    fed3.display.println("...Selected!");
    fed3.display.refresh();
    delay(500);

    fed3.SD.remove(fed3.filename);                     // Delete the initial empty file created with old ID
    fed3.writeConfigFile();                            // Write the new config
    fed3.CreateDataFile();                             // Create the new datafile using new ID
    fed3.writeHeader();                                // Write header to the new file
  }
  
  // Clear display, clear nosepoke flags, and show the normal FR1 screen
  fed3.Left = false;
  fed3.Right = false;
  fed3.display.clearDisplay();
  fed3.UpdateDisplay();
}

void loop() {
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //                                                                     Original FED3 Classic Modes
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Mode 0: Free feeding
  if (fed3.FEDmode == 0) {
    fed3.sessiontype = "Free_feed";                     //The text in "sessiontype" will appear on the screen and in the logfile
    fed3.DisplayPokes = false;                          //Turn off poke indicators for free feeding mode
    fed3.UpdateDisplay();                               //Update display for free feeding session to remove poke display (they are on by default)
    fed3.Feed();
    fed3.Timeout(5);                                    //5s timeout
  }

  // Modes 1-3: Fixed Ratio Programs FR1, FR3, FR5
  if ((fed3.FEDmode == 1) or (fed3.FEDmode == 2) or (fed3.FEDmode == 3)) {
    fed3.sessiontype = "FR" + String(fed3.FR);          //The text in "sessiontype" will appear on the screen and in the logfile
    if (fed3.Left) {
      fed3.logLeftPoke();                               //Log left poke
      if (fed3.LeftCount % fed3.FR == 0) {              //if fixed ratio is  met
        fed3.ConditionedStimulus();                     //deliver conditioned stimulus (tone and lights)
        fed3.Feed();                                    //deliver pellet
      }
    }
    if (fed3.Right) {                                    //If right poke is triggered
      fed3.logRightPoke();
    }
  }

  // Mode 4: Progressive Ratio
  if (fed3.FEDmode == 4) {
    fed3.sessiontype = "ProgRatio";                      //The text in "sessiontype" will appear on the screen and in the logfile
    if (fed3.Left) {                                     //If left poke is triggered and pellet is not in the well
      fed3.logLeftPoke();                                //Log left poke
      fed3.Click();                                      //Click
      poke_num++;                                        //store this new poke number as current poke number.
      if (poke_num == pokes_required) {                  //check to see if the mouse has acheived the correct number of pokes in order to receive the pellet
        fed3.ConditionedStimulus();                      //Deliver conditioned stimulus (tone and lights)
        fed3.Feed();                                     //Deliver pellet
        pokes_required = round((5 * exp((fed3.PelletCount + 1) * 0.2)) - 5);  //increment the number of pokes required according to the progressive ratio:
        fed3.FR = pokes_required;
        poke_num = 0;                                    //reset poke_num back to 0 for the next trial
      }
    }
    if (fed3.Right) {                                    //If right poke is triggered and pellet is not in the well
      fed3.logRightPoke();
    }
  }

  // Mode 5: Extinction
  if (fed3.FEDmode == 5) {
    fed3.sessiontype = "Extinct";                        //The text in "sessiontype" will appear on the screen and in the logfile
    if (fed3.Left) {
      fed3.logLeftPoke();                                //Log left poke
      fed3.ConditionedStimulus();                        //deliver conditioned stimulus (tone and lights)
    }
    if (fed3.Right) {                                    //If right poke is triggered
      fed3.logRightPoke();
    }
  }

  // Mode 6: Light tracking FR1 task
  if (fed3.FEDmode == 6) {
    fed3.sessiontype = "Light Trk";                       //The text in "sessiontype" will appear on the screen and in the logfile
    fed3.disableSleep();                                  //Sleep mode shuts the NeoPixels off to save power.  Therefore to leave pixels on during this task we must disable sleep mode.

    //If left poke is active, run FR1 session with left active
    if (fed3.activePoke == 1) {
      fed3.leftPokePixel(5,5,5,0) ;                       //turn on pixel inside left nosepoke dim white
      if (fed3.Left) {
        fed3.logLeftPoke();                               //Log left poke
        fed3.ConditionedStimulus();                       //deliver conditioned stimulus (tone and lights)
        fed3.Feed();
        fed3.randomizeActivePoke(3);                      //randomize which poke is active, specifying maximum on the same poke before forcing a switch
      }
      if (fed3.Right) {                                   //If right poke is triggered
        fed3.logRightPoke();
      }
    }
    //If right poke is active, run FR1 session with right active
    if (fed3.activePoke == 0) {
      fed3.rightPokePixel(5,5,5,0) ;                    //turn on pixel inside right nosepoke dim white
      if (fed3.Right) {
        fed3.logRightPoke();                              //Log left poke
        fed3.ConditionedStimulus();                       //deliver conditioned stimulus (tone and lights)
        fed3.Feed();                                      //deliver pellet
        fed3.randomizeActivePoke(3);                      //randomize which poke is active, specifying maximum on the same poke before forcing a switch
      }
      if (fed3.Left) {                                    //If right poke is triggered
        fed3.logLeftPoke();
      }
    }
  }

  // Mode 7: FR (reversed)
  if (fed3.FEDmode == 7) {
    if (fed3.FR == 1) {
      fed3.sessiontype = "FR1_R";
    } else {
      fed3.sessiontype = "FR" + String(fed3.FR) + "_R";
    }
    fed3.activePoke = 0;                                  //Set activePoke to 0 to make right poke active
    if (fed3.Left) {                                      //If left poke
      fed3.logLeftPoke();                                 //Log left poke
    }
    if (fed3.Right) {                                     //If right poke is triggered
      fed3.logRightPoke();                                //Log Right Poke
      if (fed3.RightCount % fed3.FR == 0) {               //if fixed ratio is met
        fed3.ConditionedStimulus();                         //Deliver conditioned stimulus (tone and lights)
        fed3.Feed();                                        //deliver pellet
      }
    }
  }

  // Mode 8: PR (reversed)
  if (fed3.FEDmode == 8) {
    fed3.sessiontype = "PR_R";                          //The text in "sessiontype" will appear on the screen and in the logfile
    fed3.activePoke = 0;                                //Right poke is active
    if (fed3.Right) {                                   //If Right poke is triggered
      fed3.logRightPoke();                              //Log Right poke
      poke_num++;                                       //store this new poke number as current poke number.
      if (poke_num == pokes_required) {                 //check to see if the mouse has acheived the correct number of pokes in order to receive the pellet
        fed3.ConditionedStimulus();                     //Deliver conditioned stimulus (tone and lights)
        fed3.Feed();                                    //Deliver pellet
        pokes_required = round((5 * exp((fed3.PelletCount + 1) * 0.2)) - 5);
        fed3.FR = pokes_required;
        poke_num = 0;                                   //reset the number of pokes back to 0, for the next trial
        fed3.Right = false;
      }
      else {
        fed3.Click();                                   //If not enough pokes, just do a Click
      }
    }
    if (fed3.Left) {                                    //If left poke is triggered and pellet is not in the well
      fed3.logLeftPoke();
    }
  }

  // Mode 9: Optogenetic stimulation
  if (fed3.FEDmode == 9) {
    fed3.sessiontype = "OptoStim";                      //The text in "sessiontype" will appear on the screen and in the logfile
    if (fed3.Left) {                                    //If left poke
      fed3.logLeftPoke();                               //Log left poke
      fed3.ConditionedStimulus();                       //Deliver conditioned stimulus (tone and lights)
      fed3.BNC(25, 20);                                 //Deliver 20 pulses at 20Hz (25ms HIGH, 25ms LOW), lasting 1 second
    }
    if (fed3.Right) {                                   //If right poke is triggered
      fed3.logRightPoke();                              //Log Right Poke
    }
  }

  // Mode 10: Optogenetic stimulation (reversed)
  if (fed3.FEDmode == 10) {
    fed3.sessiontype = "OptoStim_R";                     //The text in "sessiontype" will appear on the screen and in the logfile
    fed3.activePoke = 0;                                 //Set activePoke to 0 to make right poke active
    if (fed3.Right) {                                    //If Right poke
      fed3.logRightPoke();                               //Log Right poke
      fed3.ConditionedStimulus();                        //Deliver conditioned stimulus (tone and lights)
      fed3.BNC(25, 20);                                  //Deliver 20 pulses at 20Hz (25ms HIGH, 25ms LOW), lasting 1 second
    }
    if (fed3.Left) {                                     //If Left poke is triggered
      fed3.logLeftPoke();                                //Log LeftPoke
    }
  }

  // Mode 11: Timed Feeding
  if (fed3.FEDmode == 11) {
    fed3.sessiontype = "Timed";                         //The text in "sessiontype" will appear on the screen and in the logfile
    fed3.DisplayPokes = false;                          //Turn off poke indicators for free feeding mode
    fed3.DisplayTimed = true;                           //Display timed feeding info
    fed3.UpdateDisplay();
    if (fed3.currentHour >= fed3.timedStart && fed3.currentHour < fed3.timedEnd) {
      fed3.Feed();
      fed3.Timeout(5);                                  //5s timeout
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //                                                                     FNT Custom Modes
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Mode 12: Fixed Ratio with Timeout (FRTO)
  if (fed3.FEDmode == 12) {
    if (fed3.Left) {
      fed3.logLeftPoke();                               //Log left poke
      if (lastPelletTime == 0 || (millis() - lastPelletTime >= timeoutDuration)) {
        if (fed3.LeftCount % fed3.FR == 0) {
          fed3.ConditionedStimulus();
          fed3.Feed();
          lastPelletTime = millis();
        }
      } else {
        fed3.Click();                                   // Play sound click to indicate registered poke during timeout
      }
    }
    if (fed3.Right) {
      fed3.logRightPoke();
    }
  }

  // Mode 13: Random Ratio (RR)
  if (fed3.FEDmode == 13) {
    if (fed3.Left) {
      fed3.logLeftPoke();                               //Log left poke
      if (random(1, fed3.FR + 1) == 1) {
        fed3.ConditionedStimulus();
        fed3.Feed();
      } else {
        fed3.Click();
      }
    }
    if (fed3.Right) {
      fed3.logRightPoke();
    }
  }

  // Call fed.run at least once per loop
  fed3.run();

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //                                                                     FNT Serial Comm and Live Tracking
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Live Tracking System over Serial
  if (fed3.LeftCount > lastLeftCount) {
    if (Serial) {
      Serial.print("Left Poke, Total: ");
      Serial.println(fed3.LeftCount);
    }
    lastLeftCount = fed3.LeftCount;
  }
  
  if (fed3.RightCount > lastRightCount) {
    if (Serial) {
      Serial.print("Right Poke, Total: ");
      Serial.println(fed3.RightCount);
    }
    lastRightCount = fed3.RightCount;
  }
  
  if (fed3.PelletCount > lastPelletCount) {
    if (Serial) {
      Serial.print("Pellet Dispensed, Total: ");
      Serial.println(fed3.PelletCount);
    }
    lastPelletCount = fed3.PelletCount;
  }

  // Time Syncronization and Mode Update System
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); // remove any carriage returns
    
    // Handshake for GUI auto-discovery
    if (command == "PING") {
        Serial.print("PONG_FED3,ID:");
        Serial.println(fed3.FED);
    }
    else if (command == "LIST_FILES") {
        File root = fed3.SD.open("/");
        if (!root) {
            Serial.println("ERROR:CANNOT_OPEN_ROOT");
        } else {
            root.rewind();
            while (true) {
                File entry = root.openNextFile();
                if (!entry) {
                    break;
                }
                char name[32];
                entry.getName(name, sizeof(name));
                String nameStr = String(name);
                if (!entry.isDirectory() && nameStr.startsWith("FED") && nameStr.endsWith(".CSV")) {
                    Serial.print("FILE:");
                    Serial.print(nameStr);
                    Serial.print(",");
                    Serial.println(entry.fileSize());
                }
                entry.close();
            }
            root.close();
            Serial.println("END_LIST");
        }
    }
    else if (command.startsWith("GET_FILE:")) {
        String reqFilename = command.substring(9);
        reqFilename.trim();
        
        // Open the file for reading
        File file = fed3.SD.open(reqFilename, FILE_READ);
        if (!file) {
            Serial.print("ERROR:FILE_NOT_FOUND:");
            Serial.println(reqFilename);
        } else {
            Serial.print("FILE_DATA_START:");
            Serial.println(reqFilename);
            
            uint32_t crc = 0xFFFFFFFF;
            while (file.available()) {
                char c = file.read();
                Serial.write(c);
                
                // Update CRC-32
                crc ^= (uint8_t)c;
                for (int j = 0; j < 8; j++) {
                    if (crc & 1) {
                        crc = (crc >> 1) ^ 0xEDB88320;
                    } else {
                        crc >>= 1;
                    }
                }
            }
            file.close();
            crc = ~crc;
            
            // Send EOT and CRC
            Serial.write(0x04);
            Serial.println();
            Serial.print("CRC32:");
            Serial.println(crc, HEX);
        }
    }
    else if (command == "NEW_TRIAL") {
        fed3.LeftCount = 0;
        fed3.RightCount = 0;
        fed3.PelletCount = 0;
        fed3.BlockPelletCount = 0;
        
        lastLeftCount = 0;
        lastRightCount = 0;
        lastPelletCount = 0;
        
        // Reset PR variables
        poke_num = 0;
        pokes_required = 1;
        fed3.FR = 1;
        
        // Create new data file on SD card
        fed3.CreateDataFile();
        fed3.writeHeader();
        
        // Update display to reflect new counters
        fed3.UpdateDisplay();
        
        Serial.print("NEW_TRIAL_STARTED:");
        Serial.println(fed3.filename);
    }
    else if (command == "FEED") {
        fed3.Feed();
        Serial.println("Pellet dispensed manually.");
    }
    else if (command == "LIGHTS:ON") {
        fed3.pixelsOn(30, 30, 30, 30);
        Serial.println("Lights turned ON.");
    }
    else if (command == "LIGHTS:OFF") {
        fed3.pixelsOff();
        Serial.println("Lights turned OFF.");
    }
    // check if the command starts with SYNC:
    else if (command.startsWith("SYNC:")) {
        // parse the incoming string: SYNC:YYYY,MM,DD,HH,MM,SS
        int y = command.substring(5, 9).toInt();
        int m = command.substring(10, 12).toInt();
        int d = command.substring(13, 15).toInt();
        int h = command.substring(16, 18).toInt();
        int min = command.substring(19, 21).toInt();
        int s = command.substring(22, 24).toInt();
        
        // adjust the RTC
        DateTime newTime = DateTime(y, m, d, h, min, s);
        rtc.adjust(newTime);
        Serial.println("Time synced successfully.");
        fed3.UpdateDisplay();
    }
    // check if the command starts with MODE:
    else if (command.startsWith("MODE:")) {
        String mode_params = command.substring(5);
        if (mode_params.startsWith("FRTO")) {
            int comma1 = mode_params.indexOf(',');
            int ratio = 1;
            int timeout_s = 30;
            if (comma1 != -1) {
                int comma2 = mode_params.indexOf(',', comma1 + 1);
                if (comma2 != -1) {
                    ratio = mode_params.substring(comma1 + 1, comma2).toInt();
                    timeout_s = mode_params.substring(comma2 + 1).toInt();
                } else {
                    ratio = mode_params.substring(comma1 + 1).toInt();
                }
            }
            if (ratio < 1) ratio = 1;
            if (timeout_s < 0) timeout_s = 0;
            
            fed3.FEDmode = 12; 
            fed3.FR = ratio;
            timeoutDuration = (unsigned long)timeout_s * 1000;
            lastPelletTime = 0; 
            
            fed3.sessiontype = "FR" + String(ratio) + " TO" + String(timeout_s) + "s";
            fed3.DisplayPokes = true;
            fed3.DisplayTimed = false;
            fed3.UpdateDisplay();
            Serial.print("Mode set to Fixed Ratio with Timeout. FR: ");
            Serial.print(fed3.FR);
            Serial.print(", Timeout: ");
            Serial.print(timeout_s);
            Serial.println("s");
        }
        else if (mode_params == "FR1") {
            fed3.FEDmode = 1;
            fed3.FR = 1;
            fed3.sessiontype = "FR1";
            fed3.DisplayPokes = true;
            fed3.DisplayTimed = false;
            fed3.UpdateDisplay();
            Serial.println("Mode set to FR1");
        }
        else if (mode_params == "FR3") {
            fed3.FEDmode = 2;
            fed3.FR = 3;
            fed3.sessiontype = "FR3";
            fed3.DisplayPokes = true;
            fed3.DisplayTimed = false;
            fed3.UpdateDisplay();
            Serial.println("Mode set to FR3");
        }
        else if (mode_params == "FR5") {
            fed3.FEDmode = 3;
            fed3.FR = 5;
            fed3.sessiontype = "FR5";
            fed3.DisplayPokes = true;
            fed3.DisplayTimed = false;
            fed3.UpdateDisplay();
            Serial.println("Mode set to FR5");
        }
        else if (mode_params == "FR" || mode_params.startsWith("FR,")) {
            int comma_idx = mode_params.indexOf(',');
            int ratio = 1;
            if (comma_idx != -1) {
                ratio = mode_params.substring(comma_idx + 1).toInt();
            }
            if (ratio < 1) ratio = 1;
            fed3.FEDmode = 1; 
            fed3.FR = ratio;
            fed3.sessiontype = "FR" + String(ratio);
            fed3.DisplayPokes = true;
            fed3.DisplayTimed = false;
            fed3.UpdateDisplay();
            Serial.print("Mode set to Fixed Ratio, FR: ");
            Serial.println(fed3.FR);
        }
        else if (mode_params.startsWith("RR")) {
            int comma_idx = mode_params.indexOf(',');
            int ratio = 5; 
            if (comma_idx != -1) {
                ratio = mode_params.substring(comma_idx + 1).toInt();
            }
            if (ratio < 1) ratio = 1;
            fed3.FEDmode = 13; 
            fed3.FR = ratio;
            fed3.sessiontype = "RR" + String(ratio);
            fed3.DisplayPokes = true;
            fed3.DisplayTimed = false;
            fed3.UpdateDisplay();
            Serial.print("Mode set to Random Ratio, RR: ");
            Serial.println(fed3.FR);
        }
        else if (mode_params == "PR") {
            fed3.FEDmode = 4;
            fed3.sessiontype = "ProgRatio";
            fed3.DisplayPokes = true;
            fed3.DisplayTimed = false;
            poke_num = 0;
            pokes_required = 1;
            fed3.FR = 1;
            fed3.UpdateDisplay();
            Serial.println("Mode set to Progressive Ratio");
        }
        else if (mode_params == "FREE") {
            fed3.FEDmode = 0;
            fed3.sessiontype = "Free_feed";
            fed3.DisplayPokes = false;
            fed3.DisplayTimed = false;
            fed3.UpdateDisplay();
            Serial.println("Mode set to Free Feeding");
        }
        else if (mode_params == "EXTINCT") {
            fed3.FEDmode = 5;
            fed3.sessiontype = "Extinct";
            fed3.DisplayPokes = true;
            fed3.DisplayTimed = false;
            fed3.UpdateDisplay();
            Serial.println("Mode set to Extinction");
        }
        else if (mode_params == "LIGHTTRK") {
            fed3.FEDmode = 6;
            fed3.sessiontype = "Light Trk";
            fed3.DisplayPokes = true;
            fed3.DisplayTimed = false;
            fed3.disableSleep();
            fed3.UpdateDisplay();
            Serial.println("Mode set to Light Tracking");
        }
        else if (mode_params.startsWith("FR1_R") || mode_params.startsWith("FR_R")) {
            int comma_idx = mode_params.indexOf(',');
            int ratio = 1;
            if (comma_idx != -1) {
                ratio = mode_params.substring(comma_idx + 1).toInt();
            }
            if (ratio < 1) ratio = 1;
            fed3.FEDmode = 7;
            fed3.FR = ratio;
            if (ratio == 1) {
                fed3.sessiontype = "FR1_R";
            } else {
                fed3.sessiontype = "FR" + String(ratio) + "_R";
            }
            fed3.DisplayPokes = true;
            fed3.DisplayTimed = false;
            fed3.activePoke = 0;
            fed3.UpdateDisplay();
            Serial.print("Mode set to FR Reversed. FR: ");
            Serial.println(fed3.FR);
        }
        else if (mode_params == "PR_R") {
            fed3.FEDmode = 8;
            fed3.sessiontype = "PR_R";
            fed3.DisplayPokes = true;
            fed3.DisplayTimed = false;
            fed3.activePoke = 0;
            poke_num = 0;
            pokes_required = 1;
            fed3.FR = 1;
            fed3.UpdateDisplay();
            Serial.println("Mode set to PR Reversed");
        }
        else if (mode_params == "OPTO") {
            fed3.FEDmode = 9;
            fed3.sessiontype = "OptoStim";
            fed3.DisplayPokes = true;
            fed3.DisplayTimed = false;
            fed3.UpdateDisplay();
            Serial.println("Mode set to Opto Stimulation");
        }
        else if (mode_params == "OPTO_R") {
            fed3.FEDmode = 10;
            fed3.sessiontype = "OptoStim_R";
            fed3.DisplayPokes = true;
            fed3.DisplayTimed = false;
            fed3.activePoke = 0;
            fed3.UpdateDisplay();
            Serial.println("Mode set to Opto Stimulation Reversed");
        }
        else if (mode_params == "TIMED") {
            fed3.FEDmode = 11;
            fed3.sessiontype = "Timed";
            fed3.DisplayPokes = false;
            fed3.DisplayTimed = true;
            fed3.UpdateDisplay();
            Serial.println("Mode set to Timed Feeding");
        }
    }
  }
}
