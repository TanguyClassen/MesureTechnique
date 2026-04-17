#include "HX711.h" 
 
const int LOADCELL_DOUT_PIN = 2; 
const int LOADCELL_SCK_PIN = 5; 
 
HX711 scale; 
float calibration_factor = 1000; // Adjust based on your calibration 
 
void setup() { 
  Serial.begin(9600); 
  Serial.println("========================================"); 
  Serial.println("      HX711 Load Cell Scale Demo       "); 
  Serial.println("========================================"); 
   
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN); 
   
  Serial.println("Initializing scale..."); 
  scale.set_scale(); 
  scale.tare(); 
   
  Serial.println("Setup complete!"); 
  Serial.println("Place weights on scale for measurement"); 
  Serial.println("========================================"); 
   
  // Apply calibration 
  scale.set_scale(1000); 
  scale.tare(); 
} 
 
void loop() { 
  if (scale.is_ready()) { 
    float weight = scale.get_units(10); // Average of 10 readings 
     
    Serial.print("Weight: "); 
    Serial.print(weight, 2); 
    Serial.print(" g | "); 
    Serial.print(weight/1000, 3); 
    Serial.print(" kg | "); 
    Serial.print(weight*0.00220462, 3); 
    Serial.println(" lbs"); 
     
    // Display raw reading for debugging 
    Serial.print("Raw reading: "); 
    Serial.println(scale.read()); 
     
    Serial.println("------------------------"); 
  } else { 
    Serial.println("Scale not ready - check connections"); 
  } 
   
  delay(500); 
}
