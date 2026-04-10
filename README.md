 // open the Serial port for the 1st example   
  Serial.begin(57600);  

    // open the Serial port for the 2nd example  
  Serial.begin(115200);  
    
    Serial.println("? Help");  
    Serial.println("+ increase volume");  
    Serial.println("- decrease volume");  
    Serial.println("> next preset");  
    Serial.println("< previous preset");  
    Serial.println(". scan up   : scan up to next sender");  
    Serial.println(", scan down ; scan down to next sender");  
    Serial.println("fnnnnn: direct frequency input");  
    Serial.println("i station status");  
    Serial.println("s mono/stereo mode");  
    Serial.println("b bass boost");  
    Serial.println("u mute/unmute");  
