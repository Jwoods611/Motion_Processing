#include <Wire.h> //Include library for I2C communication
#include <math.h> //Include library used for math calculations (power)

#define ADDR 0x68 //MPU6050 I2C address
#define LOW_PASS_ADR 0x1A //Address for register (low-pass filter)
#define LOW_PASS 0x04 //Mask used to implement low-pass filter
#define FIRST_REG_ADDR 0x3B //First register to have data read from
#define BYTES 14 //Number of consecutive bytes to be read from MPU
#define THRESHOLD 0x03 //Threshold used for low-pass filtering

double angle = 0.0; //init angle of rotation
float numPoints = 0; //init number of points for numerical integration
float totalValue = 0; //init sum of values for numerical integration
float currentMillis; //init elapsed time
float endMillis; //init time passed for numerical integration

/**
 * Creates a structure that holds two bytes of type int.
 * Will be used to create a union that holds both bytes in one
 * memory location, so both can be accessed by data.xWord
 */
 struct wordStruct {
    int lo : 8; // low byte
    int hi : 8; // high byte
  };

/**
 * Holds two bytes to be accessed by a single pointer/in
 * a single memory location. This allows the two bytes declared in
 * wordStruct to be referenced/addressed as one variable, xWord.
 * Both variables (lo, hi) can be referenced with data.xWord
 */
  union wordUnion {
    struct wordStruct xByte; // two bytes
    short xWord; // signed 16-bit int
  };
/**
 * Struct that holds 3 variables of type float. This will be used to
 * hold the gyroscope and accelerometer data in all 3 axes. Can be   
 * called with accel.x, accel.y, gyro.x, gyro.y, etc.
 */
  struct xyzStruct {
    float x;
    float y;
    float z;
  } accel, gyro;
  
  union wordUnion data; //declare a union of two bytes
  short temp; //declare variable used for temperature data
  float currentMax = 0.0; //init current max noise (z-axis) to 0
  float currentMin = 0.0; //init current min noise (z-axis) to 0
  
void setup() {
  Serial.begin(57600);
  Serial.println("Starting up...");
  Wire.beginTransmission(ADDR); // send address intending to write 
  Wire.write(107); // send the register number 
  Wire.write(0x80); // send the value of the register (for reset)
  Wire.endTransmission(); // end the transmission

  delay(500); //delay for 500ms

  Wire.beginTransmission(ADDR); // send address intending to write 
  Wire.write(107); // send the register number 
  Wire.write(0x00); // send the value of the register 
  Wire.endTransmission(); // end the transmission


  Wire.beginTransmission(ADDR); // send address intending to write 
  Wire.write(LOW_PASS_ADR); // send the register number 
  Wire.write(LOW_PASS); // send the value of the register 
  Wire.endTransmission(); // end the transmission
  delay(1000);
}

void loop() {
  Wire.beginTransmission(ADDR); // send MPU6050 device address
  Wire.write(FIRST_REG_ADDR); // send the first register number 
  Wire.endTransmission(); // end the transmission 
  Wire.requestFrom(ADDR,BYTES,true); // BYTES = consecutive bytes read

  
  while(Wire.available()){
    data.xByte.hi = Wire.read(); //form a union of first/second bytes
    data.xByte.lo = Wire.read(); //form a union of first/second bytes
    accel.x = data.xWord; //set accelerometer x-data to these bytes
    accel.x = accel.x * (4*pow(2, -16)); //convert to real world units
    
    data.xByte.hi = Wire.read(); //form a union of third/fourth bytes
    data.xByte.lo = Wire.read(); //form a union of third/fourth bytes
    accel.y = data.xWord; //set accelerometer y-data to these bytes
    accel.y = accel.y * (4*pow(2, -16)); //convert to real world units
    
    data.xByte.hi = Wire.read(); //form a union of fifth/sixth bytes
    data.xByte.lo = Wire.read(); //form a union of fifth/sixth bytes
    accel.z = data.xWord; //set accelerometer z-data to these bytes
    accel.z = accel.z * (4*pow(2, -16)); //convert to real world units
    
    data.xByte.hi = Wire.read(); //form union of seventh/eighth bytes
    data.xByte.lo = Wire.read(); //form union of seventh/eighth bytes
    temp = data.xWord; //set the temperature to these two bytes
    temp = (temp / 340) + 36.53; //convert to degrees Celsius

    data.xByte.hi = Wire.read(); //form a union of ninth/tenth bytes
    data.xByte.lo = Wire.read(); //form a union of ninth/tenth bytes
    gyro.x = data.xWord; //set the gyro's x-axis data to these bytes
    gyro.x = gyro.x * (500 * pow(2, -16)); //real world units

    data.xByte.hi = Wire.read(); //union of eleventh/twelfth bytes
    data.xByte.lo = Wire.read(); //union of eleventh/twelfth bytes
    gyro.y = data.xWord; //set the gyro's y-axis data to these bytes
    gyro.y = gyro.y * (500 * pow(2, -16)); //real world units

    data.xByte.hi = Wire.read(); //thirteenth/fourteenth bytes
    data.xByte.lo = Wire.read(); //thirteenth/fourteenth bytes
    gyro.z = data.xWord; //set the gyro's z-axis data to these bytes
    gyro.z = gyro.z * (500 * pow(2, -16)); //real world units

    checkMax(); //check/set the maximum gyro z-axis noise encountered
    checkMin(); //check/set the minimum gyro z-axis noise encountered

    /**
     * The following print statement prints the data retrieved from 
     * the MPU6050. This is the format:
     * accel.x|accel.y|accel.z|temp|gyro.x|gyro.y|gyro.z|maxZ|minZ
     */
    Serial.println(String(accel.x) + "\t" + String(accel.y) + "\t" +   
    String(accel.z) + "\t" + "/" // accel values
    + "\t" + String(temp) + "\t" + "/" + "\t" // temperature value                                                    
    + String(gyro.x) + "\t" + String(gyro.y) + "\t" + String(gyro.z) 
    +"\t" + " / " + String(currentMax) + "\t" + " / "   
    +  String(currentMin));    // gyro values
  }

/**
 * The following 2 while loops calculate the angle of rotation by  
 * numerical integration (calculus). Specifically, they wait until the   
 * gyroscope's z-axis is greater than some threshold (3 for us).
 * Once it is, it adds up the total z-axis data retrieved during this   
 * period of time, as well as keeps track of the number of times data 
 * is retrieved, and the time elapsed while the data is greater than 
 * the threshold. Finally, the angle of rotation is determined by 
 * multiplying the total value of the data by the time elapsed, and  
 * dividing it by the number of times data was retrieved. This gives
 * us the angle the gyroscope was rotated.
 */
  while (abs(gyro.z) > THRESHOLD) 
  {
    angle=0;
    numPoints = 0;
    totalValue = 0;
    currentMillis = millis(); //gives time since device booted up

    while (abs(gyro.z) > THRESHOLD) {
      gyro.z = getGyroZ();

      totalValue += gyro.z;
      numPoints++;
      delay(100);
    }

    endMillis = millis() - currentMillis; //gives rotation time
    Serial.println("End millis: " +String(endMillis));

    Serial.println("Total value: " +String(totalValue));
    Serial.println("Num points: " +String(numPoints));

  
    angle = (totalValue* (endMillis/1000))/numPoints;//calculate angle
    Serial.println("Angle rotated: " + String(angle));
  }
}

/**
 * Function to retrieve the gyroscope data on the z-axis. Used to 
 * reassign the z-axis gyroscope data in the use of a low-pass filter  
 * in the main loop.
 */
float getGyroZ() 
{
  Wire.beginTransmission(ADDR); // send address intending to write 
  Wire.write(FIRST_REG_ADDR+0x0C); // send gyroZ register address 
  Wire.endTransmission(); // end the transmission 
  Wire.requestFrom(ADDR,2,true); //Request 2 bytes,then release SDA

  data.xByte.hi = Wire.read(); //set hi byte equal to first byte read 
  data.xByte.lo = Wire.read(); //set lo byte equal to second byte read 
  gyro.z = data.xWord; //gyroscope z-axis set to union of 2 bytes
  gyro.z = gyro.z * (500 * pow(2, -16)); //convert to real world units

  Serial.println(gyro.z); //Finally, print the z-axis data
  
  return gyro.z; //Return the gyroscope's z-axis data
}

/**
 * Function to check/set the maximum value of the gyroscope
 * data on the z-axis. The data is used to determine the noise of
 * the z-axis of the gyroscope. Sets a new maximum if the value seen 
 * is higher than the current maximum value.
 */
void checkMax () {
  if (gyro.z > currentMax) {
    currentMax = gyro.z;
  }
}

/**
 * Function used to check/set the minimum value of the gyroscope
 * data on the z-axis. The data is used to determine the noise of the
 * z-axis of the gyroscope. Sets a new minimum if the value seen is 
 * lower than the current minimum value.
 */
bool checkMin () {
  if (gyro.z < currentMin) {
    currentMin = gyro.z;
  }
}

