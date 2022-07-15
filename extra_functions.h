#include <stdio.h>

// function to sort the array in ascending order
int Array_median(long *array , int n)
{ 
    // declare some local variables
    int i=0 , j=0 , temp=0;

    for(i=0 ; i<n ; i++)
    {
        for(j=0 ; j<n-1 ; j++)
        {
            if(array[j]>array[j+1])
            {
                temp        = array[j];
                array[j]    = array[j+1];
                array[j+1]  = temp;
            }
        }
    }
/*
    Serial.println("\nThe array after sorting is..\n");
    for(i=0 ; i<n ; i++)
    {
        Serial.println("\narray_1[%d] : %d",i,array[i]);
    } */

    long median=0;
    
    // if number of elements are even
    if(n%2 == 0)
        median = (array[(n-1)/2] + array[n/2])/2.0;
    // if number of elements are odd
    else
        median = array[n/2];
    
    return median;
    //Serial.print("The median is ");
    //Serial.println(median);
}

void DeepSleepMode() 
{
  delay(100);
  looptime = millis() - loopstart;
  SP.write("Loop duration: ");
  SP.print(looptime);
  SP.println(" ms");
  if(looptime>loopintv){
    sleeptime = 10000;
  }
  else{
  sleeptime = loopintv - looptime;
  }
  SP.write("Sleep time: ");
  SP.print(sleeptime);
  SP.println(" ms");
  if (sleeptime < 10000) {
    sleeptime = 10000;
    SP.println("Warning! Sleeptime is too low - was set to 10000 ms");
  }

  SP.println("Setting digital ports for sleep:");
  for (int i = 0; i <= 7; i++) {
    SP.print(i);
    pinMode(i, INPUT);
    digitalWrite(i, LOW);
    SP.println(": Input, LOW");
  }
  
  SP.println("Sleeping..");
  delay(sleeptime);
  //LowPower.deepSleep(sleeptime);
  Wire.end();
}

void SensorWakeUp() 
{
  Wire.begin();
  SP.println("Setting digital ports for wakeup:");

  for (int i = 0; i <= 7; i++) {
    SP.print(i);
    if (i == BATT_DPORT || i == SW33_A_DPORT || i == SW33_B_DPORT || i == SW12_DPORT || i == LED_DPORT || i == IMU_DPORT || i == StpCtr1_DPORT) {        // IMU_DPORT HERE
      pinMode(i, OUTPUT);
      SP.print(": Output");
      SP.print(" \t");
    }
    else {
      if (i == LED_BUILTIN) {                     // SET LED port to output low, all other unused ports to INPUT_PULLUP to save energy
        pinMode(i, INPUT);
        SP.print(": Input (LED Off)");
        SP.print(" \t");
      }
      else {
        pinMode(i, INPUT_PULLUP);
        SP.print(": INPUT_PULLUP");
        SP.print(" \t");
      }
    }
  }
}

void K1_AllOff () {             // turn off relais for channel 1
  digitalWrite(A2, LOW);        
  digitalWrite(A3, LOW);
}

void K1_TurnA () {              // turn on measurement resistor for ADS channel 1 
  digitalWrite(A2, HIGH);       
  digitalWrite(A3, LOW);  
  delay(50);
  digitalWrite(A2, LOW);    
}

void K1_TurnB () {              // turn off measurement resistor for ADS channel 1
  digitalWrite(A2, LOW);        
  digitalWrite(A3, HIGH);
  delay(50);
  digitalWrite(A3, LOW);      
}

void K2_AllOff () {             // turn off relais for channel 2
  digitalWrite(A4, LOW);
  digitalWrite(A5, LOW);
}

void K2_TurnA () {              // turn on measurement resistor for ADS channel 2
  digitalWrite(A4, HIGH);       
  digitalWrite(A5, LOW); 
    delay(50);
  digitalWrite(A4, LOW);       
}

void K2_TurnB () {              // turn off measurement resistor for ADS channel 2
  digitalWrite(A4, LOW);       
  digitalWrite(A5, HIGH); 
  delay(50);
  digitalWrite(A5, LOW);
}
