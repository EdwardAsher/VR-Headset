//Jonel Alcasid
#include <IRremote.h>

const int RECV_PIN = 7;
int flag =0;
const int led = 12;
IRrecv irrecv(RECV_PIN);
decode_results results;
unsigned long key_value = 0;

void setup()
{
  pinMode(led, OUTPUT);
  Serial.begin(115200);
  irrecv.enableIRIn();
  irrecv.blink13(true);
}

void loop()
{
  if (flag==0)
  {
    digitalWrite(led, LOW);
  }
  else
  {
    digitalWrite(led, HIGH);
  }
  if (irrecv.decode(&results)){

        if (flag==0)
        {
          flag=1;
        }

        else if(flag==1)
        {
          flag=0;
        }
 
        if (results.value == 0XFFFFFFFF)
          results.value = key_value;

        switch(results.value){
          case 0xFFA25D:
          Serial.write(1);
          break;
          case 0xFF629D:
          Serial.write(2);
          break;
          case 0xFFE21D:
          Serial.write(3);
          break;
          case 0xFF22DD:
          Serial.write(4);
          break;
          case 0xFF02FD:
          Serial.write(5);
          break ;  
          case 0xFFC23D:
          Serial.write(6);
          break ;               
          case 0xFFE01F:
          Serial.write(7);
          break ;  
          case 0xFFA857:
          Serial.write(8);
          break ;  
          case 0xFF906F:
          Serial.write(9);
          break ;  
          case 0xFF6897:
          Serial.write(10);
          break ;  
          case 0xFF9867:
          Serial.write(11);
          break ;
          case 0xFFB04F:
          Serial.write(12);
          break ;
          case 0xFF30CF:
          Serial.write(13);
          break ;
          case 0xFF18E7:
          Serial.write(14);
          break ;
          case 0xFF7A85:
          Serial.write(15);
          break ;
          case 0xFF10EF:
          Serial.write(16);
          break ;
          case 0xFF38C7:
          Serial.write(17);
          break ;
          case 0xFF5AA5:
          Serial.write(18);
          break ;
          case 0xFF42BD:
          Serial.write(19);
          break ;
          case 0xFF4AB5:
          Serial.write(20);
          break ;
          case 0xFF52AD:
          Serial.write(21);
          break ;      
        }
        digitalWrite(LED_BUILTIN, LOW);
        key_value = results.value;
        irrecv.resume(); 
        delay(400);
  }
}

