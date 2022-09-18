
float velocity = 0.0;
float angle = 0.0;

int enA = 5; 
int enB = 6;

int in1 = 7;
int in2 = 8;
int in3 = 9;
int in4 = 10;


void setup() {
    Serial.begin(115200);
    
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

void loop() {
    commuincation();
    
    if (velocity > 0)
    {
        forward();
    }
    else if (velocity < 0)
    {
        back();
    }

    if (angle > 0)
    {
        right();
    }
    else if (angle < 0)
    {
        left();
    }
    else
    {
      brake();
    }
}


void forward()
{
    analogWrite(enA, velocity);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
}

void back()
{
    analogWrite(enA, 250-velocity);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
}

void right()
{
    analogWrite(enB, 200);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    analogWrite(enA, velocity);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
}

void left()
{
    analogWrite(enB, 200);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    analogWrite(enA, velocity);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
}

void brake()
{ 
    analogWrite(enA,0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    
    analogWrite(enB,0);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

void commuincation()
{
    if(Serial.available()>0)
    {
        String msg = Serial.readStringUntil('\n');
        // Split string into two values
        for (int i = 0; i < msg.length(); i++)
        {
            if (msg.substring(i, i+1) == ",")
            {
            velocity = msg.substring(0, i).toInt();
            angle = msg.substring(i+1).toInt();
            break;
            }
        }
        delay(50);
    }
}
