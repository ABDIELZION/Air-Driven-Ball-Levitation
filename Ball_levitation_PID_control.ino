
const int trigPin = 12;
const int echoPin = 11;
const int speedPin = 9; 
const int in1 = 8;      
const int in2 = 7;      

double Kp = 10.0;   
double Ki = 0.02;  
double Kd = 6.0;   

const int TUBE_LENGTH = 30; 
const int BASE_PWM = 95;    

double targetDistanceFromTop = 15.0; 
double currentDist, output, error, lastError, cumError, rateError;
unsigned long previousTime;
bool systemRunning = false;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(speedPin, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(speedPin, 0);

  Serial.println("--- System Ready ---");
  Serial.println("Press '+' to START | Press '-' to STOP");
}

void loop() {

  if (Serial.available() > 0) {
    char input = Serial.read();

    if (input == '+') {
      systemRunning = true;
      previousTime = millis(); 
      cumError = 0;          
      Serial.println(">>> STARTING: PID Control Active");
    } 
    else if (input == '-') {
      systemRunning = false;
      analogWrite(speedPin, 0); 
      Serial.println("<<< STOPPED: Fan Off / Sensors Idle");
    }
  }

 
  if (systemRunning) {
    currentDist = getDistance();
    
    unsigned long currentTime = millis();
    double dt = (double)(currentTime - previousTime) / 1000.0;

    
    error = currentDist - targetDistanceFromTop; 
    cumError += error * dt;
    rateError = (error - lastError) / dt;

  
    output = BASE_PWM + (Kp * error) + (Ki * cumError) + (Kd * rateError);

   
    if (output > 200) output = 200;
    if (output < 0) output = 0;

    analogWrite(speedPin, (int)output);


    Serial.print("Target:"); Serial.print(targetDistanceFromTop);
    Serial.print(" Current:"); Serial.print(currentDist);
    Serial.print(" PWM:"); Serial.println(output);

    lastError = error;
    previousTime = currentTime;
  }

  delay(50); 
}

float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 26000); 
  float d = (duration * 0.034) / 2;
  
  if (d <= 0 || d > TUBE_LENGTH) return TUBE_LENGTH; 
  return d;
}