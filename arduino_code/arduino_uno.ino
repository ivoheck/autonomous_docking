int lock_controll = 4;


void setup() {
  Serial.begin(9600);
  pinMode(lock_controll, OUTPUT);
}

void open_lock(){
  digitalWrite(lock_controll, HIGH); 
}

void close_lock(){
  digitalWrite(lock_controll, LOW); 
}

void loop() {
  if (Serial.available()) {
    char inChar = (char)Serial.read();

    if (inChar == '1'){
      open_lock();
    }else{
      close_lock();
    }
  }
}
