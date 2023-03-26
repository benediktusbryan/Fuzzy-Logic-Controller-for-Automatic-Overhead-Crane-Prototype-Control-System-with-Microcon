/*
CODING TUGAS AKHIR
PENGENDALIAN POSISI DAN SISTEM ANTI-SWAY
PADA PURWARUPA OVERHEAD CRANE OTOMATIS MENGGUNAKAN ELEKTROMAGNET


Dibuat Tahun 2019
Oleh Benediktus Bryan Bimantoro (21060115140105)
     Arbian Aji Prabowo         (21060115130076)
     Dwi Maulida Puspita Sari   (21060115120046)

Pembimbing Mochammad Facta, S.T., M.T., Ph.D.
           Ir. Sudjadi, M.T.
*/

//-----DEKLARASI FUNGSI-------------------------------------------------------------------------------------------------------------------------------------
#define posisiLongMin 0
#define posisiLongMax 116
#define posisiTransMin 0
#define posisiTransMax 72

//-----DEKLARASI PIN-------------------------------------------------------------------------------------------------------------------------------------

#define encTransA PB9
#define encTransB PB8
#define encLongA PB6
#define encLongB PB7
#define limitLongMin PA8
#define limitLongMax PB13
#define limitTransMin PB14
#define limitTransMax PB15
#define relay PA2
#define arahTrans PA5
#define pwmTrans PA6
#define arahLong PA4
#define pwmLong PB0
#define arahHoist PA0
#define pwmHoist PA1
#define pbEmergency PA7

//-----DEKLARASI VARIABEL-------------------------------------------------------------------------------------------------------------------------------------

static int posisiLong=0, posisiTrans=0, state=0;
float outX, outY;
long timerReport=0, timerSteady=0;
bool magnet, emergency, stateBaca, stateKontrol;
String dataIn, dataOut, antiSway;
char protokol;
String str_xAkhir, str_yAkhir, str_x, str_y, str_z, str_magnet, inX_string, inY_string;

int ind1, ind2, ind3, ind4, ind5, ind6, ind7, ind8, pwmManualX, pwmManualY;
float inX, inY, inZ, errorX, errorY, lastErrorX, lastErrorY, dErrorX, dErrorY; //input
float NB_errorX, NS_errorX, Z_errorX, PS_errorX, PB_errorX; //derajat
float NB_dErrorX, NS_dErrorX, Z_dErrorX, PS_dErrorX, PB_dErrorX; //derajat
float NB_errorY, NS_errorY, Z_errorY, PS_errorY, PB_errorY; //derajat
float NB_dErrorY, NS_dErrorY, Z_dErrorY, PS_dErrorY, PB_dErrorY; //derajat
float zx1, zx2, zx3, zx4, zx5, zx6, zx7, zx8, zx9, zx10, zx11, zx12, zx13, zx14, zx15, zx16, zx17, zx18, zx19, zx20, zx21, zx22, zx23, zx24, zx25; //output
float wx1, wx2, wx3, wx4, wx5, wx6, wx7, wx8, wx9, wx10, wx11, wx12, wx13, wx14, wx15, wx16, wx17, wx18, wx19, wx20, wx21, wx22, wx23, wx24, wx25; //fungsi pembobotan
float zy1, zy2, zy3, zy4, zy5, zy6, zy7, zy8, zy9, zy10, zy11, zy12, zy13, zy14, zy15, zy16, zy17, zy18, zy19, zy20, zy21, zy22, zy23, zy24, zy25; //output
float wy1, wy2, wy3, wy4, wy5, wy6, wy7, wy8, wy9, wy10, wy11, wy12, wy13, wy14, wy15, wy16, wy17, wy18, wy19, wy20, wy21, wy22, wy23, wy24, wy25; //fungsi pembobotan

float offsetDegX = 0.0;
float offsetDegY = 0.0;
float degXOri, degYOri;
float degX, degY, errorDegX, errorDegY, dErrorDegX, dErrorDegY, lastEDegX, lastEDegY;
String dataInBT,degXBT,degYBT;
int index1,index2,index3;
float NB_errorDegX, NS_errorDegX, Z_errorDegX, PS_errorDegX, PB_errorDegX; //derajat
float NB_dErrorDegX, NS_dErrorDegX, Z_dErrorDegX, PS_dErrorDegX, PB_dErrorDegX; //derajat
float zDegX1, zDegX2, zDegX3, zDegX4, zDegX5, zDegX6, zDegX7, zDegX8, zDegX9, zDegX10, zDegX11, zDegX12, zDegX13, zDegX14, zDegX15, zDegX16, zDegX17, zDegX18, zDegX19, zDegX20, zDegX21, zDegX22, zDegX23, zDegX24, zDegX25; //output
float wDegX1, wDegX2, wDegX3, wDegX4, wDegX5, wDegX6, wDegX7, wDegX8, wDegX9, wDegX10, wDegX11, wDegX12, wDegX13, wDegX14, wDegX15, wDegX16, wDegX17, wDegX18, wDegX19, wDegX20, wDegX21, wDegX22, wDegX23, wDegX24, wDegX25; //fungsi pembobotan
float NB_errorDegY, NS_errorDegY, Z_errorDegY, PS_errorDegY, PB_errorDegY; //derajat
float NB_dErrorDegY, NS_dErrorDegY, Z_dErrorDegY, PS_dErrorDegY, PB_dErrorDegY; //derajat
float zDegY1, zDegY2, zDegY3, zDegY4, zDegY5, zDegY6, zDegY7, zDegY8, zDegY9, zDegY10, zDegY11, zDegY12, zDegY13, zDegY14, zDegY15, zDegY16, zDegY17, zDegY18, zDegY19, zDegY20, zDegY21, zDegY22, zDegY23, zDegY24, zDegY25; //output
float wDegY1, wDegY2, wDegY3, wDegY4, wDegY5, wDegY6, wDegY7, wDegY8, wDegY9, wDegY10, wDegY11, wDegY12, wDegY13, wDegY14, wDegY15, wDegY16, wDegY17, wDegY18, wDegY19, wDegY20, wDegY21, wDegY22, wDegY23, wDegY24, wDegY25; //fungsi pembobotan
float dErrorAngleX, dErrorAngleY, lastErrorAngleX, lastErrorAngleY;

int fuzzyOutputPositionX, fuzzyOutputPositionY, fuzzyOutputAntiSwayX, fuzzyOutputAntiSwayY, pwmX, unsPwmX, unsPwmY, pwmY; //hasil fuzzy

//-----DEKLARASI FUNGSI-------------------------------------------------------------------------------------------------------------------------------------

void NGTLongA();
void NGTTransA();
void cekLimitSwitch();
void cekEmergency();
float mapFloat(float x, float fromLow, float fromHigh, float toLow, float toHigh);
void kontrolPosisiX();
void fuzzyficationX();
void ruleBaseX();
void defuzzyficationX();
void kontrolPosisiY();
void fuzzyficationY();
void ruleBaseY();
void defuzzyficationY();
void kontrolDegX();
void fuzzyficationDegX();
void ruleBaseDegX();
void defuzzyficationDegX();
void kontrolDegY();
void fuzzyficationDegY();
void ruleBaseDegY();
void defuzzyficationDegY();
void terimaDataPC();
void kirimDataPC();
void terimaDataBluetooth();
void Serial3Monitor();


//-----PROGRAM UTAMA-------------------------------------------------------------------------------------------------------------------------------------

void setup() {
  pinMode(relay, OUTPUT);
  pinMode(arahLong, OUTPUT);
  pinMode(arahTrans, OUTPUT);
  pinMode(arahHoist, OUTPUT);
  pinMode(pwmLong, OUTPUT);
  pinMode(pwmTrans, OUTPUT);
  pinMode(pwmHoist, OUTPUT);        
    
  digitalWrite(relay, LOW); //active low nc
  analogWrite(pwmLong, 0);
  analogWrite(pwmTrans, 0);
  analogWrite(pwmHoist, 0);

  attachInterrupt(encLongA,NGTLongA,FALLING);
  attachInterrupt(encTransA,NGTTransA,FALLING);
      
  Serial.begin(115200);
  Serial1.begin(9600);
  Serial3.begin(57600);

  delay(1000);
  posisiLong=0, posisiTrans=0, state=1;
}

void loop() {
//Serial1.print("start: ");Serial1.println(millis());  

  cekLimitSwitch();
  if (pwmX<0){
    digitalWrite(arahLong, LOW);
    unsPwmX=pwmX*(-1);
  } else {
    digitalWrite(arahLong, HIGH);
    unsPwmX=pwmX;
  } 
  if (pwmY<0){
    digitalWrite(arahTrans, LOW);
    unsPwmY=pwmY*(-1);
  } else {
    digitalWrite(arahTrans, HIGH);
    unsPwmY=pwmY;
  }
  analogWrite(pwmLong, unsPwmX);
  analogWrite(pwmTrans, unsPwmY);
  
  terimaDataPC();
  terimaDataBluetooth();
  cekEmergency();
    
  if (dataIn!=""){
    protokol=dataIn.charAt(0);
    if (protokol=='#'){      
      ind1=dataIn.indexOf(',');
      ind2=dataIn.indexOf(',', ind1+1);
      str_x=dataIn.substring(ind1+1, ind2);
      ind3=dataIn.indexOf(',', ind2+1);
      str_y=dataIn.substring(ind2+1, ind3);
      ind4=dataIn.indexOf(',', ind3+1);
      str_z=dataIn.substring(ind3+1, ind4);
      ind5=dataIn.indexOf(',', ind4+1);
      str_magnet=dataIn.substring(ind4+1, ind5);
      ind6=dataIn.indexOf(',', ind5+1);
      antiSway=dataIn.substring(ind5+1, ind6);

      if (str_x=="-"){
        pwmManualX=-255;   
        state= 4;
      } else if (str_x=="+"){
        pwmManualX=255;
        state= 4;   
      } else{  
        pwmManualX=0;
        pwmX=0;
        analogWrite(pwmLong,0);  
      }

      if (str_y=="-"){       
        pwmManualY=-255;
        Serial3.println("y min");
        state= 5;   
      } else if (str_y=="+"){  
        pwmManualY=255;
        Serial3.println("y plus");
        state= 5;   
        } else{  
        pwmY=0;
        pwmManualY=0;
        analogWrite(pwmTrans,0);
        Serial3.println("y stop");  
      }

      if (str_z=="-"){  
        digitalWrite(arahHoist, LOW);
        analogWrite(pwmHoist, 255);
        Serial3.println("z min");
        state=1;               
      } else if (str_z=="+"){  
        digitalWrite(arahHoist, HIGH);
        analogWrite(pwmHoist, 255);
        Serial3.println("z plus");
        state=1;   
      } else{
        analogWrite(pwmHoist, 0);
        Serial3.println("z stop");  
      }
          
      if (str_magnet=="1"){  
        detachInterrupt(encLongA);
        detachInterrupt(encTransA);
        digitalWrite(relay, HIGH);
        Serial3.println("Magnet On");      
        delay(100);          
        attachInterrupt(encLongA,NGTLongA,FALLING);
        attachInterrupt(encTransA,NGTTransA,FALLING);                  
      } else{
        detachInterrupt(encLongA);
        detachInterrupt(encTransA);      
        digitalWrite(relay, LOW);
        Serial3.println("Magnet Lepas");  
        delay(100);          
        attachInterrupt(encLongA,NGTLongA,FALLING);
        attachInterrupt(encTransA,NGTTransA,FALLING);      
      }        
    } else if (protokol=='*'){
      ind1=dataIn.indexOf(',');
      ind2=dataIn.indexOf(',', ind1+1);
      inX_string=dataIn.substring(ind1+1, ind2);
      ind3=dataIn.indexOf(',', ind2+1);
      inY_string=dataIn.substring(ind2+1, ind3);
      ind4=dataIn.indexOf(',', ind3+1);
      antiSway=dataIn.substring(ind3+1, ind4);
      
      inX= inX_string.toInt();
      inY= inY_string.toInt();
      state= 2;    
    }
    dataIn="";
  }
  if (stateKontrol==1){
    switch(state){
      case 0: //Reset Posisi
      if(digitalRead(limitLongMin)==1){
        digitalWrite(arahLong, LOW);
        analogWrite(pwmLong, 255);
      } else {
        posisiLong=posisiLongMin;
        analogWrite(pwmLong, 0);
      }  
      if(digitalRead(limitTransMin)==1){
        digitalWrite(arahTrans, LOW);
        analogWrite(pwmTrans, 255);
      } else {
        posisiTrans=posisiTransMin;
        analogWrite(pwmTrans, 0);
      }
      if(digitalRead(limitLongMin)==0 && digitalRead(limitTransMin)==0){
        state=1;
      }
    break;

    case 1: //Crane Diam
      pwmX=0;
      pwmY=0;
      analogWrite(pwmLong, 0);
      analogWrite(pwmTrans, 0);
    break;
  
    case 2: //Auto Longitudinal
      pwmY=0;
      analogWrite(pwmTrans, 0);
      kontrolPosisiX();
      if (antiSway=="1"){
        kontrolDegX();
        pwmX=fuzzyOutputPositionX+fuzzyOutputAntiSwayX;
        if(pwmX<50 && pwmX>-50){
          pwmX=0;
        } else if (pwmX>255){
        pwmX=255;
        } else if (pwmX<-255){
        pwmX=-255;
        }
      } else if (antiSway=="0"){
        pwmX=fuzzyOutputPositionX;
        if(pwmX<60 && pwmX>-60){
          pwmX=0;
        }
      }
      
      cekLimitSwitch();

      if (pwmX<0){
        digitalWrite(arahLong, LOW);
        unsPwmX=pwmX*(-1);
      } else {
        digitalWrite(arahLong, HIGH);
        unsPwmX=pwmX;
      }
      analogWrite(pwmLong, unsPwmX);
      
      if(pwmX!=0){
        timerSteady = millis();
      } else{
        if(millis() - timerSteady > 2000){
          state=3;}
      }      
    break;
               
    case 3: //Auto Transversal
      pwmX=0;
      analogWrite(pwmLong, 0);
      kontrolPosisiY();
      if (antiSway=="1"){
        kontrolDegY();
        pwmY=fuzzyOutputPositionY+fuzzyOutputAntiSwayY;
        if(pwmY<40 && pwmY>-40){
         pwmY=0;
        } else if (pwmY>255){
          pwmY=255;
        } else if (pwmY<-255){
          pwmY=-255;
        }
      } else if (antiSway=="0"){
        pwmY=fuzzyOutputPositionY;
        if(pwmY<60 && pwmY>-60){
          pwmY=0;
        }
      }

      cekLimitSwitch();
  
      if (pwmY<0){
        digitalWrite(arahTrans, LOW);
        unsPwmY=pwmY*(-1);
      } else {
        digitalWrite(arahTrans, HIGH);
        unsPwmY=pwmY;
      } 
      analogWrite(pwmTrans, unsPwmY);
    break;    
  
    case 4: //Manual Longitudinal
      pwmY=0;
      analogWrite(pwmTrans, 0);
      if (antiSway=="1"){
        kontrolDegX();
        pwmX=pwmManualX+fuzzyOutputAntiSwayX;
        if(pwmX<50 && pwmX>-50){
          pwmX=0;
        } else if (pwmX>255){
          pwmX=255;
        } else if (pwmX<-255){
          pwmX=-255;
        }
      } else if (antiSway=="0"){
        pwmX=pwmManualX;
      }
      
      cekLimitSwitch();

      if (pwmX<0){
        digitalWrite(arahLong, LOW);
        unsPwmX=pwmX*(-1);
      } else {
        digitalWrite(arahLong, HIGH);
        unsPwmX=pwmX;
      }
      analogWrite(pwmLong, unsPwmX);      
    break;    
  
    case 5: //Manual Transversal
      pwmX=0;
      analogWrite(pwmLong, 0);
      if (antiSway=="1"){
        kontrolDegY();
        pwmY=pwmManualY+fuzzyOutputAntiSwayY;
        if(pwmY<40 && pwmY>-40){
          pwmY=0;
        } else if (pwmY>255){
          pwmY=255;
        } else if (pwmY<-255){
          pwmY=-255;
        }
      } else if (antiSway=="0"){
        pwmY=pwmManualY;
      }      

      cekLimitSwitch();
  
      if (pwmY<0){
        digitalWrite(arahTrans, LOW);
        unsPwmY=pwmY*(-1);
      } else {
        digitalWrite(arahTrans, HIGH);
        unsPwmY=pwmY;
      } 
      analogWrite(pwmTrans, unsPwmY);
    break;
    }
    stateKontrol=0;
  }
  if(millis() - timerReport > 1000){
    kirimDataPC();
    Serial3Monitor();
    timerReport = millis();}

//Serial1.print("end: ");Serial1.println(millis());
}


//-----FUNGSI ROTARY ENCODER-------------------------------------------------------------------------------------------------------------------------------------

void NGTLongA(){
  if(digitalRead(encLongB)==0){
    posisiLong--;
  }
  else {
    posisiLong++;
  }
}

void NGTTransA(){
  if(digitalRead(encTransB)==0){
    posisiTrans++;
  }
  else {
    posisiTrans--;
  }
}

//-----FUNGSI LIMIT SWITCH & EMERGENCY BUTTON-------------------------------------------------------------------------------------------------------------------------------------

void cekLimitSwitch(){
  if(digitalRead(limitLongMin)==LOW){
    posisiLong=posisiLongMin;
    if(pwmX<0){
      pwmX=0;
    }
  } else if(digitalRead(limitLongMax)==LOW){
    posisiLong=posisiLongMax;
    if(pwmX>0){
      pwmX=0;
    }
  }  

  if(digitalRead(limitTransMin)==LOW){
    posisiTrans=posisiTransMin;
    if(pwmY<0){
      pwmY=0;
    }
  } else if(digitalRead(limitTransMax)==LOW){
    posisiTrans=posisiTransMax;
    if(pwmY>0){
      pwmY=0;
    }
  }
}

void cekEmergency(){
  if(digitalRead(pbEmergency)==1){  //active low nc
    emergency= 1;
    state= 1; 
  } else{
    emergency=0;
  }
}

//-----FUNGSI FUZZY POSISI LONGITUDINAL-------------------------------------------------------------------------------------------------------------------------------------

float mapFloat(float x, float fromLow, float fromHigh, float toLow, float toHigh){
  return (x-fromLow)*(toHigh-toLow)/(fromHigh-fromLow)+toLow;
}

void kontrolPosisiX(){
  outX=posisiLong*0.61;
  errorX=inX-outX;
  dErrorX=errorX-lastErrorX;
  lastErrorX=errorX;
  fuzzyficationX();
  ruleBaseX();
  defuzzyficationX();  
}

void fuzzyficationX(){
//errorX
  if(errorX>22){
    PB_errorX=1;
    PS_errorX=0;
    Z_errorX=0;
    NS_errorX=0;    
    NB_errorX=0;  
  } else if(errorX>11){
    PB_errorX=mapFloat(errorX, 11, 22, 0, 1);
    PS_errorX=1-PB_errorX;
    Z_errorX=0;
    NS_errorX=0;
    NB_errorX=0;
  } else if(errorX>0){
    PB_errorX=0;
    PS_errorX=mapFloat(errorX, 0, 11, 0, 1);
    Z_errorX=1-PS_errorX;
    NS_errorX=0;
    NB_errorX=0;
  } else if(errorX>-11){
    PB_errorX=0;
    PS_errorX=0;
    Z_errorX=mapFloat(errorX, -11, 0, 0, 1);
    NS_errorX=1-Z_errorX;
    NB_errorX=0;    
  } else if(errorX>-22){
    PB_errorX=0;
    PS_errorX=0;
    Z_errorX=0;
    NS_errorX=mapFloat(errorX, -22, -11, 0, 1);
    NB_errorX=1-NS_errorX;
  } else {
    PB_errorX=0;
    PS_errorX=0;
    Z_errorX=0;
    NS_errorX=0;
    NB_errorX=1;
  }

//dErrorX
  if(dErrorX>1){
    PB_dErrorX=1;    
    PS_dErrorX=0;
    Z_dErrorX=0;
    NS_dErrorX=0;
    NB_dErrorX=0;
  } else if(dErrorX>0.5){
    PB_dErrorX=mapFloat(dErrorX, 0.5, 1, 0, 1);
    PS_dErrorX=1-PB_dErrorX;
    Z_dErrorX=0;
    NS_dErrorX=0;
    NB_dErrorX=0;
  } else if(dErrorX>0){
    PB_dErrorX=0;
    PS_dErrorX=mapFloat(dErrorX, 0, 0.5, 0, 1);
    Z_dErrorX=1-PS_dErrorX;
    NS_dErrorX=0;
    NB_dErrorX=0;
  } else if(dErrorX>-0.5){
    PB_dErrorX=0;
    PS_dErrorX=0;
    Z_dErrorX=mapFloat(dErrorX, -0.5, 0, 0, 1);
    NS_dErrorX=1-Z_dErrorX;
    NB_dErrorX=0;    
  } else if(dErrorX>-1){
    PB_dErrorX=0;
    PS_dErrorX=0;
    Z_dErrorX=0;
    NS_dErrorX=mapFloat(dErrorX, -1, -0.5, 0, 1);
    NB_dErrorX=1-NS_dErrorX;
  } else {
    PB_dErrorX=0;
    PS_dErrorX=0;
    Z_dErrorX=0;
    NS_dErrorX=0;
    NB_dErrorX=1;
  }
}

void ruleBaseX(){
  const int NB_PwmX=-255, NM_PwmX=-127, NS_PwmX=-63, Z_PwmX=0, PS_PwmX=63, PM_PwmX=127, PB_PwmX=255; //output

  wx1=min(NB_errorX, NB_dErrorX);
  zx1=NB_PwmX;
  wx2=min(NB_errorX, NS_dErrorX);
  zx2=NB_PwmX;
  wx3=min(NB_errorX, Z_dErrorX);
  zx3=NB_PwmX;
  wx4=min(NB_errorX, PS_dErrorX);
  zx4=NB_PwmX;
  wx5=min(NB_errorX, PB_dErrorX);
  zx5=NB_PwmX;

  wx6=min(NS_errorX, NB_dErrorX);
  zx6=NB_PwmX;
  wx7=min(NS_errorX, NS_dErrorX);
  zx7=NB_PwmX;
  wx8=min(NS_errorX, Z_dErrorX);
  zx8=NB_PwmX;
  wx9=min(NS_errorX, PS_dErrorX);
  zx9=NB_PwmX;
  wx10=min(NS_errorX, PB_dErrorX);
  zx10=NM_PwmX;

  wx11=min(Z_errorX, NB_dErrorX);
  zx11=Z_PwmX;
  wx12=min(Z_errorX, NS_dErrorX);
  zx12=PS_PwmX;
  wx13=min(Z_errorX, Z_dErrorX);
  zx13=Z_PwmX;
  wx14=min(Z_errorX, PS_dErrorX);
  zx14=NS_PwmX;
  wx15=min(Z_errorX, PB_dErrorX);
  zx15=Z_PwmX;

  wx16=min(PS_errorX, NB_dErrorX);
  zx16=PM_PwmX;
  wx17=min(PS_errorX, NS_dErrorX);
  zx17=PB_PwmX;
  wx18=min(PS_errorX, Z_dErrorX);
  zx18=PB_PwmX;
  wx19=min(PS_errorX, PS_dErrorX);
  zx19=PB_PwmX;
  wx20=min(PS_errorX, PB_dErrorX);
  zx20=PB_PwmX;

  wx21=min(PB_errorX, NB_dErrorX);
  zx21=PB_PwmX;
  wx22=min(PB_errorX, NS_dErrorX);
  zx22=PB_PwmX;
  wx23=min(PB_errorX, Z_dErrorX);
  zx23=PB_PwmX;
  wx24=min(PB_errorX, PS_dErrorX);
  zx24=PB_PwmX;
  wx25=min(PB_errorX, PB_dErrorX);
  zx25=PB_PwmX;
}

void defuzzyficationX(){
  fuzzyOutputPositionX=(wx1*zx1+wx2*zx2+wx3*zx3+wx4*zx4+wx5*zx5+wx6*zx6+wx7*zx7+wx8*zx8+wx9*zx9+wx10*zx10+wx11*zx11+wx12*zx12+wx13*zx13+wx14*zx14+wx15*zx15+wx16*zx16+wx17*zx17+wx18*zx18+wx19*zx19+wx20*zx20+wx21*zx21+wx22*zx22+wx23*zx23+wx24*zx24+wx25*zx25)/(wx1+wx2+wx3+wx4+wx5+wx6+wx7+wx8+wx9+wx10+wx11+wx12+wx13+wx14+wx15+wx16+wx17+wx18+wx19+wx20+wx21+wx22+wx23+wx24+wx25);  
}

//-----FUNGSI FUZZY POSISI TRANSVERSAL-------------------------------------------------------------------------------------------------------------------------------------

void kontrolPosisiY(){
  outY=posisiTrans*1.23;
  errorY=inY-outY;
  dErrorY=errorY-lastErrorY;
  lastErrorY=errorY;
  fuzzyficationY();
  ruleBaseY();
  defuzzyficationY();  
}

void fuzzyficationY(){
//errorY
  if(errorY>14){
    PB_errorY=1;
    PS_errorY=0;
    Z_errorY=0;
    NS_errorY=0;    
    NB_errorY=0;  
  } else if(errorY>7){
    PB_errorY=mapFloat(errorY, 7, 14, 0, 1);
    PS_errorY=1-PB_errorY;
    Z_errorY=0;
    NS_errorY=0;
    NB_errorY=0;
  } else if(errorY>0){
    PB_errorY=0;
    PS_errorY=mapFloat(errorY, 0, 7, 0, 1);
    Z_errorY=1-PS_errorY;
    NS_errorY=0;
    NB_errorY=0;
  } else if(errorY>-7){
    PB_errorY=0;
    PS_errorY=0;
    Z_errorY=mapFloat(errorY, -7, 0, 0, 1);
    NS_errorY=1-Z_errorY;
    NB_errorY=0;    
  } else if(errorY>-14){
    PB_errorY=0;
    PS_errorY=0;
    Z_errorY=0;
    NS_errorY=mapFloat(errorY, -14, -7, 0, 1);
    NB_errorY=1-NS_errorY;
  } else {
    PB_errorY=0;
    PS_errorY=0;
    Z_errorY=0;
    NS_errorY=0;
    NB_errorY=1;
  }

//dErrorY
  if(dErrorY>1){
    PB_dErrorY=1;    
    PS_dErrorY=0;
    Z_dErrorY=0;
    NS_dErrorY=0;
    NB_dErrorY=0;
  } else if(dErrorY>0.5){
    PB_dErrorY=mapFloat(dErrorY, 0.5, 1, 0, 1);
    PS_dErrorY=1-PB_dErrorY;
    Z_dErrorY=0;
    NS_dErrorY=0;
    NB_dErrorY=0;
  } else if(dErrorY>0){
    PB_dErrorY=0;
    PS_dErrorY=mapFloat(dErrorY, 0, 0.5, 0, 1);
    Z_dErrorY=1-PS_dErrorY;
    NS_dErrorY=0;
    NB_dErrorY=0;
  } else if(dErrorY>-0.5){
    PB_dErrorY=0;
    PS_dErrorY=0;
    Z_dErrorY=mapFloat(dErrorY, -0.5, 0, 0, 1);
    NS_dErrorY=1-Z_dErrorY;
    NB_dErrorY=0;    
  } else if(dErrorY>-1){
    PB_dErrorY=0;
    PS_dErrorY=0;
    Z_dErrorY=0;
    NS_dErrorY=mapFloat(dErrorY, -1, -0.5, 0, 1);
    NB_dErrorY=1-NS_dErrorY;
  } else {
    PB_dErrorY=0;
    PS_dErrorY=0;
    Z_dErrorY=0;
    NS_dErrorY=0;
    NB_dErrorY=1;
  }
}

void ruleBaseY(){
  const int NB_PwmY=-255, NM_PwmY=-127, NS_PwmY=-63, Z_PwmY=0, PS_PwmY=63, PM_PwmY=127, PB_PwmY=255; //output

  wy1=min(NB_errorY, NB_dErrorY);
  zy1=NB_PwmY;
  wy2=min(NB_errorY, NS_dErrorY);
  zy2=NB_PwmY;
  wy3=min(NB_errorY, Z_dErrorY);
  zy3=NB_PwmY;
  wy4=min(NB_errorY, PS_dErrorY);
  zy4=NB_PwmY;
  wy5=min(NB_errorY, PB_dErrorY);
  zy5=NB_PwmY;

  wy6=min(NS_errorY, NB_dErrorY);
  zy6=NB_PwmY;
  wy7=min(NS_errorY, NS_dErrorY);
  zy7=NB_PwmY;
  wy8=min(NS_errorY, Z_dErrorY);
  zy8=NB_PwmY;
  wy9=min(NS_errorY, PS_dErrorY);
  zy9=NB_PwmY;
  wy10=min(NS_errorY, PB_dErrorY);
  zy10=NM_PwmY;

  wy11=min(Z_errorY, NB_dErrorY);
  zy11=Z_PwmY;
  wy12=min(Z_errorY, NS_dErrorY);
  zy12=PS_PwmY;
  wy13=min(Z_errorY, Z_dErrorY);
  zy13=Z_PwmY;
  wy14=min(Z_errorY, PS_dErrorY);
  zy14=NS_PwmY;
  wy15=min(Z_errorY, PB_dErrorY);
  zy15=Z_PwmY;

  wy16=min(PS_errorY, NB_dErrorY);
  zy16=PM_PwmY;
  wy17=min(PS_errorY, NS_dErrorY);
  zy17=PB_PwmY;
  wy18=min(PS_errorY, Z_dErrorY);
  zy18=PB_PwmY;
  wy19=min(PS_errorY, PS_dErrorY);
  zy19=PB_PwmY;
  wy20=min(PS_errorY, PB_dErrorY);
  zy20=PB_PwmY;

  wy21=min(PB_errorY, NB_dErrorY);
  zy21=PB_PwmY;
  wy22=min(PB_errorY, NS_dErrorY);
  zy22=PB_PwmY;
  wy23=min(PB_errorY, Z_dErrorY);
  zy23=PB_PwmY;
  wy24=min(PB_errorY, PS_dErrorY);
  zy24=PB_PwmY;
  wy25=min(PB_errorY, PB_dErrorY);
  zy25=PB_PwmY;
}

void defuzzyficationY(){
  fuzzyOutputPositionY=(wy1*zy1+wy2*zy2+wy3*zy3+wy4*zy4+wy5*zy5+wy6*zy6+wy7*zy7+wy8*zy8+wy9*zy9+wy10*zy10+wy11*zy11+wy12*zy12+wy13*zy13+wy14*zy14+wy15*zy15+wy16*zy16+wy17*zy17+wy18*zy18+wy19*zy19+wy20*zy20+wy21*zy21+wy22*zy22+wy23*zy23+wy24*zy24+wy25*zy25)/(wy1+wy2+wy3+wy4+wy5+wy6+wy7+wy8+wy9+wy10+wy11+wy12+wy13+wy14+wy15+wy16+wy17+wy18+wy19+wy20+wy21+wy22+wy23+wy24+wy25);   
}

//-----FUNGSI FUZZY ANTI-SWAY LONGITUDINAL-------------------------------------------------------------------------------------------------------------------------------------

void kontrolDegX(){
  degX = degXOri - offsetDegX;
  errorDegX = 0.0 - degX;

  dErrorDegX = errorDegX - lastEDegX;
  lastEDegX = errorDegX;

  fuzzyficationDegX();
  ruleBaseDegX();
  defuzzyficationDegX();  
}

void fuzzyficationDegX(){
//errorDegX
  if(errorDegX>4){
    PB_errorDegX=1;
    PS_errorDegX=0;
    Z_errorDegX=0;
    NS_errorDegX=0;    
    NB_errorDegX=0;  
  } else if(errorDegX>1){
    PB_errorDegX=mapFloat(errorDegX, 1, 4, 0, 1);
    PS_errorDegX=1-PB_errorDegX;
    Z_errorDegX=0;
    NS_errorDegX=0;
    NB_errorDegX=0;
  } else if(errorDegX>0){
    PB_errorDegX=0;
    PS_errorDegX=mapFloat(errorDegX, 0, 1, 0, 1);
    Z_errorDegX=1-PS_errorDegX;
    NS_errorDegX=0;
    NB_errorDegX=0;
  } else if(errorDegX>-1){
    PB_errorDegX=0;
    PS_errorDegX=0;
    Z_errorDegX=mapFloat(errorDegX, -1, 0, 0, 1);
    NS_errorDegX=1-Z_errorDegX;
    NB_errorDegX=0;    
  } else if(errorDegX>-4){
    PB_errorDegX=0;
    PS_errorDegX=0;
    Z_errorDegX=0;
    NS_errorDegX=mapFloat(errorDegX, -4, -1, 0, 1);
    NB_errorDegX=1-NS_errorDegX;
  } else {
    PB_errorDegX=0;
    PS_errorDegX=0;
    Z_errorDegX=0;
    NS_errorDegX=0;
    NB_errorDegX=1;
  }

//dErrorDegX
  if(dErrorDegX>2){
    PB_dErrorDegX=1;    
    PS_dErrorDegX=0;
    Z_dErrorDegX=0;
    NS_dErrorDegX=0;
    NB_dErrorDegX=0;
  } else if(dErrorDegX>0.5){
    PB_dErrorDegX=mapFloat(dErrorDegX, 0.5, 2, 0, 1);
    PS_dErrorDegX=1-PB_dErrorDegX;
    Z_dErrorDegX=0;
    NS_dErrorDegX=0;
    NB_dErrorDegX=0;
  } else if(dErrorDegX>0){
    PB_dErrorDegX=0;
    PS_dErrorDegX=mapFloat(dErrorDegX, 0, 0.5, 0, 1);
    Z_dErrorDegX=1-PS_dErrorDegX;
    NS_dErrorDegX=0;
    NB_dErrorDegX=0;
  } else if(dErrorDegX>-0.5){
    PB_dErrorDegX=0;
    PS_dErrorDegX=0;
    Z_dErrorDegX=mapFloat(dErrorDegX, -0.5, 0, 0, 1);
    NS_dErrorDegX=1-Z_dErrorDegX;
    NB_dErrorDegX=0;    
  } else if(dErrorDegX>-2){
    PB_dErrorDegX=0;
    PS_dErrorDegX=0;
    Z_dErrorDegX=0;
    NS_dErrorDegX=mapFloat(dErrorDegX, -2, -0.5, 0, 1);
    NB_dErrorDegX=1-NS_dErrorDegX;
  } else {
    PB_dErrorDegX=0;
    PS_dErrorDegX=0;
    Z_dErrorDegX=0;
    NS_dErrorDegX=0;
    NB_dErrorDegX=1;
  }
}

void ruleBaseDegX(){
  const int NB_PwmDegX=-255, NM_PwmDegX=-170, NS_PwmDegX=-60, Z_PwmDegX=0, PS_PwmDegX=60, PM_PwmDegX=170, PB_PwmDegX=255; //output

  wDegX1=min(NB_errorDegX, NB_dErrorDegX);
  zDegX1=PB_PwmDegX;
  wDegX2=min(NB_errorDegX, NS_dErrorDegX);
  zDegX2=PB_PwmDegX;
  wDegX3=min(NB_errorDegX, Z_dErrorDegX);
  zDegX3=PM_PwmDegX;
  wDegX4=min(NB_errorDegX, PS_dErrorDegX);
  zDegX4=PS_PwmDegX;
  wDegX5=min(NB_errorDegX, PB_dErrorDegX);
  zDegX5=Z_PwmDegX;

  wDegX6=min(NS_errorDegX, NB_dErrorDegX);
  zDegX6=PB_PwmDegX;
  wDegX7=min(NS_errorDegX, NS_dErrorDegX);
  zDegX7=PM_PwmDegX;
  wDegX8=min(NS_errorDegX, Z_dErrorDegX);
  zDegX8=PS_PwmDegX;
  wDegX9=min(NS_errorDegX, PS_dErrorDegX);
  zDegX9=PS_PwmDegX;
  wDegX10=min(NS_errorDegX, PB_dErrorDegX);
  zDegX10=Z_PwmDegX;

  wDegX11=min(Z_errorDegX, NB_dErrorDegX);
  zDegX11=PS_PwmDegX;
  wDegX12=min(Z_errorDegX, NS_dErrorDegX);
  zDegX12=PS_PwmDegX;
  wDegX13=min(Z_errorDegX, Z_dErrorDegX);
  zDegX13=Z_PwmDegX;
  wDegX14=min(Z_errorDegX, PS_dErrorDegX);
  zDegX14=NS_PwmDegX;
  wDegX15=min(Z_errorDegX, PB_dErrorDegX);
  zDegX15=NS_PwmDegX;

  wDegX16=min(PS_errorDegX, NB_dErrorDegX);
  zDegX16=Z_PwmDegX;
  wDegX17=min(PS_errorDegX, NS_dErrorDegX);
  zDegX17=NS_PwmDegX;
  wDegX18=min(PS_errorDegX, Z_dErrorDegX);
  zDegX18=NS_PwmDegX;
  wDegX19=min(PS_errorDegX, PS_dErrorDegX);
  zDegX19=NM_PwmDegX;
  wDegX20=min(PS_errorDegX, PB_dErrorDegX);
  zDegX20=NB_PwmDegX;

  wDegX21=min(PB_errorDegX, NB_dErrorDegX);
  zDegX21=Z_PwmDegX;
  wDegX22=min(PB_errorDegX, NS_dErrorDegX);
  zDegX22=NS_PwmDegX;
  wDegX23=min(PB_errorDegX, Z_dErrorDegX);
  zDegX23=NM_PwmDegX;
  wDegX24=min(PB_errorDegX, PS_dErrorDegX);
  zDegX24=NB_PwmDegX;
  wDegX25=min(PB_errorDegX, PB_dErrorDegX);
  zDegX25=NB_PwmDegX;
}

void defuzzyficationDegX(){
  fuzzyOutputAntiSwayX=(wDegX1*zDegX1+wDegX2*zDegX2+wDegX3*zDegX3+wDegX4*zDegX4+wDegX5*zDegX5+wDegX6*zDegX6+wDegX7*zDegX7+wDegX8*zDegX8+wDegX9*zDegX9+wDegX10*zDegX10+wDegX11*zDegX11+wDegX12*zDegX12+wDegX13*zDegX13+wDegX14*zDegX14+wDegX15*zDegX15+wDegX16*zDegX16+wDegX17*zDegX17+wDegX18*zDegX18+wDegX19*zDegX19+wDegX20*zDegX20+wDegX21*zDegX21+wDegX22*zDegX22+wDegX23*zDegX23+wDegX24*zDegX24+wDegX25*zDegX25)/(wDegX1+wDegX2+wDegX3+wDegX4+wDegX5+wDegX6+wDegX7+wDegX8+wDegX9+wDegX10+wDegX11+wDegX12+wDegX13+wDegX14+wDegX15+wDegX16+wDegX17+wDegX18+wDegX19+wDegX20+wDegX21+wDegX22+wDegX23+wDegX24+wDegX25);  
  fuzzyOutputAntiSwayX=fuzzyOutputAntiSwayX*(-1.0);
}

//-----FUNGSI FUZZY ANTI-SWAY TRANSVERSAL-------------------------------------------------------------------------------------------------------------------------------------

void kontrolDegY(){
  degY = degYOri - offsetDegY;
  errorDegY = 0.0 - degY;

  dErrorDegY = errorDegY - lastEDegY;
  lastEDegY = errorDegY;

  fuzzyficationDegY();
  ruleBaseDegY();
  defuzzyficationDegY();  
}

void fuzzyficationDegY(){
//errorDegY
  if(errorDegY>4){
    PB_errorDegY=1;
    PS_errorDegY=0;
    Z_errorDegY=0;
    NS_errorDegY=0;    
    NB_errorDegY=0;  
  } else if(errorDegY>1){
    PB_errorDegY=mapFloat(errorDegY, 1, 4, 0, 1);
    PS_errorDegY=1-PB_errorDegY;
    Z_errorDegY=0;
    NS_errorDegY=0;
    NB_errorDegY=0;
  } else if(errorDegY>0){
    PB_errorDegY=0;
    PS_errorDegY=mapFloat(errorDegY, 0, 1, 0, 1);
    Z_errorDegY=1-PS_errorDegY;
    NS_errorDegY=0;
    NB_errorDegY=0;
  } else if(errorDegY>-1){
    PB_errorDegY=0;
    PS_errorDegY=0;
    Z_errorDegY=mapFloat(errorDegY, -1, 0, 0, 1);
    NS_errorDegY=1-Z_errorDegY;
    NB_errorDegY=0;    
  } else if(errorDegY>-4){
    PB_errorDegY=0;
    PS_errorDegY=0;
    Z_errorDegY=0;
    NS_errorDegY=mapFloat(errorDegY, -4, -1, 0, 1);
    NB_errorDegY=1-NS_errorDegY;
  } else {
    PB_errorDegY=0;
    PS_errorDegY=0;
    Z_errorDegY=0;
    NS_errorDegY=0;
    NB_errorDegY=1;
  }

//dErrorDegY
  if(dErrorDegY>2){
    PB_dErrorDegY=1;    
    PS_dErrorDegY=0;
    Z_dErrorDegY=0;
    NS_dErrorDegY=0;
    NB_dErrorDegY=0;
  } else if(dErrorDegY>1){
    PB_dErrorDegY=mapFloat(dErrorDegY, 1, 4, 0, 1);
    PS_dErrorDegY=1-PB_dErrorDegY;
    Z_dErrorDegY=0;
    NS_dErrorDegY=0;
    NB_dErrorDegY=0;
  } else if(dErrorDegY>0){
    PB_dErrorDegY=0;
    PS_dErrorDegY=mapFloat(dErrorDegY, 0, 1, 0, 1);
    Z_dErrorDegY=1-PS_dErrorDegY;
    NS_dErrorDegY=0;
    NB_dErrorDegY=0;
  } else if(dErrorDegY>-1){
    PB_dErrorDegY=0;
    PS_dErrorDegY=0;
    Z_dErrorDegY=mapFloat(dErrorDegY, -1, 0, 0, 1);
    NS_dErrorDegY=1-Z_dErrorDegY;
    NB_dErrorDegY=0;    
  } else if(dErrorDegY>-4){
    PB_dErrorDegY=0;
    PS_dErrorDegY=0;
    Z_dErrorDegY=0;
    NS_dErrorDegY=mapFloat(dErrorDegY, -4, -1, 0, 1);
    NB_dErrorDegY=1-NS_dErrorDegY;
  } else {
    PB_dErrorDegY=0;
    PS_dErrorDegY=0;
    Z_dErrorDegY=0;
    NS_dErrorDegY=0;
    NB_dErrorDegY=1;
  }
}

void ruleBaseDegY(){
  const int NB_PwmDegY=-255, NM_PwmDegY=-175, NS_PwmDegY=-60, Z_PwmDegY=0, PS_PwmDegY=60, PM_PwmDegY=175, PB_PwmDegY=255; //output

  wDegY1=min(NB_errorDegY, NB_dErrorDegY);
  zDegY1=PB_PwmDegY;
  wDegY2=min(NB_errorDegY, NS_dErrorDegY);
  zDegY2=PM_PwmDegY;
  wDegY3=min(NB_errorDegY, Z_dErrorDegY);
  zDegY3=PM_PwmDegY;
  wDegY4=min(NB_errorDegY, PS_dErrorDegY);
  zDegY4=PS_PwmDegY;
  wDegY5=min(NB_errorDegY, PB_dErrorDegY);
  zDegY5=Z_PwmDegY;

  wDegY6=min(NS_errorDegY, NB_dErrorDegY);
  zDegY6=PM_PwmDegY;
  wDegY7=min(NS_errorDegY, NS_dErrorDegY);
  zDegY7=PM_PwmDegY;
  wDegY8=min(NS_errorDegY, Z_dErrorDegY);
  zDegY8=PS_PwmDegY;
  wDegY9=min(NS_errorDegY, PS_dErrorDegY);
  zDegY9=PS_PwmDegY;
  wDegY10=min(NS_errorDegY, PB_dErrorDegY);
  zDegY10=Z_PwmDegY;

  wDegY11=min(Z_errorDegY, NB_dErrorDegY);
  zDegY11=PS_PwmDegY;
  wDegY12=min(Z_errorDegY, NS_dErrorDegY);
  zDegY12=PS_PwmDegY;
  wDegY13=min(Z_errorDegY, Z_dErrorDegY);
  zDegY13=Z_PwmDegY;
  wDegY14=min(Z_errorDegY, PS_dErrorDegY);
  zDegY14=NS_PwmDegY;
  wDegY15=min(Z_errorDegY, PB_dErrorDegY);
  zDegY15=NS_PwmDegY;

  wDegY16=min(PS_errorDegY, NB_dErrorDegY);
  zDegY16=Z_PwmDegY;
  wDegY17=min(PS_errorDegY, NS_dErrorDegY);
  zDegY17=NS_PwmDegY;
  wDegY18=min(PS_errorDegY, Z_dErrorDegY);
  zDegY18=NS_PwmDegY;
  wDegY19=min(PS_errorDegY, PS_dErrorDegY);
  zDegY19=NM_PwmDegY;
  wDegY20=min(PS_errorDegY, PB_dErrorDegY);
  zDegY20=NM_PwmDegY;

  wDegY21=min(PB_errorDegY, NB_dErrorDegY);
  zDegY21=Z_PwmDegY;
  wDegY22=min(PB_errorDegY, NS_dErrorDegY);
  zDegY22=NS_PwmDegY;
  wDegY23=min(PB_errorDegY, Z_dErrorDegY);
  zDegY23=NM_PwmDegY;
  wDegY24=min(PB_errorDegY, PS_dErrorDegY);
  zDegY24=NM_PwmDegY;
  wDegY25=min(PB_errorDegY, PB_dErrorDegY);
  zDegY25=NB_PwmDegY;
}

void defuzzyficationDegY(){
  fuzzyOutputAntiSwayY=(wDegY1*zDegY1+wDegY2*zDegY2+wDegY3*zDegY3+wDegY4*zDegY4+wDegY5*zDegY5+wDegY6*zDegY6+wDegY7*zDegY7+wDegY8*zDegY8+wDegY9*zDegY9+wDegY10*zDegY10+wDegY11*zDegY11+wDegY12*zDegY12+wDegY13*zDegY13+wDegY14*zDegY14+wDegY15*zDegY15+wDegY16*zDegY16+wDegY17*zDegY17+wDegY18*zDegY18+wDegY19*zDegY19+wDegY20*zDegY20+wDegY21*zDegY21+wDegY22*zDegY22+wDegY23*zDegY23+wDegY24*zDegY24+wDegY25*zDegY25)/(wDegY1+wDegY2+wDegY3+wDegY4+wDegY5+wDegY6+wDegY7+wDegY8+wDegY9+wDegY10+wDegY11+wDegY12+wDegY13+wDegY14+wDegY15+wDegY16+wDegY17+wDegY18+wDegY19+wDegY20+wDegY21+wDegY22+wDegY23+wDegY24+wDegY25);  
//  fuzzyOutputAntiSwayY=fuzzyOutputAntiSwayY * -1.0;
}

//-----FUNGSI KOMUNIKASI HMI, BLUETOOTH MIKRO SLAVE, USB TTL-------------------------------------------------------------------------------------------------------------------------------------

void terimaDataPC(){ 
  dataIn="";
  while (Serial.available()) {
    char dataReceived = (char)Serial.read();
    if (dataReceived=='#'||dataReceived=='*'){
      dataIn="";}
    dataIn += dataReceived;
    Serial3.print(dataReceived);}
  if (dataIn=="#,1"){
    state=0;
    dataIn="";
  } else if (dataIn=="#,2"){
    offsetDegX = degXOri;
    offsetDegY = degYOri;
    Serial3.print("Offset DegX = ");
    Serial3.print(offsetDegX);
    Serial3.print("Offset DegY = ");
    Serial3.print(offsetDegY);    
  }
}

void kirimDataPC(){
  int degXKirim, degYKirim, outXKirim, outYKirim;

  degX = degXOri - offsetDegX;
  degY = degYOri - offsetDegY;
  outX=posisiLong*0.61;
  outY=posisiTrans*1.23;    
  outXKirim= round(outX);
  outYKirim= round(outY);
  degXKirim= degX*100;
  degYKirim= degY*100;
  dataOut="#,"+String(outXKirim)+','+ String(outYKirim)+','+ String(degXKirim)+','+ String(degYKirim)+','+ String(magnet)+", ";
  Serial.println(dataOut);
  Serial.flush();
}

void terimaDataBluetooth(){
  while (Serial1.available()) {
  char dataReceived = (char)Serial1.read();
  if (dataReceived=='#'){
    dataInBT="";}
  else if (dataReceived=='*'){
    stateBaca=1;}
  dataInBT += dataReceived;
  }
    
//    Parsing dataInBT
  if (stateBaca==1){
    protokol=dataInBT.charAt(0);
    if (protokol=='#'){      
      index1=dataInBT.indexOf(',');
      index2=dataInBT.indexOf(',', index1+1);
      degYBT=dataInBT.substring(index1+1, index2);
      index3=dataInBT.indexOf(',', index2+1);
      degXBT=dataInBT.substring(index2+1, index3);
 
      degXOri = degXBT.toInt() / 100.0;
      degYOri = degYBT.toInt() / 100.0;
      dataInBT="";
      
      stateKontrol=1;    
    }
    stateBaca=0;
  }
}

void Serial3Monitor(){
  Serial3.print("Koordinat X : ");Serial1.println(outX);
  Serial3.print("Koordinat Y : ");Serial1.println(outY);
  Serial3.print("AngleX : ");Serial1.println(degX);
  Serial3.print("AngleY : ");Serial1.println(degY);
  Serial3.print("Elektromagnet : ");
  if(magnet==1){
    Serial3.println("ON");
  } else{Serial3.println("OFF");}
  Serial3.print("Tombol Emergency : ");
  if(digitalRead(pbEmergency)==0){
    Serial3.println("ON");
  } else{Serial3.println("OFF");}
  Serial3.println("-------------------------------------------");
}
