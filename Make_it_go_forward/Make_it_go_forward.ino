#include <Servo.h>

float Height = 7.5;
float L1 = 5;
float L2 = 5;

float HipEquil = 30;
float HipRange = 25;
float HipMin = HipEquil - HipRange / 2;
float HipMax = HipEquil + HipRange / 2;

int numCycles = 13;
int count = 1;
int PhaseDelay = 20;
float PhaseStep = .05;

float KneeFlightAngle = 90;

int StanceFrontR = 1;

int StanceFrontL = 1;

int StanceHindR = 1;

int StanceHindL = 1;

Servo FRHM; 
int FRHM_Pin = 7;
int FRHM_Home = 120; 

Servo FRKM; 
int FRKM_Pin = 10;
int FRKM_Home = 185;


Servo FLHM; 
int FLHM_Pin = 2;
int FLHM_Home = 100;

Servo FLKM; 
int FLKM_Pin = 11;
int FLKM_Home = 133;


Servo HindLHM;
int HindLHM_Pin = 6;
int HindLHM_Home = 220;


Servo HindLKM;
int HindLKM_Pin = 4;
int HindLKM_Home = 120;

Servo HindRHM;
int HindRHM_Pin = 9;
int HindRHM_Home = 80;


Servo HindRKM;
int HindRKM_Pin = 12;
int HindRKM_Home = 126;



float Angle_FRHM = HipEquil * (PI / 180);
float Angle_FRKM = Angle_FRHM + acos((Height - L1*cos(Angle_FRHM)) / L2);


float Angle_HindLHM = HipEquil * (PI / 180);
float Angle_HindLKM = Angle_HindLHM + acos((Height - L1*cos(Angle_HindLHM)) / L2);

float Angle_FLHM = HipEquil * (PI / 180);
float Angle_FLKM = Angle_FLHM + acos((Height - L1*cos(Angle_FLHM)) / L2);


float Angle_HindRHM = HipEquil * (PI / 180);
float Angle_HindRKM = Angle_FRHM + acos((Height - L1*cos(Angle_FRHM)) / L2);



float Angle_FRKM_OLD = Angle_FRKM;

float Angle_HindLKM_OLD = Angle_HindLKM;

float Angle_FLKM_OLD = Angle_FLKM;

float Angle_HindRKM_OLD = Angle_HindRKM;


float Phase = 0;


void setup() {

  Serial.begin(115200);
  
  FRHM.attach (FRHM_Pin); 
  FRKM.attach (FRKM_Pin); 

  HindLHM.attach (HindLHM_Pin); 
  HindLKM.attach (HindLKM_Pin); 

  FLHM.attach (FLHM_Pin); 
  FLKM.attach (FLKM_Pin); 

  HindRHM.attach (HindRHM_Pin);  
  HindRKM.attach (HindRKM_Pin); 



  Angle_FRHM = Angle_FRHM * (180 / PI);
  Angle_FRKM = Angle_FRKM * (180 / PI);

  Angle_HindLHM = Angle_HindLHM * (180 / PI);
  Angle_HindLKM = Angle_HindLKM * (180 / PI);


  Angle_FLHM = Angle_FLHM * (180 / PI);
  Angle_FLKM = Angle_FLKM * (180 / PI);

  Angle_HindRHM = Angle_HindRHM * (180 / PI);
  Angle_HindRKM = Angle_HindRKM * (180 / PI);

 Serial.println("------- ENTERING HIP EQUILIBRIUM POSITION -------");

 InitialPosition();

  Serial.println("------- COUNTDOWN TO MOTION -------");
  
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);
  

}

void loop() {
 
  if (count <= numCycles) 
  {
    Serial.println("Stance most forward position");
    StanceFrontR = 1;
    StanceHindL = 1;
    StanceFrontL = 0;
    StanceHindR = 0;
    UpdateServoPos(HipMax, HipMax, HipMax, HipMin);
    //delay(1000); 

    Serial.println("Stance most backward position");
    UpdateServoPos(HipMin, HipMin, HipMin, HipMax);
    //delay(1000);

    Serial.println("Flight most backward position - lifted knee");
    StanceFrontR = 0;
    StanceHindL = 0;
    StanceFrontL = 1;
    StanceHindR = 1;   
    UpdateServoPos(HipMin, HipMin, HipMin, HipMax);
    //delay(1000); 

    Serial.println("Flight most forward position - lifted knee");
    UpdateServoPos(HipMax, HipMax, HipMax, HipMin);
    //delay(1000); 
    
    
    count = count + 1; 
  }

  else {
    StanceFrontR = 1;
    UpdateServoPos(HipEquil, HipEquil, HipEquil, HipEquil); 
    StanceFrontL = 1;
    StanceHindR = 1;
    StanceHindL = 1;  
  
   }

  
}


void InitialPosition() {

  FRHM.write(map(FRHM_Home - Angle_FRHM,0,270,0,180));
  FRKM.write(map(FRKM_Home - Angle_FRKM,0,270,0,180));

  HindLHM.write(map(HindLHM_Home - Angle_HindLHM,0,270,0,180));
  HindLKM.write(map(HindLKM_Home + Angle_HindLKM,0,270,0,180));

  FLHM.write(map(FLHM_Home - Angle_FLHM,0,270,0,180));
  FLKM.write(map(FLKM_Home + Angle_FLKM,0,270,0,180));

  HindRHM.write(map(HindRHM_Home + Angle_HindRHM,0,270,0,180));
  HindRKM.write(map(HindRKM_Home + Angle_HindRKM,0,270,0,180));
}

void UpdateServoPos(float TargetAngFRHM, float TargetAngHindRHM, float TargetAngLHM, float TargetAngHindLHM) {

  float FRHM_StartAngle = FRHM.read();
  float FRKM_StartAngle = FRKM.read();
  FRHM_StartAngle = map(FRHM_StartAngle,0,180,0,270);
  FRKM_StartAngle = map(FRKM_StartAngle,0,180,0,270);

  float HindLHM_StartAngle = HindLHM.read();
  float HindLKM_StartAngle = HindLKM.read();
  HindLHM_StartAngle = map(HindLHM_StartAngle,0,180,0,270);
  HindLKM_StartAngle = map(HindLKM_StartAngle,0,180,0,270);

  float FLHM_StartAngle = FLHM.read();
  float FLKM_StartAngle = FLKM.read();
  FLHM_StartAngle = map(FLHM_StartAngle,0,180,0,270);
  FLKM_StartAngle = map(FLKM_StartAngle,0,180,0,270);

  float HindRHM_StartAngle = HindRHM.read();
  float HindRKM_StartAngle = HindRKM.read();
  HindRHM_StartAngle = map(HindRHM_StartAngle,0,180,0,270);
  HindRKM_StartAngle = map(HindRKM_StartAngle,0,180,0,270);
  
  FRHM_StartAngle = FRHM_Home - FRHM_StartAngle; 
  FRKM_StartAngle = FRKM_Home - FRKM_StartAngle ;  
  HindLHM_StartAngle = HindLHM_Home - HindLHM_StartAngle; 
  HindLKM_StartAngle = HindLKM_StartAngle - HindLKM_Home;  
  FLHM_StartAngle = FLHM_Home - FLHM_StartAngle; 
  FLKM_StartAngle = FLKM_StartAngle - FLKM_Home;  
  HindRHM_StartAngle = HindRHM_StartAngle- HindRHM_Home ; 
  HindRKM_StartAngle = HindRKM_StartAngle - HindRKM_Home;  


  
  for (Phase = 0; Phase <= 1; Phase = Phase + PhaseStep)
  {

    Angle_FRHM =  FRHM_StartAngle + Phase * (TargetAngFRHM - FRHM_StartAngle); 
    Angle_FRHM = Angle_FRHM * (PI / 180); 
    if (StanceFrontR == 1) { 
      Angle_FRKM_OLD = Angle_FRKM; 
      Angle_FRKM = Angle_FRHM + acos((Height - L1 * cos(Angle_FRHM)) / L2);
      Angle_FRKM = Angle_FRKM * (180 / PI); 
      if (isnan(Angle_FRKM)) { 
        Angle_FRKM = Angle_FRKM_OLD;
      }
    }
    if (StanceFrontR == 0) { 
      Angle_FRKM = FRKM_StartAngle + Phase * (KneeFlightAngle - FRKM_StartAngle);
    }
    Angle_FRHM = Angle_FRHM * (180 / PI); 

  

    Angle_FLHM =  FLHM_StartAngle + Phase * (TargetAngLHM - FLHM_StartAngle); 
    Angle_FLHM = Angle_FLHM * (PI / 180); 
    if (StanceFrontL == 1) { 
      Angle_FLKM_OLD = Angle_FLKM; 
      Angle_FLKM = Angle_FLHM + acos((Height - L1 * cos(Angle_FLHM)) / L2);
      Angle_FLKM = Angle_FLKM * (180 / PI); 
      if (isnan(Angle_FLKM)) { 
        Angle_FLKM = Angle_FLKM_OLD;
      }
    }
    if (StanceFrontL == 0) { 
      Angle_FLKM = FLKM_StartAngle + Phase * (KneeFlightAngle - FLKM_StartAngle);
    }
    Angle_FLHM = Angle_FLHM * (180 / PI); 



  
    Angle_HindLHM =  HindLHM_StartAngle + Phase * (TargetAngHindLHM - HindLHM_StartAngle); 
    Angle_HindLHM = Angle_HindLHM * (PI / 180); 
    if (StanceHindL == 1) { 
      Angle_HindLKM_OLD = Angle_HindLKM; 
      Angle_HindLKM = Angle_HindLHM + acos((Height - L1 * cos(Angle_HindLHM)) / L2);
      Angle_HindLKM = Angle_HindLKM * (180 / PI); 
      if (isnan(Angle_HindLKM)) { 
        Angle_HindLKM = Angle_HindLKM_OLD;
      }
    }
    if (StanceHindL == 0) { 
      Angle_HindLKM = HindLKM_StartAngle + Phase * (KneeFlightAngle - HindLKM_StartAngle);
    }
    Angle_HindLHM = Angle_HindLHM * (180 / PI);



 Angle_HindRHM =  HindRHM_StartAngle + Phase * (TargetAngHindRHM - HindRHM_StartAngle); 
    Angle_HindRHM = Angle_HindRHM * (PI / 180); 
    if (StanceHindR == 1) { 
      Angle_HindRKM_OLD = Angle_HindRKM;
      Angle_HindRKM = Angle_HindRHM + acos((Height - L1 * cos(Angle_HindRHM)) / L2);
      Angle_HindRKM = Angle_HindRKM * (180 / PI); 
      if (isnan(Angle_HindRKM)) { 
        Angle_HindRKM = Angle_HindRKM_OLD;
      }
    }
    if (StanceHindR == 0) {
      Angle_HindRKM = HindRKM_StartAngle + Phase * (KneeFlightAngle - HindRKM_StartAngle);
    }
    Angle_HindRHM = Angle_HindRHM * (180 / PI);

    FRHM.write(map(FRHM_Home - Angle_FRHM,0,270,0,180));
    FRKM.write(map(FRKM_Home - Angle_FRKM,0,270,0,180));
    
    HindLHM.write(map(HindLHM_Home - Angle_HindLHM,0,270,0,180));
    HindLKM.write(map(HindLKM_Home + Angle_HindLKM,0,270,0,180));
    
    FLHM.write(map(FLHM_Home - Angle_FLHM,0,270,0,180));
    FLKM.write(map(FLKM_Home + Angle_FLKM,0,270,0,180));
    
    HindRHM.write(map(HindRHM_Home + Angle_HindRHM,0,270,0,180));
    HindRKM.write(map(HindRKM_Home + Angle_HindRKM,0,270,0,180));
    
    delay(PhaseDelay);
  }
}


