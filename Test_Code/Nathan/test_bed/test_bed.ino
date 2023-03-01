
// class to create a custom PID
class PID {
  private:
    double pkP;
    double pkI;
    double pkD;
    double pPrevInput;
    double pPrevTime;
    double pDmax;
    double pSetpoint;
  public:
    // default counstructor
    PID(){
      pkP = 0;
      pkI = 0;
      pkD = 0;
      pDmax = 0;
    }
    
    // Overloaded constructor
    PID(double P, double I, double D, double Dmax) {
      pkP = P;
      pkI = I;
      pkD = D;
      pDmax = Dmax;
    }
  
    // setters:
    void setP(double P){ pkP = P; }
    void setI(double I){ pkP = I; }
    void setD(double D){ pkD = D; }
    void setDmax(double Dmax) { pDmax = Dmax; }
    void setSetpoint(double setpoint) { pSetpoint = setpoint; }
    
    // getters:
    double getP(){ return pkP; }
    double getI(){ return pkI; }
    double getD(){ return pkD; }
    double getDmax(){ return pDmax; }
    void getSetpoint() { return pSetpoint; }

    double output(double curInput, double curTime){ 
      // Evaluate P, i, and D terms:
      P = pkP*(pSetpoint-input);
      I = pkI*(currTime-pPrevTime)*curInput;
      D = pkD*(pPrevInput-curInput)/(pPrevTime-currTime);

      // Check if D is greater than the max:
      if(D > pDmax) {
        D = pDmax;
      }
      
      pPrevInput = curInput;
      pPrevTime = curTime;
      return (P + I + D);
    }
};


void setup() {
  // put your setup code here, to run once:
  PID* myPID = new PID();
  

}



void loop() {
  // put your main code here, to run repeatedly:

}
