/**
 *Programa para controlar el acercamiento a un objeto.
 */

/** Se incluye el header */
#include "onfacedetection.h"

/** Librerias para las variables del Nao */
#include <alvalue/alvalue.h>
/** Librerias de proxys */
#include <alcommon/alproxy.h>
/** Broker */
#include <alcommon/albroker.h>
/** Libreria para usar el mutex en una seccion critica */
#include <althread/alcriticalsection.h>

/** Logs */
#include <qi/log.hpp>

/** Constructor, en el se inicializan las variables */
OnFaceDetection::OnFaceDetection(
  boost::shared_ptr<AL::ALBroker> broker,
  const std::string& name):
    AL::ALModule(broker, name),
    fMemoryProxy(getParentBroker()),
    fFaces(AL::ALValue()),
    fFacesCount(0),
    Dref(0.5),
    Dest(0),
    Xtk1(4),
    Vxtk1(0),
    Xtk(4),
    Vxtk(0),
    Kp(1),
    Ki(0.2),
    Eant(0),
    Eact(0),
    Etk1(0),
    M(0),
    Q1(0.000000001),
    Q2(0),
    Q3(0),
    Q4(0.000000001),
    R(0.0001),
    F1(1),
    F2(0),
    F3(0),
    F4(1),
    H1(0),
    H2(0),
    P1(0),
    P2(0),
    P3(0),
    P4(0),
    G1(0),
    G2(0),
    divAux(0),
    ballXAnt(1),
    ballXAct(1),
    ballYAnt(1),
    ballYAct(1),
    innov(0),
    DeltaT(0),
    fCallbackMutex(AL::ALMutex::createALMutex())
{
  /** Descripcion del modulo */
  setModuleDescription("Modulo para moverse hacia un objeto (cara) al activarse un evento");
  /** Funcion que se llama al activarse el evento */
  functionName("callback", getName(), "");
  /** Se liga la funcion al modulo */
  BIND_METHOD(OnFaceDetection::callback);
}

OnFaceDetection::~OnFaceDetection() {}

/** Inicializar el modulo */
void OnFaceDetection::init() {
  try {
    aware.stopAwareness();
    /** Se activan los motores y se ponen rigidas las uniones de todo el cuerpo */
    AL::ALValue stiffness = 1.0f;
    AL::ALValue stime = 1.0f;
    AL::ALValue jointName = "Body";
    motion.stiffnessInterpolation(jointName, stiffness, stime);
    /** Las manos se mueven al caminar */
    motion.setWalkArmsEnabled(true, true);
    /** Si el robot golpea un objeto con el pie se detiene */
    AL::ALValue confMot = AL::ALValue::array("ENABLE_FOOT_CONTACT_PROTECTION", true);
    motion.setMotionConfig(confMot);
    /** El Nao toma la postura para iniciar a caminar */
    posture.goToPosture("StandInit", 0.5);
    /** Se obtiene el tiempo actual */
    gettimeofday(&start, NULL);
    Tk=((double)start.tv_sec+(double)start.tv_usec*.000001);
    /** Se revisa si se detecto alguna cara al inicializarse */
    fFaces = fMemoryProxy.getData("FaceDetected");
    /** Si no hay ninguna el robot dice "No face detected" */
    if (fFaces.getSize() < 2) {
      qiLogInfo("module.example") << "No face detected" << std::endl;
      fTtsProxy.say("No face detected");
    }
    /** Suscripcion al evento, cuando reconoce una cara llama la funcion callback */
    fMemoryProxy.subscribeToEvent("FaceDetected", "OnFaceDetection", "callback");
  }
  catch (const AL::ALError& e) {
    qiLogError("module.name") << e.what() << std::endl;
  }
}

/** Funcion al reconocer caras */
void OnFaceDetection::callback() {
  /** Uso de un mutex para proteger la funcion de cambios en la memoria */
  AL::ALCriticalSection section(fCallbackMutex);
  try {
    /** Obtener los datos obtenidos por el evento */
    fFaces = fMemoryProxy.getData("FaceDetected");
    
    
    /** En caso de no haber una cara detectada el robot se detiene, reinicia las variables y dice "No face detected" */
    if (fFaces.getSize() < 2 ) {
      if (fFacesCount != 0) {
        qiLogInfo("module.example") << "No face detected" << std::endl;
        fTtsProxy.say("No face detected.");
        fFacesCount = 0;
        /** Detenerse */
        AL::ALValue X = 0.0;
        AL::ALValue Y = 0.0;
        AL::ALValue Theta = 0.0;
        AL::ALValue Frequency = 0.0;
        motion.setWalkTargetVelocity(X, Y, Theta, Frequency);
        /** Reiniciar variables */
        Vxtk=0;
        M=0;
        P1=0;
        P2=0;
        P3=0;
        P4=0;
        Eact=0;
        Etk1=0;
        sleep(1);
      }
      return;
    }
    
    
    /** Se verifica si hay el mismo numero de caras y si cambia el Nao dice cuantas hay */
    if (fFaces[1].getSize() - 1 != fFacesCount) {
      qiLogInfo("module.name") << fFaces[1].getSize() - 1 << " face(s) detected." << std::endl;
      char buffer[50];
      sprintf(buffer, "%d faces detected.", fFaces[1].getSize() - 1);
      fTtsProxy.say(std::string(buffer));
      /** Se actualiza el contador con el numero de caras */
      fFacesCount = fFaces[1].getSize() - 1;
      /** Se obtiene el tiempo actual */
      gettimeofday(&start, NULL);
      Tk=((double)start.tv_sec+(double)start.tv_usec*.000001);
    }
    
    
    /** Al localizar una cara el Nao ejecuta el siguiente codigo para aproximarse al objeto hasta alcanzarlo */
    if(fFaces[1].getSize() - 1 == 1 && Xtk>=0.2){
      /** Se obtiene el arreglo con las caras detectadas */
      AL::ALValue faceData = fFaces[1];
      /** Se obtiene la informacion de la primera cara */
      AL::ALValue faceData2 = faceData[0];
      /** Se obtiene la informacion de la forma de la cara */
      AL::ALValue faceData3 = faceData2[0];
      /** Se obtiene la informacion de los ejes X y Y de la cara */
      AL::ALValue faceData4 = faceData3[3];
      AL::ALValue faceData5 = faceData3[4];
      
      /** Se obtiene el tiempo actual y se guarda el anterior */
      gettimeofday(&start, NULL);
      Tk1=Tk;
      Tk=(double)start.tv_sec+(double)start.tv_usec*.000001;
      /** Se calcula el tiempo transcurrido */
      DeltaT=Tk-Tk1;
      
      /** Se guardan los valores anteriores de X y Y del objeto */
      ballXAnt=ballXAct;
      ballYAnt=ballYAct;
      
      /** Variables para el calculo del flujo divergente */
      ballXAct=(double)faceData4;
      ballYAct=(double)faceData5;
      
      /** Estimacion del flujo divergente con las observaciones */
      Dest=((((ballXAct-ballXAnt)/ballXAnt)+((ballYAct-ballYAnt)/ballYAnt))/(2*DeltaT));
      
      /** Se guarda el error anterior */
      Eant=Eact;
      /** Se calcula el error actual */
      Eact=(Dref-(Vxtk/(Xtk)));
      /** Se calcula el error acumulado */
      Etk1=(Etk1+(((Eact+Eant)*DeltaT)/2));
      
      /** Calculo del PI */
      M=((Kp*Eact)+Ki*Etk1);
      
      /** En esta seccion se limita la velocidad del Nao */
      if(Vxtk>=1 && M>=0){
        Vxtk=1;
        M=0;
      }
      
      
      /** Se envia la velocidad al sistema */
      AL::ALValue X = Vxtk;
      AL::ALValue Y = 0.0;
      AL::ALValue Theta = 0.0;
      AL::ALValue Frequency = 0.5;
      motion.setWalkTargetVelocity(X, Y, Theta, Frequency);
      
      
      /** Calculo del jacobiano del sistema */
      F2=-DeltaT;
      /** Calculo del jacobiano de la observacion */
      H1=(0-(Vxtk/(Xtk*Xtk)));
      H2=(1/Xtk);
      
      /** Se guarda el estado anterior del sistema */
      Xtk1=Xtk;
      Vxtk1=Vxtk;
      /** Se estima el estado actual */
      Xtk=(Xtk1-(DeltaT*Vxtk1)-((DeltaT*DeltaT*M)/2));
      Vxtk=(Vxtk1+(DeltaT*M));
      
      /** Se estiman las covarianzas */
      P1=((((F1*P1)+(F2*P3))*F1)+(((F1*P2)+(F2*P4))*F2)+Q1);
      P2=((((F1*P1)+(F2*P3))*F3)+(((F1*P2)+(F2*P4))*F4)+Q2);
      P3=((((F3*P1)+(F4*P3))*F1)+(((F3*P2)+(F4*P4))*F2)+Q3);
      P4=((((F3*P1)+(F4*P3))*F3)+(((F3*P2)+(F4*P4))*F4)+Q4);
      
      /** Se calcula la ganancia de Kalman */
      divAux=((((H1*P1)+(H2*P3))*H1)+(((H1*P2)+(H2*P4))*H2)+R);
      G1=(((P1*H1)+(P2*H2))/divAux);
      G2=(((P3*H1)+(P4*H2))/divAux);
      
      /** Se calcula la innovacion */
      innov=(Dest-(Vxtk/Xtk));
      /** Se actualizan las variables */
      Xtk=(Xtk+(G1*innov));
      Vxtk=(Vxtk+(G2*innov));
      
      /** Se actualizan las covarianzas */
      P1=(((1-(G1*H1))*P1)-(G1*H2*P3));
      P2=(((1-(G1*H1))*P2)-(G1*H2*P4));
      P3=(((1-(G2*H2))*P3)-(G2*H1*P1));
      P4=(((1-(G2*H2))*P4)-(G2*H1*P2));
      
      sleep(1);
    }else{
      /** Si se alcanzo el objeto */
      if(Xtk<0.2){
        /** Se reinicia la distancia */
        Xtk=4;
        /** El robot dice que alcanzo el objetivo */
        fTtsProxy.say("I have reached the target");
        /** Se espera un tiempo para volver a empezar */
        sleep(7);
      }
      /** Al haber mas de 1 cara o al llegar al objetivo el robot se detiene y se reinician las variables */
      AL::ALValue X = 0.0;
      AL::ALValue Y = 0.0;
      AL::ALValue Theta = 0.0;
      AL::ALValue Frequency = 0.0;
      /** Detenerse */
      motion.setWalkTargetVelocity(X, Y, Theta, Frequency);
      /** Reiniciar variables */
      Vxtk=0;
      M=0;
      P1=0;
      P2=0;
      P3=0;
      P4=0;
      Eact=0;
      Etk1=0;
      sleep(1);
    }
  }
  catch (const AL::ALError& e) {
    qiLogError("module.name") << e.what() << std::endl;
  }
}
