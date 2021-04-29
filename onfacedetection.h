/**
 * Header del programa para crear variables y definir los metodos
 */


#ifndef ONFACEDETECTION_ONFACEDETECTION_H
#define ONFACEDETECTION_ONFACEDETECTION_H


/** Librerias boost */
#include <boost/shared_ptr.hpp>
/** Se incluye la libreria para crear modulos */
#include <alcommon/almodule.h>
/** Libreria para manejar strings */
#include <string>

/** Este proxy permite acceder a la memoria y a los eventos */
#include <alproxies/almemoryproxy.h>
/** Este proxy le permite usar las funciones de hablar, se utiliza para que el robot nos de informacion de su estado */
#include <alproxies/altexttospeechproxy.h>
/** Este proxy le permite usar las funciones de moverse */
#include <alproxies/almotionproxy.h>
/** Este proxy le permite usar las funciones para pararse y sentarse */
#include <alproxies/alrobotpostureproxy.h>
/** Este proxy activa y desactiva el movimiento autonomo */
#include <alproxies/albasicawarenessproxy.h>

/** Para evitar problemas al leer la memoria se utiliza un mutex */
#include <althread/almutex.h>

/** Para utilizar el timer se incluyen las librerias time */
#include <sys/time.h>
#include <time.h>
/** Librerias para hacer calculos */
#include <math.h>


namespace AL
{
  class ALBroker;
}


class OnFaceDetection : public AL::ALModule
{
  public:
    /** Constructor */
    OnFaceDetection(boost::shared_ptr<AL::ALBroker> broker, const std::string& name);
    virtual ~OnFaceDetection();
    virtual void init();
    
    /**
    * Metodo que sera llamado cada vez que se localice una objeto o este se mueva
    */
    void callback();
    
  private:
    /** Se crean los proxys de memoria, movimiento autonomo, hablar, postura y caminar */
    AL::ALMemoryProxy fMemoryProxy;
    AL::ALTextToSpeechProxy fTtsProxy;
    AL::ALMotionProxy motion;
    AL::ALRobotPostureProxy posture;
    AL::ALBasicAwarenessProxy aware;
    
    /** Variable con la informacion de las caras encontradas */
    AL::ALValue fFaces;
    /** Variable que llevara la cuenta del numero de caras */
    unsigned int fFacesCount;
    /** Mutex */
    boost::shared_ptr<AL::ALMutex> fCallbackMutex;
    
    /** Se definen las variables que se utilizaran para el controlador y el filtro */
    /** Flujo divergente de referencia */
    double Dref;
    /** Flujo divergente estimado */
    double Dest;
    /** Posicion anterior */
    double Xtk1;
    /** Velocidad anterior */
    double Vxtk1;
    /** Posicion actual */
    double Xtk;
    /** Velocidad actual */
    double Vxtk;
    /** Constante proporcional */
    double Kp;
    /** Constante del integrador */
    double Ki;
    /** Error anterior */
    double Eant;
    /** Error actual */
    double Eact;
    /** Error acumulado */
    double Etk1;
    /** Aceleracion */
    double M;
    /** Covarianzas del ruido del proceso */
    double Q1;
    double Q2;
    double Q3;
    double Q4;
    /** Varianza del ruido de la observacion */
    double R;
    /** Jacobiano del proceso */
    double F1;
    double F2;
    double F3;
    double F4;
    /** Jacobiano de la observacion */
    double H1;
    double H2;
    /** Covarianzas de Kalman */
    double P1;
    double P2;
    double P3;
    double P4;
    /** Ganancia de Kalman */
    double G1;
    double G2;
    /** Division auxiliar */
    double divAux;
    /** X de la pelota o cara anterior */
    double ballXAnt;
    /** X de la pelota o cara actual */
    double ballXAct;
    /** Y de la pelota o cara anterior */
    double ballYAnt;
    /** Y de la pelota o cara actual */
    double ballYAct;
    /** Innovacion */
    double innov;
    /** Diferencial de tiempo */
    double DeltaT;
    /** Esta variable nos permitira obtener el diferencial de tiempo */
    struct timeval start;
    /** Tiempo actual */
    double Tk;
    /** Tiempo anterior */
    double Tk1;
};

#endif  // ONFACEDETECTION_ONFACEDETECTION_H
