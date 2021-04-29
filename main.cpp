/**
 * Main del programa
 */

/** Libreria de manejo de señales */
#include <signal.h>
/** Libreria boost */
#include <boost/shared_ptr.hpp>
/** Borker para comunicarse con los modulos */
#include <alcommon/albroker.h>
/** Libreria para crear modulos */
#include <alcommon/almodule.h>
/** Libreria de broker */
#include <alcommon/albrokermanager.h>
#include <alcommon/altoolsmain.h>

/** Se incluye el header */
#include "onfacedetection.h"

/** Se define si el modulo es remoto y si se ejecuta en windows o linux */
#ifdef ONFACEDETECTION_IS_REMOTE
# define ALCALL
#else
# ifdef _WIN32
#  define ALCALL __declspec(dllexport)
# else
#  define ALCALL
# endif
#endif

extern "C"
{
  /** crear modulo */
  ALCALL int _createModule(boost::shared_ptr<AL::ALBroker> pBroker)
  {
    /** El broker toma los valores del broker padre */
    AL::ALBrokerManager::setInstance(pBroker->fBrokerManager.lock());
    AL::ALBrokerManager::getInstance()->addBroker(pBroker);
      AL::ALModule::createModule<OnFaceDetection>( pBroker, "OnFaceDetection" );

    return 0;
  }
  
  /** Cerrar modulo */
  ALCALL int _closeModule()
  {
    return 0;
  }
}

/** Si el modulo es remoto se necesita una funcion principal (main) para el ejecutable */
#ifdef ONFACEDETECTION_IS_REMOTE
  int main(int argc, char *argv[])
  {
    /** Puntero para crear el modulo */
    TMainType sig;
    sig = &_createModule;
    /** Llamar al main */
    ALTools::mainFunction("onfacedetection", argc, argv, sig);
  }
#endif

