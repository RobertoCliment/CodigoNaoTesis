#Version de CMake minima para construir el proyecto
cmake_minimum_required(VERSION 2.8)
#Nombre del proyecto
project(Velocidad2)
#Paquete para construir el proyecto
find_package(qibuild)

#Se puede elegir si se quiere el modulo remoto o local, para efectos de este proyecto se hara local para que el Nao lo ejecute independientemente
option(ONFACEDETECTION_IS_REMOTE
  "module is compiled as a remote module (ON or OFF)"
  OFF)

#Se definen los archivos con el codigo
set(_srcs
  main.cpp
  onfacedetection.h
  onfacedetection.cpp
)

#Al ser local el modulo se creara como una libreria (.so) que se podra descargar en el nao, si se elige remoto se crea un ejecutable
if(ONFACEDETECTION_IS_REMOTE)
  add_definitions(" -DONFACEDETECTION_IS_REMOTE ")
  qi_create_bin(onfacedetection ${_srcs})
else()
  qi_create_lib(onfacedetection SHARED ${_srcs} SUBFOLDER naoqi)
endif()

#Se definen las otras librerias que se utilizaran
qi_use_lib(onfacedetection ALCOMMON ALPROXIES ALTHREAD)
