%%En este script se observa el ruido en el proceso

%%Parametros para simulaci�n de la pelota
%Tama�o de la pelota
RadioBola=5;
%Distancia de la pelota al foco
DistanciaFocal=1;

%Diferencial de tiempo
DeltaT=0.05;
%Constantes del Controlador PI
Kp=6;
Ki=10;
%Flujo divergente deseado
Dref=0.6;
%Flujo divergente estimado
Destim=0;
%Error acumulado entre el flujo deseado y el estimado
ErrorTotal=0;
%Error actual entre el flujo deseado y el estimado
ErrorActual=0;
%Error en t-1 entre el flujo deseado y el estimado
ErrorAnterior=0;

%Errores para las variables con ruido
ErrorTotalRuido=0;
ErrorActualRuido=0;
ErrorAnteriorRuido=0;

%%Modelo discreto
FI=[1,-DeltaT;0,1];
GAMMA=[(DeltaT^2)/2;DeltaT];

%Posici�n y velocidad anteriores
Xtk1=[4;0];
%Posici�n y velocidad actuales
Xtk=[4;0];

%Variables de posici�n y velocidad con ruido
Xtk1Ruido=[4;0];
XtkRuido=[4;0];

%Inicializaci�n del contador
i=0;

%Usando triangulos semejantes se calcula el radio de la pelota
r=DistanciaFocal*RadioBola/Xtk(1);
XpelotaAct=r;
YpelotaAct=r;
%Estas variables se crean para comparar con ruido y sin ruido
XpelotaActRuido=r;
YpelotaActRuido=r;

%Se detiene el proceso al ser la distancia menor a 0.05
while(Xtk(1)>0.05)
    %Contador aumenta
    i=i+1;
    
    %Se guardan los valores anteriores de la pelota
    XpelotaAnt=XpelotaAct;
    YpelotaAnt=YpelotaAct;
    XpelotaAntRuido=XpelotaActRuido;
    YpelotaAntRuido=YpelotaActRuido;
    %Usando triangulos semejantes se calcula el radio de la pelota
    r=DistanciaFocal*RadioBola/Xtk(1);
    XpelotaAct=r;
    YpelotaAct=r;
    %C�lculo del radio introduciendo ruido del proceso
    rRuido=DistanciaFocal*RadioBola/XtkRuido(1);
    XpelotaActRuido=awgn(rRuido,40);
    YpelotaActRuido=awgn(rRuido,40);
    
    %Estimaci�n del flujo divergente usando triangulos semejantes para
    %el modelo camara objeto. Se promedian los valores "x" y "y" de la
    %pelota
    Dest=(((XpelotaAct-XpelotaAnt)/XpelotaAnt)+((YpelotaAct-YpelotaAnt)/YpelotaAnt))/(2*DeltaT);
    %Estimaci�n con el ruido
    DestRuido=(((XpelotaActRuido-XpelotaAntRuido)/XpelotaAntRuido)+((YpelotaActRuido-YpelotaAntRuido)/YpelotaAntRuido))/(2*DeltaT);
    
    %Se guarda el error anterior
    ErrorAnterior=ErrorActual;
    %Se calcula el error actual
    ErrorActual=Dref-(Xtk1(2)/Xtk1(1));
    %Se calcula el error acumulado sumando el error acumulado anterior
    %mas la integral del error entre t y t-1 (usando el m�todo num�rico del
    %paralelogramo para la integral)
    ErrorTotal=ErrorTotal+DeltaT*(ErrorAnterior+ErrorActual)/2;
    %C�lculo del controlador PI
    M=(Kp*ErrorActual+Ki*ErrorTotal);
    
    %Se guarda la velocidad y posici�n anterior
    Xtk1=Xtk;
    %Estimaci�n de la velocidad y posici�n actual
    Xtk=FI*Xtk1+GAMMA*M;
    
    %C�lculo de los errores tomando en cuenta el ruido del proceso
    ErrorAnteriorRuido=ErrorActualRuido;
    ErrorActualRuido=Dref-(Xtk1Ruido(2)/Xtk1Ruido(1));
    ErrorTotalRuido=ErrorTotalRuido+DeltaT*(ErrorAnteriorRuido+ErrorActualRuido)/2;
    MRuido=(Kp*ErrorActualRuido+Ki*ErrorTotalRuido);
    
    %Estimaciones con ruido del proceso
    Xtk1Ruido=XtkRuido;
    XtkRuido=FI*Xtk1Ruido+GAMMA*MRuido;
    %Se introduce ruido al proceso 
    XtkRuido=awgn(XtkRuido,40);
    
    %Gr�fica de velocidad (rojo) y posici�n (azul) sin ruido
    figure(1)
    title('Velocidad (rojo) y posici�n (azul) sin ruido')
    plot(i*DeltaT, Xtk(1), 'ob')
    axis([0 10 0 6])
    hold on
    figure(1)
    plot(i*DeltaT, Xtk(2), 'or')
    axis([0 10 0 6])
    hold on
    
    %Gr�fica de velocidad (rojo) y posici�n (azul) con ruido
    figure(2)
    title('Velocidad (rojo) y posici�n (azul) con ruido')
    plot(i*DeltaT, XtkRuido(1), '*b')
    axis([0 10 0 6])
    hold on
    figure(2)
    plot(i*DeltaT, XtkRuido(2), '*r')
    axis([0 10 0 6])
    hold on
    
    %Pausa de 0.1 segundos entre gr�fica y gr�fica
    pause(0.1)
end