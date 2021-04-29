%%En este test se observa la diferencia del ruido en las observaciones con
%%el valor real

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

%%Modelo discreto
FI=[1,-DeltaT;0,1];
GAMMA=[-(DeltaT^2)/2;DeltaT];

%Posici�n y velocidad anteriores
Xtk1=[4;0];
%Posici�n y velocidad actuales
Xtk=[4;0];

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
    %Se introduce ruido a las dimensiones de la pelota (a la observaci�n)
    XpelotaActRuido=awgn(r,40);
    YpelotaActRuido=awgn(r,40);
    
    %Estimaci�n del flujo divergente usando triangulos semejantes para
    %el modelo camara objeto. Se promedian los valores "x" y "y" de la
    %pelota
    Dest=(((XpelotaAct-XpelotaAnt)/XpelotaAnt)+((YpelotaAct-YpelotaAnt)/YpelotaAnt))/(2*DeltaT);
    %Estimaci�n con el ruido
    DestRuido=(((XpelotaActRuido-XpelotaAntRuido)/XpelotaAntRuido)+((YpelotaActRuido-YpelotaAntRuido)/YpelotaAntRuido))/(2*DeltaT);
    
    %Gr�fica de la diferencia entre D (flujo divergente estimado) con ruido y D sin ruido
    figure(1)
    title('Diferencia entre D (flujo divergente estimado) con ruido y D sin ruido')
    plot(i*DeltaT, DestRuido-Dest, 'ob')
    axis([0 10 -0.8 0.8])
    hold on
    
    %Gr�fica de la diferencia entre la dimensi�n X de la pelota con ruido y sin ruido
    figure(2)
    title('Diferencia entre la dimensi�n X de la pelota con ruido y sin ruido')
    plot(i*DeltaT, XpelotaAct-XpelotaActRuido, '*b')
    axis([0 10 -0.21 0.21])
    hold on
    
    %Gr�fica de la diferencia entre la dimensi�n Y de la pelota con ruido y sin ruido
    figure(3)
    title('Diferencia entre la dimensi�n Y de la pelota con ruido y sin ruido')
    plot(i*DeltaT, YpelotaAct-YpelotaActRuido, '*b')
    axis([0 10 -0.21 0.21])
    hold on
    
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
    
    %Pausa de 0.1 segundos entre gr�fica y gr�fica
    pause(0.1)
end