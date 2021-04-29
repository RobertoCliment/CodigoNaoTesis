%%Este script muestra la aproximación dada por el filtro de Kalman

%%Parametros para simulación de la pelota
%Tamaño de la pelota
RadioBola=5;
%Distancia de la pelota al foco
DistanciaFocal=1;

%Diferencial de tiempo
DeltaT=0.05;
%Constantes del Controlador PI
Kp=6;
Ki=0;
%Flujo divergente deseado
Dref=0.6;
%Flujo divergente estimado
Dest=0;
%Error acumulado entre el flujo deseado y el estimado
ErrorTotal=0;
%Error actual entre el flujo deseado y el estimado
ErrorActual=0;
%Error en t-1 entre el flujo deseado y el estimado
ErrorAnterior=0;

%%Modelo discreto
FI=[1,-DeltaT;0,1];
GAMMA=[-(DeltaT^2)/2;DeltaT];

%Matriz identidad
I=[1,0;0,1];
%Inicialización de la matriz de covarianzas
P=[0,0;0,0];
%Inicialización del jacobiano de la función del proceso
F=[0,0;0,0];
%Inicialización del jacobiano de la función de observación
H=[0,0];
%Inicialización de la ganancia de Kalman
G=[0;0];

%Matriz de covarianzas del ruido en el proceso (Se supone un ruido
%gaussiano blanco)
Q=[0.00000001,0;0,0.00000001];
%Varianza del ruido en la observación
R=0.0001;

%Posición y velocidad anteriores
Xtk1=[4;0];
%Posición y velocidad actuales
Xtk=[4;0];

%Inicialización del contador
i=0;

%Usando triangulos semejantes se calcula el radio de la pelota
r=DistanciaFocal*RadioBola/Xtk(1);
XpelotaAct=r;
YpelotaAct=r;

%Se detiene el proceso al ser la distancia menor a 0.05
while(Xtk(1)>0.2)
    %Contador aumenta
    i=i+1;
    
    %Se guardan los valores anteriores de la pelota
    XpelotaAnt=XpelotaAct;
    YpelotaAnt=YpelotaAct;
    %Usando triangulos semejantes se calcula el radio de la pelota
    %Se introduce el ruido de observación
    r=awgn(DistanciaFocal*RadioBola/Xtk(1),40);
    XpelotaAct=r;
    YpelotaAct=r;
    
    %Estimación del flujo divergente usando triangulos semejantes para
    %el modelo camara objeto. Se promedian los valores "x" y "y" de la
    %pelota
    Dest=(((XpelotaAct-XpelotaAnt)/XpelotaAnt)+((YpelotaAct-YpelotaAnt)/YpelotaAnt))/(2*DeltaT);
    
    %Se guarda el error anterior
    ErrorAnterior=ErrorActual;
    %Se calcula el error actual
    ErrorActual=Dref-(Xtk1(2)/Xtk1(1));
    %Se calcula el error acumulado sumando el error acumulado anterior
    %mas la integral del error entre t y t-1 (usando el método numérico del
    %paralelogramo para la integral)
    ErrorTotal=ErrorTotal+DeltaT*(ErrorAnterior+ErrorActual)/2;
    %Cálculo del controlador PI
    M=(Kp*ErrorActual+Ki*ErrorTotal);
    
    %Se guarda la velocidad y posición anterior
    Xtk1=Xtk;
    %Estimación de la velocidad y posición actual
    Xtk=FI*Xtk1+GAMMA*M;
    %Se introduce ruido al proceso
    Xtk=awgn(Xtk,40);
    
    %Se calculan las derivadas parciales del proceso f y de la observación
    %h evaluadas en el momento t (jacobiano)
    F=[1,-DeltaT;0,1];%+[((DeltaT^2)*Kp*Xtk(2))/(2*(Xtk(1)^2)),-(DeltaT^2)*Ki*DeltaT/(2*Xtk(1));-DeltaT*Kp*Xtk(2)/(Xtk(1)^2),Ki*(DeltaT^2)/Xtk(1)];
    H=[-Xtk(2)/(Xtk(1)^2),1/Xtk(1)];
    
    %Cálculo de la covarianza
    P=F*P*transpose(F)+Q;
    %Cálculo de la ganancia de Kalman
    G=P*transpose(H)/(H*P*transpose(H)+R);
    %Actualización de la estimación
    Xtk=Xtk+G*(Dest-Xtk(2)/Xtk(1));
    %Actualización de la covarianza
    P=(I-G*H)*P;
    
    %Gráfica de posición y velocidad con filtro de Kalman
    figure(1)
    title('Posición (azul) y velocidad (rojo) con filtro de Kalman')
    plot(i*DeltaT, Xtk(1), '*b')
    axis([0 10 0 5])
    hold on
    figure(1)
    plot(i*DeltaT, Xtk(2), '*r')
    axis([0 10 0 5])
    hold on
    
    %Pausa de 0.1 segundos entre gráfica y gráfica
    pause(0.1)
end