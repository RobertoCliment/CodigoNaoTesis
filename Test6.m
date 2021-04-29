%%Este script muestra la aproximaci�n dada por el filtro de Kalman

%%Parametros para simulaci�n de la pelota
%Tama�o de la pelota
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
%Inicializaci�n de la matriz de covarianzas
P=[0,0;0,0];
%Inicializaci�n del jacobiano de la funci�n del proceso
F=[0,0;0,0];
%Inicializaci�n del jacobiano de la funci�n de observaci�n
H=[0,0];
%Inicializaci�n de la ganancia de Kalman
G=[0;0];

%Matriz de covarianzas del ruido en el proceso (Se supone un ruido
%gaussiano blanco)
Q=[0.00000001,0;0,0.00000001];
%Varianza del ruido en la observaci�n
R=0.0001;

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

%Se detiene el proceso al ser la distancia menor a 0.05
while(Xtk(1)>0.2)
    %Contador aumenta
    i=i+1;
    
    %Se guardan los valores anteriores de la pelota
    XpelotaAnt=XpelotaAct;
    YpelotaAnt=YpelotaAct;
    %Usando triangulos semejantes se calcula el radio de la pelota
    %Se introduce el ruido de observaci�n
    r=awgn(DistanciaFocal*RadioBola/Xtk(1),40);
    XpelotaAct=r;
    YpelotaAct=r;
    
    %Estimaci�n del flujo divergente usando triangulos semejantes para
    %el modelo camara objeto. Se promedian los valores "x" y "y" de la
    %pelota
    Dest=(((XpelotaAct-XpelotaAnt)/XpelotaAnt)+((YpelotaAct-YpelotaAnt)/YpelotaAnt))/(2*DeltaT);
    
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
    %Se introduce ruido al proceso
    Xtk=awgn(Xtk,40);
    
    %Se calculan las derivadas parciales del proceso f y de la observaci�n
    %h evaluadas en el momento t (jacobiano)
    F=[1,-DeltaT;0,1];%+[((DeltaT^2)*Kp*Xtk(2))/(2*(Xtk(1)^2)),-(DeltaT^2)*Ki*DeltaT/(2*Xtk(1));-DeltaT*Kp*Xtk(2)/(Xtk(1)^2),Ki*(DeltaT^2)/Xtk(1)];
    H=[-Xtk(2)/(Xtk(1)^2),1/Xtk(1)];
    
    %C�lculo de la covarianza
    P=F*P*transpose(F)+Q;
    %C�lculo de la ganancia de Kalman
    G=P*transpose(H)/(H*P*transpose(H)+R);
    %Actualizaci�n de la estimaci�n
    Xtk=Xtk+G*(Dest-Xtk(2)/Xtk(1));
    %Actualizaci�n de la covarianza
    P=(I-G*H)*P;
    
    %Gr�fica de posici�n y velocidad con filtro de Kalman
    figure(1)
    title('Posici�n (azul) y velocidad (rojo) con filtro de Kalman')
    plot(i*DeltaT, Xtk(1), '*b')
    axis([0 10 0 5])
    hold on
    figure(1)
    plot(i*DeltaT, Xtk(2), '*r')
    axis([0 10 0 5])
    hold on
    
    %Pausa de 0.1 segundos entre gr�fica y gr�fica
    pause(0.1)
end