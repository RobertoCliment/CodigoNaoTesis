%%Este script muestra la aproximaci�n dada por el filtro de Kalman y todas
%%las gr�ficas

%%Parametros para simulaci�n de la pelota
%Tama�o de la pelota
RadioBola=5;
%Distancia de la pelota al foco
DistanciaFocal=1;
%Vector para gr�ficar el contorno de la pelota
ang=0:0.01:2*pi;

%Diferencial de tiempo
DeltaT=0.05;
%Constantes del Controlador PI
Kp=6;
Ki=10;
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
while(Xtk(1)>0.4)
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
    
    %Simulaci�n de la pelota acercandose a la c�mara
    figure(1)
    plot(XpelotaAct*cos(ang),YpelotaAct*sin(ang))
    axis([-20 20 -20 20])
    title('Pelota y camara')
    
    %Estimaci�n del flujo divergente usando triangulos semejantes para
    %el modelo camara objeto. Se promedian los valores "x" y "y" de la
    %pelota
    Dest=(((XpelotaAct-XpelotaAnt)/XpelotaAnt)+((YpelotaAct-YpelotaAnt)/YpelotaAnt))/(2*DeltaT);
    
    %Gr�fica de la estimaci�n del flujo divergente estimado y real y de la
    %diferencia entre ambas
    figure(2)
    subplot(2,1,1); plot(i*DeltaT, Dest, '*b', i*DeltaT, Xtk(2)/Xtk(1), '*r')
    axis([0 7 -2 2])
    title('Destimado (azul) vs Dreal (rojo)')
    hold on
    subplot(2,1,2); plot(i*DeltaT, Dest-(Xtk(2)/Xtk(1)),'*r')
    axis([0 7 -1 1])
    title('Destimado-Dreal')
    hold on
    
    %Se guarda el error anterior
    ErrorAnterior=ErrorActual;
    %Se calcula el error actual
    ErrorActual=Dref-(Xtk1(2)/Xtk1(1));
    %Se calcula el error acumulado sumando el error acumulado anterior
    %mas la integral del error entre t y t-1 (usando el m�todo num�rico del
    %paralelogramo para la integral)
    ErrorTotal=ErrorTotal+DeltaT*(ErrorAnterior+ErrorActual)/2;
    
    %Gr�ficas de los errores: Error actual y Error acumulado
    figure(3)
    subplot(2,1,1); plot(i*DeltaT, ErrorActual,'*r')
    axis([0 7 -2 2])
    title('Error Actual')
    hold on
    subplot(2,1,2); plot(i*DeltaT, ErrorTotal,'*r')
    axis([0 7 -2 2])
    title('Error Total')
    hold on
    
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
    
    %Gr�fica de la innovaci�n
    figure(4)
    plot(i*DeltaT, (Dest-(Xtk(2)/Xtk(1))), '*b')
    axis([0 7 -1 1])
    title('Innovaci�n')
    hold on
    
    %Actualizaci�n de la estimaci�n
    Xtk=Xtk+G*(Dest-(Xtk(2)/Xtk(1)));
    %Actualizaci�n de la covarianza
    P=(I-G*H)*P;
    
    %Gr�ficas de: Posici�n, velocidad y aceleraci�n
    figure(5)
    subplot(3,1,1);plot(i*DeltaT, Xtk(1), '*b')
    axis([0 7 0 5])
    title('Posici�n')
    hold on
    figure(5)
    subplot(3,1,2);plot(i*DeltaT, Xtk(2), '*r')
    axis([0 7 0 5])
    title('Velocidad')
    hold on
    figure(5)
    subplot(3,1,3);plot(i*DeltaT, M, '*r')
    axis([0 7 -5 5])
    title('Aceleraci�n')
    hold on
    
    %Se hace una pausa para que el usuario pueda empezar la simulaci�n
    %cuando quiera y tenga tiempo de ordenar las gr�ficas
    if(i==1)
        pause
    end
    
    %Pausa de 0.2 segundos entre gr�fica y gr�fica
    pause(0.2)
end