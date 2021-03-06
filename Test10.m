%%Este script muestra la aproximación dada por el filtro de Kalman con
%%limitacion del robot Vmax=limiteRobot

%%Parametros para simulación de la pelota
%Tamaño de la pelota
RadioBola=5;
%Distancia de la pelota al foco
DistanciaFocal=1;
%Vector para gráficar el contorno de la pelota
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
while(Xtk(1)>0.5)
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
    
    %Simulación de la pelota acercandose a la cámara
    figure(1)
    plot(XpelotaAct*cos(ang),YpelotaAct*sin(ang))
    axis([-20 20 -20 20])
    title('Pelota y camara')
    
    %Estimación del flujo divergente usando triangulos semejantes para
    %el modelo camara objeto. Se promedian los valores "x" y "y" de la
    %pelota
    Dest=(((XpelotaAct-XpelotaAnt)/XpelotaAnt)+((YpelotaAct-YpelotaAnt)/YpelotaAnt))/(2*DeltaT);
    
    %Gráfica de la estimación del flujo divergente estimado y real y de la
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
    %mas la integral del error entre t y t-1 (usando el método numérico del
    %paralelogramo para la integral)
    ErrorTotal=ErrorTotal+DeltaT*(ErrorAnterior+ErrorActual)/2;
    
    %Gráficas de los errores: Error actual y Error acumulado
    figure(3)
    subplot(2,1,1); plot(i*DeltaT, ErrorActual,'*r')
    axis([0 7 -2 2])
    title('Error Actual')
    hold on
    subplot(2,1,2); plot(i*DeltaT, ErrorTotal,'*r')
    axis([0 7 -2 2])
    title('Error Total')
    hold on
    
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
    
    %Gráfica de la innovación
    figure(4)
    plot(i*DeltaT, (Dest-(Xtk(2)/Xtk(1))), '*b')
    axis([0 7 -1 1])
    title('Innovación')
    hold on
    
    %Actualización de la estimación
    Xtk=Xtk+G*(Dest-(Xtk(2)/Xtk(1)));
    %Actualización de la covarianza
    P=(I-G*H)*P;
    
    %Se limita la velocidad al llegar a la velocidad máxima del robot, la
    %aceleración se hace cero también para tener una estimación correcta
    if(Xtk(2)>=2)
        Xtk(2)=2;
        M=0;
    end
    
    %Gráficas de: Posición, velocidad y aceleración
    figure(5)
    subplot(3,1,1);plot(i*DeltaT, Xtk(1), '*b')
    axis([0 7 0 5])
    title('Posición')
    hold on
    figure(5)
    subplot(3,1,2);plot(i*DeltaT, Xtk(2), '*r')
    axis([0 7 0 5])
    title('Velocidad')
    hold on
    figure(5)
    subplot(3,1,3);plot(i*DeltaT, M, '*r')
    axis([0 7 -5 5])
    title('Aceleración')
    hold on
    
    %Se hace una pausa para que el usuario pueda empezar la simulación
    %cuando quiera y tenga tiempo de ordenar las gráficas
    if(i==1)
        pause
    end
    
    %Pausa de 0.2 segundos entre gráfica y gráfica
    pause(0.2)
end