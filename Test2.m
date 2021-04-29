%%Este script simula el efecto del PI (que utiliza una D constante de 
%%referencia) en el sistema modelado y se varian los valores de Kp, Ki, 
%%Dref para ver sus efectos

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

%Se detiene el proceso al ser la distancia menor a 0.05
while(Xtk(1)>0.05)
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
    
    %Gr�fica de la velocidad (rojo) y de la posici�n (azul)
    figure(1)
    title('Velocidad (rojo) y posicion (azul) vs tiempo')
    plot(i*DeltaT, Xtk(1), '*b')
    axis([0 10 0 5])
    hold on
    figure(1)
    plot(i*DeltaT, Xtk(2), '*r')
    axis([0 10 0 5])
    hold on
    
    %Pausa de 0.1 segundos entre gr�fica y gr�fica
    pause(0.1)
    
    %Contador aumenta
    i=i+1;
end