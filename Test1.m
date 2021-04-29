%%Este script simula el acercamiento de una pelota a una camara

%%Parametros para simulaci�n de la pelota
%Tama�o de la pelota
RadioBola=5;
%Distancia inicial entre el robot y la pelota
Dist=20;
%Distancia en el tiempo t entre el robot y la pelota
DistActual=Dist;
%Tama�o de la c�mara
AnchoCamara=1;
%Distancia de la pelota al foco
DistanciaFocal=1;

%%Valores de distancia y tiempo para graficar la posici�n de la pelota
%%con respecto al tiempo
VectorDist=Dist:-1:0;
VectorTiempo=0:1:20;

%%Vector para gr�ficar el contorno de la pelota
ang=0:0.01:2*pi;

%%Gr�fica del movimiento (lineal)
figure(1)
plot(VectorDist,VectorTiempo)
title('Distancia Vs Tiempo')
pause(0.1)

%Gr�fica de la pelota
figure('Position', [50 50 400 400])
for i=0:0.1:Dist
    %%La disancia actual es Dist-i;
    DistActual=Dist-i;
    
    %Usando triangulos semejantes se calcula el radio de la pelota
    r=DistanciaFocal*RadioBola/DistActual;
    %Se crea un vector de los valores x y y de la pelota para graficarse
    Xpelota=r*cos(ang);
    Ypelota=r*sin(ang);
    plot(Xpelota,Ypelota)
    axis([-20 20 -20 20])
    title('Pelota acercandose a camara')
    
    %Pausa de 0.1 segundos entre gr�fica y gr�fica
    pause(0.1)
end

%Cerrar todo
close all
%Eliminar las variables
clear