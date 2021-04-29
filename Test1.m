%%Este script simula el acercamiento de una pelota a una camara

%%Parametros para simulación de la pelota
%Tamaño de la pelota
RadioBola=5;
%Distancia inicial entre el robot y la pelota
Dist=20;
%Distancia en el tiempo t entre el robot y la pelota
DistActual=Dist;
%Tamaño de la cámara
AnchoCamara=1;
%Distancia de la pelota al foco
DistanciaFocal=1;

%%Valores de distancia y tiempo para graficar la posición de la pelota
%%con respecto al tiempo
VectorDist=Dist:-1:0;
VectorTiempo=0:1:20;

%%Vector para gráficar el contorno de la pelota
ang=0:0.01:2*pi;

%%Gráfica del movimiento (lineal)
figure(1)
plot(VectorDist,VectorTiempo)
title('Distancia Vs Tiempo')
pause(0.1)

%Gráfica de la pelota
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
    
    %Pausa de 0.1 segundos entre gráfica y gráfica
    pause(0.1)
end

%Cerrar todo
close all
%Eliminar las variables
clear