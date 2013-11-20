clc
clear all
close all

load matrixQ.txt
%load matrixPositions.txt
load ePos1.txt
load ePos2.txt
load eTheta1.txt
load eTheta2.txt

dt = 0.01;

figure
plot(matrixQ)

Qdot = diff(matrixQ)/dt;

figure
plot(Qdot)

figure
plot(ePos1)

figure
plot(ePos2)

figure
plot(eTheta1)

figure
plot(eTheta2)
