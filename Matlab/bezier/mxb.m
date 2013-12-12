function [ BAx, BAy ] = mxb( t, ptA , ptB )
%BEZIER Summary of this function goes here
%   Detailed explanation goes here

    BAx = ptA(1) + t.* (ptB(1) - ptA(1)) ;
    BAy = ptA(2) + t.* (ptB(2) - ptA(2)) ;
    
end

