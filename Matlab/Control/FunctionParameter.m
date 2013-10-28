function param = FunctionParameter(qi,qf,tf)

param = [0 0 0 0]';
     
  param(1) = (2*(qi-qf))./(tf^3);  
  param(2) = -(3/2)*tf*param(1);
  param(3) = 0;
  param(4) = qi;

end
