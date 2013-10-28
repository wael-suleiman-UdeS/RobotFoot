function NoError = VerifyError(varargin)
% Return true if ErrorThreshold > norm(ValueError)
%
% Parameter N : ErrorThreshold
% Parameter N+1 : ErrorValue

    if(nargin == 0 || mod(nargin,2) == 1)
        error('Unexpected Input')
    else
        NoError = true;
        LoopNb = (nargin)/2;
        
        for i = 1:LoopNb
            ErrorThreshold = varargin{(i*2)-1};
            ErrorValue = varargin{(i*2)};
            
            if norm(ErrorValue) > ErrorThreshold
                NoError = false;
                break
            end
        end
    end
        
end
