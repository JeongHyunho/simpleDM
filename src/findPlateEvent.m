function Idx = findPlateEvent(plaData, th, winSize, flag)

if strcmp(flag, 'HS')
    Idx = find((plaData <= th).*([plaData(2:end); 0] > th));
    Idx = minmax(Idx, winSize, 'up');
elseif strcmp(flag, 'TO')
    Idx = find((plaData <= th).*([0; plaData(1:end-1)] > th));
    Idx = minmax(Idx, winSize, 'down');
end

    function idxNew = minmax(idxOld,win,dir)
        idxNew = [];
        if nargin ==2
            dir='mean';
        end
        while ~isempty(idxOld)
            idxIn = find((idxOld>=idxOld(1)-win).*(idxOld<=idxOld(1)+win));
            if strcmp(dir,'up')
                idxNew = [idxNew; max(idxOld(idxIn))];
            elseif strcmp(dir,'down')
                idxNew = [idxNew; min(idxOld(idxIn))];
            elseif strcmp(dir,'mean')
                idxNew = [idxNew; mean(idxOld(idxIn))];
            end
            idxOld(idxIn) = [];
        end
        idxNew = round(idxNew);
    end
end