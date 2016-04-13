function [ TLcornerRow,TLcornerCol ] = FindTargetCorner(RGBarray,targetRGB)
%FindTargetCorner outputs the coordinates of the top left corner pixel of
%the target rectangle.
%Matthew Ludlow u0668914


[row,col,layer] = size(RGBarray);
for i = 1:row
    for j = 1:col
        topLeft = [RGBarray(i,j,1) RGBarray(i,j,2) RGBarray(i,j,3)];
        [TrueorFalse] = IsColor2(topLeft, targetRGB);
        if TrueorFalse == 1
            TLcornerRow = i;
            TLcornerCol = j;
            return;
        end
    end
end


end

