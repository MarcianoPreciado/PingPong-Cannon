function [centroidRow,centroidCol,modImage] = FindTargetCentroid( RGBarray,targetRGB )
%FindTargetCentroid loops through the pixels of the image until reaching
%the color of the desired target, it then outputs the coordinates of the
%centroid of that target.
%Matthew Ludlow u0668914

    
    topRightCoords = [];
    bottomLeftCoords = [];
    [TLcornerRow,TLcornerCol] = FindTargetCorner(RGBarray,targetRGB);
    [row,col,layer] = size(RGBarray);
    i = TLcornerRow;
    for j = TLcornerCol:col
        topRight = [RGBarray(i,j,1) RGBarray(i,j,2) RGBarray(i,j,3)];
        next = [RGBarray(i,j+1,1) RGBarray(i,j+1,2) RGBarray(i,j+1,3)];
        if (next ~= topRight)
            topRightCoords = [i j];
            col = j;
            break;
        end
    end

    j = TLcornerCol;
    for i = TLcornerRow:row;
        bottomLeft = [RGBarray(i,j,1) RGBarray(i,j,2) RGBarray(i,j,3)];
        next = [RGBarray(i+1,j,1) RGBarray(i+1,j,2) RGBarray(i+1,j,3)];
        if (next ~= bottomLeft)
            bottomLeftCoords = [i j];
            row = i;
            break;
        end
    end

    centroidRow = TLcornerRow + round((bottomLeftCoords(1) - topRightCoords(1)) / 2);
    centroidCol = TLcornerCol + round((topRightCoords(2) - bottomLeftCoords(2)) / 2);

    modImage = RGBarray;
    modImage(TLcornerRow:row, TLcornerCol:col, :) = 0;

end

