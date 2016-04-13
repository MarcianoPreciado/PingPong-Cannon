function [ centroidRows,centroidCols,modImage ] = FindAllTargetCentroids( RGBarray,targetRGB )
%FindAllTargetCentroids is a function which locates the center pixel of all
%target rectangles on the image, and it labels those pixels with a white x.
%Matthew Ludlow u0668914
modImage = RGBarray;

    for i = 1:6
        [centroidRow,centroidCol,modImage] = FindTargetCentroid(modImage,targetRGB);
        centroidRows(i) = centroidRow;
        centroidCols(i) = centroidCol;
    end

end

