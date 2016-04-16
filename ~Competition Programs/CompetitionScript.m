clc, clear
if length(instrfind) > 0
fclose(instrfind);
delete(instrfind);
end

% Create serial port object (SPO)

RomeoCOM = serial('COM3','Baudrate',9600);

% Connect SPO to Romeo

fopen(RomeoCOM);

RGBarray = imread('blue_B1.bmp');

disp('Click on a target rectangle');
targetRGB = ColorPicker(RGBarray)
[ centroidRows,centroidCols,modImage ] = FindAllTargetCentroids( RGBarray,targetRGB );
image(modImage)
for i = 1:6
    hold on
   plot(centroidCols(i),centroidRows(i),'wx') 
end
% Send value to Romeo
% [encoder,distance]

rowVal = centroidRows
colVal = centroidCols

encoderPos = rowVal/10-6;
%----- Reordering targets such that targets-----%
%------closest to 'home' come first and last----%
coordinateVec = [encoderPos',centroidCols'];    %
modVec = sortrows(coordinateVec);               %
coordinateVecOrdered = [modVec(1,:);            %
                        modVec(end,:);          %
                        modVec(end-1,:);        %
                        modVec(end-2,:);        %
                        modVec(end-3,:);        %
                        modVec(end-4,:)];       %
                                                %
encoderPos = coordinateVecOrdered(:,1);         %
colVal = coordinateVecOrdered(:,2);             %
%------------------------------------------------
xTarget_mm = colVal+650;
xTarget_HB = floor(xTarget_mm/256);
xTarget_LB = xTarget_mm - 256 * xTarget_HB;

for k = 1:length(encoderPos)
    fwrite(RomeoCOM, encoderPos(k));

    fwrite(RomeoCOM, xTarget_HB(k));

    fwrite(RomeoCOM, xTarget_LB(k));

% Read message from Romeo
 while RomeoCOM.BytesAvailable == 0
      
 end

 
% message = fscanf(RomeoCOM);

% Display message to CW

% disp(message);
end
 
 while (1 == 1) 
     if RomeoCOM.BytesAvailable
        message = fscanf(RomeoCOM);
        message = message(1:end-2);
        if length(message) == 0
            break
        else
            disp(message);
        end    
     end
 end
% Disconnect SPO from Romeo

fclose(RomeoCOM);