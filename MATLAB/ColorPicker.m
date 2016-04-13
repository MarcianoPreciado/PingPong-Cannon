function [ RGB ] = ColorPicker( RGBimage )
%ColorPicker opens an RGB image for the user to select a pixel from
%Matthew Ludlow u0668914

image(RGBimage)
axis image
coords = ginput(1);
col = round(coords(1));
row = round(coords(2));
RGB(1) = RGBimage(row,col,1);
RGB(2) = RGBimage(row,col,2);
RGB(3) = RGBimage(row,col,3);

end

