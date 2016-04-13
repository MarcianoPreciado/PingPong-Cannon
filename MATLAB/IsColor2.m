function [ TrueorFalse1 ] = IsColor2( vector_1, vector_2 )
%IsColor gives a true value if vector_1 = vector_2, and a false value if
%vector_1 does not equal vector_2

TrueorFalse = vector_1 == vector_2;
if ((TrueorFalse(1) == 1) && (TrueorFalse(2) == 1) && (TrueorFalse(3) == 1) )
    TrueorFalse1 = 1;
else
    TrueorFalse1 = 0;
end

end

