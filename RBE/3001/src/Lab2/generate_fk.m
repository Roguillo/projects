%% Step 1: Define your symbolic DH table

clc; clear;
trooper = Robot();

% See sym_example.m for guidance on how to define symbolic variables
% Remember: you CAN freely mix symbolic variables and plain-ol numbers.
% syms a b c d

%% Step 2: Pass your symbolic DH table into dh2fk to get your symbolic 
% FK matrix
% It's really that simple. MATLAB will leave everything unsolved
% Show this to an SA for SIGN-OFF #4

syms a b c d; %our four joint variables

%fk matrix with our joint variables
e = [0,         36.076,   0,        0;
     a,         60.25,    0,       -1.5708;
    -1.3858+b,  0,        130.23,   0;
     1.3858+c,  0,        124,      0;
     d,         0,        133.4,    0];            

% disp(trooper.dh2fk(e));

%% Step 3: Feed your symbolic FK matrix into 'matlabFunction' to turn it
% into a floating point precision function that runs fast.

% Write the fk_3001 function in trooper.m to complete sign-off #5

% Curiosity bonus (0 points): replicate the timeit experiment I did in
% sym_example.m to compare the matlabFunction FK function to just using
% subs to substitute the variables.

%saved the fkMatrix function as its own file
%ht = matlabFunction(fkMatrix, "File", "fkMatrix_to_function") 

% q = [0 0 0 0];
% 
% disp(trooper.fk_3001(q))

%%

syms a b c d; %our four joint variables

%fk matrix with our joint variables
e = [0,         36.076,   0,        0;
     a,         60.25,    0,       -1.5708;
    -1.3858+b,  0,        130.23,   0;
     1.3858+c,  0,        124,      0;
     d,         0,        133.4,    0];            

FK_mat  = trooper.dh2fk(e);
P       = [FK_mat(1:3, 4)];

J_p     = [diff(P(1), a), diff(P(1), b), diff(P(1), c), diff(P(1), d);
           diff(P(2), a), diff(P(2), b), diff(P(2), c), diff(P(2), d);
           diff(P(3), a), diff(P(3), b), diff(P(3), c), diff(P(3), d)];

T_01    = trooper.dh2fk(e(1  , :));
T_02    = trooper.dh2fk(e(1:2, :));
T_03    = trooper.dh2fk(e(1:3, :));
T_04    = trooper.dh2fk(e(1:3, :));

J_o     = [T_01(1:3, 3), ...
           T_02(1:3, 3), ...
           T_03(1:3, 3), ...
           T_04(1:3, 3)];

J = [J_p; J_o];