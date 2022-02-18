function xDot = Problem2_5(t, x, u, m1, m2, Kw, Ks, B)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
Input = u;
State = x;
x1Dot = -(B/m1) * State(1) + (B/m1) * State(2) ...
    - (1/m1) * State(3) + (1/m1) * State(4);
x2Dot = (B/m2) * State(1) - (B/m2) * State(2) ...
    - (1/m2) * State(4);
x3Dot = Kw * State(1) - Kw * Input;
x4Dot = -Ks * State(1) + Ks * State(2);
xDot = [x1Dot; x2Dot; x3Dot; x4Dot];
end