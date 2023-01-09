function [Ed,Eb] = calerror(Xcw,Ycw,Xccw,Yccw,L,b0)
%Calculating Eb & Ed based on clockwise and counter-clockwise results of X
%and Y

beta_x = ((Xcw - Xccw)/(-4*L)) * ((180)/pi);
beta_y = ((Ycw + Yccw)/(-4*L)) * ((180)/pi);

beta_x = ((Xcw - Xccw))/((-4*L));
beta_y = ((Ycw + Yccw))/((-4*L));

beta = 0.5 * (beta_x + beta_y);
R = (L/2)/(beta/2);
Ed = (R + b0/2)/(R - b0/2)


alpha_x = (Xcw + Xccw)/(-4*L) * ((180)/pi);
alpha_y = (Ycw - Yccw)/(-4*L) * ((180)/pi);
alpha = 0.5 * (alpha_x + alpha_y);
Eb = 90/(90-alpha)
end

